// EML-2 magnetic log → NMEA 2000 bridge for ESP32-C3
//
// Handled sentences
//   $IIVHW → PGN 128259  Speed, Water Referenced  (m/s, electromagnetic type)
//   $IIVLW → PGN 128275  Distance Log             (total + trip, in metres)
//
// VHW sentence layout:  $IIVHW,a,T,b,M,c,N,d,K*hh
//   field 0 = heading true (degrees)   — may be empty on mag-log-only devices
//   field 2 = heading magnetic (deg)   — may be empty
//   field 4 = speed through water (knots)
//   field 6 = speed through water (km/h)
//
// VLW sentence layout:  $IIVLW,a,N,b,N*hh
//   field 0 = total cumulative log (nautical miles)
//   field 2 = trip log since reset  (nautical miles)

#include <Arduino.h>
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_pm.h"
#include "config.h"

#include <N2kMessages.h>
#include <NMEA0183.h>
#include "NMEA2000_TWAI.h"

// ---------------------------------------------------------------------------
// Debug output — expands to nothing in release build
// ---------------------------------------------------------------------------
#ifdef DEBUG_BUILD
  #define DBG(fmt, ...)  Serial.printf(fmt, ##__VA_ARGS__)
  #define DBGLN(s)       Serial.println(s)
#else
  #define DBG(fmt, ...)  ((void)0)
  #define DBGLN(s)       ((void)0)
#endif

#define KNOTS2MS(k) ((k) * 0.514444)

static tNMEA2000_TWAI NMEA2000(TWAI_TX_PIN, TWAI_RX_PIN);
static TaskHandle_t g_loopTask  = nullptr;
static uint32_t     g_lastVhwMs = 0;   // throttle PGN 
static uint32_t     g_lastVlwMs = 0;   // throttle PGN 128275 to VLW_INTERVAL_MS

// Called by the Arduino UART driver when RX data arrives.
// Unblocks the loop task immediately rather than waiting for the timeout.
static void IRAM_ATTR onNmeaRx() {
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(g_loopTask, &woken);
    portYIELD_FROM_ISR(woken);
}

// ---------------------------------------------------------------------------
// Power reduction
// ---------------------------------------------------------------------------
static void disableUnusedPeripherals()
{
    esp_wifi_stop();
    esp_wifi_deinit();

#if CONFIG_BT_ENABLED
    esp_bt_controller_disable();
#endif

    // 80 MHz is the minimum APB clock for reliable 250 kbps TWAI
    setCpuFrequencyMhz(CPU_FREQ_MHZ);
}

// In release: enable automatic light sleep during FreeRTOS idle.
// Not used in debug because light sleep disconnects USB CDC.
static void enableLightSleep()
{
    esp_pm_config_t pm = {
        .max_freq_mhz       = CPU_FREQ_MHZ,
        .min_freq_mhz       = CPU_FREQ_MIN_MHZ,
        .light_sleep_enable = true
    };
    esp_pm_configure(&pm);
}

// ---------------------------------------------------------------------------
// NMEA 0183 input
// ---------------------------------------------------------------------------
static tNMEA0183 NMEA0183;

// ---------------------------------------------------------------------------
// Field helper
// ---------------------------------------------------------------------------
static double fieldDouble(const tNMEA0183Msg &msg, int idx)
{
    const char *f = msg.Field(idx);
    if (!f || f[0] == '\0') return N2kDoubleNA;
    return atof(f);
}

// ---------------------------------------------------------------------------
// N2K senders
// ---------------------------------------------------------------------------

// PGN 128259 — Speed, Water Referenced
static void sendWaterSpeed(double speedKnots)
{
    if (speedKnots == N2kDoubleNA) return;
    tN2kMsg n2kMsg;
    SetN2kBoatSpeed(n2kMsg, 0,
                    KNOTS2MS(speedKnots),
                    N2kDoubleNA,
                    N2kSWRT_Electro_magnetic);
    bool ok = NMEA2000.SendMsg(n2kMsg);
    DBG("[VHW] %.2f kn -> PGN 128259 %s\n", speedKnots, ok ? "ok" : "FAILED");
}

// PGN 128275 — Distance Log
static void sendDistanceLog(double totalNm, double tripNm)
{
    if (totalNm == N2kDoubleNA && tripNm == N2kDoubleNA) return;
    tN2kMsg n2kMsg;
    // Guard against N2kDoubleNA (huge sentinel) overflowing uint32
    uint32_t totalM = (totalNm != N2kDoubleNA) ? (uint32_t)(totalNm * 1852.0) : 0xFFFFFFFFu;
    uint32_t tripM  = (tripNm  != N2kDoubleNA) ? (uint32_t)(tripNm  * 1852.0) : 0xFFFFFFFFu;
    SetN2kDistanceLog(n2kMsg, 0, 0, totalM, tripM);   // no RTC on this device
    bool ok = NMEA2000.SendMsg(n2kMsg);
    DBG("[VLW] total=%.3f nm  trip=%.3f nm -> PGN 128275 %s\n",
        totalNm, tripNm, ok ? "ok" : "FAILED");
}

// ---------------------------------------------------------------------------
// Sentence dispatcher
// ---------------------------------------------------------------------------
static void handleNMEA0183Msg(const tNMEA0183Msg &msg)
{
    uint32_t now = millis();
    if (msg.IsMessageCode("VHW")) 
    {
        if (now - g_lastVhwMs >= VHW_INTERVAL_MS) 
        {
            sendWaterSpeed(fieldDouble(msg, 4));
            g_lastVhwMs = now;
        };
    };

    if (msg.IsMessageCode("VLW")) 
    {
        if (now - g_lastVlwMs >= VLW_INTERVAL_MS) 
        {
            sendDistanceLog(fieldDouble(msg, 0), fieldDouble(msg, 2));
            g_lastVlwMs = now;
        };
    }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup()
{
#ifdef DEBUG_BUILD
    Serial.begin(DEBUG_BAUD);
#endif

    // C3 Super Mini has a WS2812B on GPIO8 — drive it low to kill its idle draw
    pinMode(8, OUTPUT);
    digitalWrite(8, LOW);

    disableUnusedPeripherals();

    DBGLN("[boot] EML-2 N2K bridge starting");
    DBG("[boot] NMEA0183 UART1 RX=GPIO%d  %d baud  invert=%d\n",
        NMEA0183_RX_PIN, NMEA0183_BAUD, (int)NMEA0183_INVERT);
    DBG("[boot] TWAI TX=GPIO%d  RX=GPIO%d\n", TWAI_TX_PIN, TWAI_RX_PIN);

    g_loopTask = xTaskGetCurrentTaskHandle();
    Serial1.begin(NMEA0183_BAUD, SERIAL_8N1, NMEA0183_RX_PIN, -1, NMEA0183_INVERT);
    Serial1.onReceive(onNmeaRx);
    NMEA0183.Begin(&Serial1, 3);

    NMEA2000.SetProductInformation(N2K_SERIAL_NUMBER, N2K_PRODUCT_CODE,
                                   N2K_MODEL_ID, N2K_SW_VERSION, N2K_HW_VERSION);
    NMEA2000.SetDeviceInformation(N2K_DEVICE_UNIQUE, N2K_DEVICE_FUNCTION,
                                  N2K_DEVICE_CLASS, N2K_MFR_CODE);
    NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, 22);
    NMEA2000.EnableForward(false);
    NMEA2000.Open();

#ifndef DEBUG_BUILD
    enableLightSleep();
#endif

    DBGLN("[boot] ready");
}

void loop()
{
    tNMEA0183Msg inMsg;
    if (NMEA0183.GetMessage(inMsg)) {
        handleNMEA0183Msg(inMsg);
    }
    NMEA2000.ParseMessages();

    // Block until the UART RX callback wakes us, or 100 ms passes.
    // 100 ms keeps ParseMessages() running for N2K address-claiming.
    // The CPU enters light sleep for the full wait duration.
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
}
