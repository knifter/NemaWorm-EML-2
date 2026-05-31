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
#include "esp_sleep.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "config.h"
#include "esp_private/periph_ctrl.h"

#include <N2kMessages.h>
#include <NMEA0183.h>
#include "NMEA2000_TWAI.h"

// ---------------------------------------------------------------------------
// Debug output — expands to nothing in release build
// ---------------------------------------------------------------------------
#ifdef DEBUG_BUILD
  #define DBG(fmt, ...)  Serial.printf("[%ld] " fmt, millis(), ##__VA_ARGS__)
#else
  #define DBG(fmt, ...)  ((void)0)
#endif

#define KNOTS2MS(k) ((k) * 0.514444)

static tNMEA2000_TWAI NMEA2000(TWAI_TX_PIN, TWAI_RX_PIN);

// Latest values received from EML-2, cached here and sent on our own schedule
static double    g_speedKn  = N2kDoubleNA;
static double    g_totalNm  = N2kDoubleNA;
static double    g_tripNm   = N2kDoubleNA;

// Keep track of CAN/TWAI sleep/standby state
static bool      g_standby  = true;

// USB connected? -> dont sleep
static bool      g_usbActive = false;

// Next scheduled send times
static uint32_t  g_nextSpeed = 0;
static uint32_t  g_nextLog = 0;


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
};

// Close USB CDC before the first light sleep. Light sleep gates the USB
// peripheral and CDC ends up in a half-broken state on wake anyway, so
// tear it down deliberately rather than letting sleep entry mangle it.
// Compiles to nothing when CDC wasn't enabled at boot (release build).
static void disableUsbCDC()
{
#if ARDUINO_USB_CDC_ON_BOOT
    Serial.flush();
    Serial.end();
    periph_module_disable(PERIPH_USB_DEVICE_MODULE);
#endif
};

void can_wake()
{
    if(!g_standby)
        return;
    digitalWrite(TWAI_SB_PIN, LOW);
    NMEA2000.twaiWake();
    g_standby = false;
};
void can_sleep()
{
    if(g_standby)
        return;
    NMEA2000.twaiSleep();
    digitalWrite(TWAI_SB_PIN, HIGH);
    g_standby = true;
};

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
    DBG("[tx:PGN128259] %.2f kn [%s]\n", speedKnots, ok ? "ok" : "FAILED");
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
    DBG("[tx:PGN128275] total=%.3f nm  trip=%.3f nm [%s]\n",
        totalNm, tripNm, ok ? "ok" : "FAILED");
}

// ---------------------------------------------------------------------------
// Sentence dispatcher — only caches values, never sends directly
// ---------------------------------------------------------------------------
static void handleNMEA0183Msg(const tNMEA0183Msg &msg)
{
    if (msg.IsMessageCode("VHW")) {
        g_speedKn = fieldDouble(msg, 4);
        DBG("[rx:VHW] %.2f kn\n", g_speedKn);
    } else if (msg.IsMessageCode("VLW")) {
        g_totalNm = fieldDouble(msg, 0);
        g_tripNm  = fieldDouble(msg, 2);
        DBG("[rx:VLW] total=%.3f nm  trip=%.3f nm\n", g_totalNm, g_tripNm);
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
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // CAN Transceiver standby pin
    pinMode(TWAI_SB_PIN, OUTPUT);
    digitalWrite(TWAI_SB_PIN, LOW);

    disableUnusedPeripherals();

    DBG("[boot] EML-2 N2K bridge starting\n");
    DBG("[boot] reset_reason=%d\n", (int)esp_reset_reason());
    DBG("[boot] NMEA0183 UART1 RX=GPIO%d  %d baud  invert=%d\n",
        NMEA0183_RX_PIN, NMEA0183_BAUD, (int)NMEA0183_INVERT);
    DBG("[boot] TWAI TX=GPIO%d  RX=GPIO%d\n", TWAI_TX_PIN, TWAI_RX_PIN);

    Serial1.begin(NMEA0183_BAUD, SERIAL_8N1, NMEA0183_RX_PIN, -1, NMEA0183_INVERT);

    // Move UART1 onto XTAL clock so the peripheral survives light sleep —
    // FIFO keeps capturing characters while the CPU is asleep. Default
    // APB clock would be gated on entry to light sleep and we'd lose data.
    uart_config_t cfg = {
        .baud_rate           = NMEA0183_BAUD,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk          = UART_SCLK_XTAL,
    };
    uart_param_config(UART_NUM_1, &cfg);

    // Wake the CPU once ~10 characters' worth of RX edges have accumulated
    uart_set_wakeup_threshold(UART_NUM_1, NMEA0183_WAKEUP_THRESHOLD);
    esp_sleep_enable_uart_wakeup(UART_NUM_1);
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

    if((bool)Serial)
    {
        DBG("[boot] USB-CDC Detected. Disabling sleep.\n");
        g_usbActive = true;
    };
    if(usb_serial_jtag_is_connected())
    {
        DBG("[boot] USB-JTAG Detected. Disabling sleep.\n");
        g_usbActive = true;
    };

    DBG("[boot] ready\n");
}

void loop()
{
    uint32_t now = millis();

    digitalWrite(LED_PIN, LOW);

    if(NMEA2000.handleBusError())
    {
        DBG("[ERROR] Bus error detected, CAN restarted.\n");
    };

    // Drain all buffered NMEA sentences into the cache
    tNMEA0183Msg inMsg;
    while (NMEA0183.GetMessage(inMsg)) 
    {
        handleNMEA0183Msg(inMsg);
    };

    // Send if due — wake TWAI only for the transmission window
    if(now >= g_nextSpeed)
    {
        can_wake();
        sendWaterSpeed(g_speedKn);
        while(now >= g_nextSpeed)
            g_nextSpeed += VHW_INTERVAL_MS;
    };
    if(now >= g_nextLog)
    {
        can_wake();
        sendDistanceLog(g_totalNm, g_tripNm);
        while(now >= g_nextLog)
            g_nextLog += VLW_INTERVAL_MS;
    };
    if(!g_standby)
    {
        NMEA2000.ParseMessages();   // handle N2K address-claiming while bus is active
    };

    // Manual light sleep — bypasses the FreeRTOS auto-sleep machinery which is
    // blocked by PM locks held by the UART and USB Serial/JTAG drivers.
    // Wakes on either the timer expiry (next send) or a UART RX edge.
    uint32_t sleepMs = min(g_nextSpeed - now, g_nextLog - now);

    // Stay awake while a USB host has the CDC port open (DTR asserted) — that's
    // a developer with a terminal or esptool attached. Also stay awake during
    // the post-boot grace window so the NMEA 2000 address claim completes with
    // the controller actually running and listening for counter-claims.
    bool inGrace   = (now < BOOT_GRACE_MS);
    if (g_usbActive || inGrace)
    {
        vTaskDelay(pdMS_TO_TICKS(sleepMs));
        return;
    };

    DBG("usbActive: %d\n", g_usbActive);

    // First commit to sleeping for the rest of this session: USB peripheral
    // is going dark anyway on sleep entry, tear the CDC driver down cleanly.
    static bool cdcDisabled = false;
    if (!cdcDisabled)
    {
        disableUsbCDC();
        cdcDisabled = true;
    };

    digitalWrite(LED_PIN, HIGH);
    can_sleep();
    esp_sleep_enable_timer_wakeup((uint64_t)sleepMs * 1000ULL);
    esp_light_sleep_start();
};
