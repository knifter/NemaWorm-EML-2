// EML-2 magnetic log → NMEA 2000 bridge for ESP32-C3 (pure ESP-IDF build)
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>             // fsync()
#include <sys/param.h>          // MIN()

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "hal/usb_serial_jtag_ll.h"
#include "esp_private/periph_ctrl.h"
#include "soc/periph_defs.h"

#include "config.h"
#include "UartStream.h"

#include <N2kMessages.h>
#include <NMEA0183.h>
#include "NMEA2000_TWAI.h"

static const char *TAG = "nemaworm";

// ---------------------------------------------------------------------------
// Debug output — expands to nothing in release build
// ---------------------------------------------------------------------------
#ifdef DEBUG_BUILD
  #define DBG(fmt, ...)  ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#else
  #define DBG(fmt, ...)  ((void)0)
#endif

#define KNOTS2MS(k) ((k) * 0.514444)

static tNMEA2000_TWAI NMEA2000(TWAI_TX_PIN, TWAI_RX_PIN);
static tUartStream    nmea0183Uart(UART_NUM_1);
static tNMEA0183      NMEA0183(&nmea0183Uart, 3);

// Latest values received from EML-2, cached here and sent on our own schedule
static double    g_speedKn  = N2kDoubleNA;
static double    g_totalNm  = N2kDoubleNA;
static double    g_tripNm   = N2kDoubleNA;

// Keep track of CAN/TWAI sleep/standby state
static bool      g_standby  = true;

// USB connected? -> dont sleep
static bool      g_usbActive = false;

// Next scheduled send times
static uint32_t  g_nextVhwMs = 0;
static uint32_t  g_nextVlwMs = 0;


// ---------------------------------------------------------------------------
// Power reduction
// ---------------------------------------------------------------------------
static void disableUnusedPeripherals()
{
    esp_wifi_stop();
    esp_wifi_deinit();


    // 80 MHz is the minimum APB clock for reliable 250 kbps TWAI
    // Pin CPU at a fixed frequency. esp_pm_configure with min==max and light
    // sleep disabled is the IDF equivalent of Arduino's setCpuFrequencyMhz().
    // In release this gets overwritten later by enableLightSleep() with a
    // dynamic min/max + light_sleep_enable=true.
    esp_pm_config_t pm = {
        .max_freq_mhz       = CPU_FREQ_MHZ,
        .min_freq_mhz       = CPU_FREQ_MHZ,
        .light_sleep_enable = false,
    };
    esp_pm_configure(&pm);
};

// In release: enable automatic light sleep during FreeRTOS idle.
// Not used in debug because light sleep gates the USB-Serial-JTAG peripheral.
static void enableLightSleep()
{
    esp_pm_config_t pm = {
        .max_freq_mhz       = CPU_FREQ_MHZ,
        .min_freq_mhz       = CPU_FREQ_MIN_MHZ,
        .light_sleep_enable = true,
    };
    esp_pm_configure(&pm);
};

// SOF (Start-of-Frame) tokens arrive every ~1 ms while a USB host is
// enumerated. Clear the SOF interrupt status, wait, then see if it reasserted.
static bool usbHostConnected()
{
    usb_serial_jtag_ll_clr_intsts_mask(USB_SERIAL_JTAG_INTR_SOF);
    vTaskDelay(pdMS_TO_TICKS(3));
    return (usb_serial_jtag_ll_get_intraw_mask() & USB_SERIAL_JTAG_INTR_SOF) != 0;
};

// Tear down USB-Serial-JTAG before the first light sleep. Flush libc stdio
// buffers, then gate the USB peripheral's clock so it stops drawing current.
// The host sees a disconnect on entry and a fresh enumeration on wake.
//
// In DEBUG builds we keep the peripheral alive so esptool can still reach
// the chip for flashing — once the clock is gated, USB is invisible until a
// hardware reset (boot+EN). Only release pays the development tax.
static void disableUsbCDC()
{
    fflush(stdout);
    fsync(fileno(stdout));
    periph_module_disable(PERIPH_USB_DEVICE_MODULE);
};

void can_wake()
{
    if(!g_standby)
        return;
    gpio_set_level(TWAI_SB_PIN, 0);
    NMEA2000.twaiWake();
    g_standby = false;
};
void can_sleep()
{
    if(g_standby)
        return;
    NMEA2000.twaiSleep();
    gpio_set_level(TWAI_SB_PIN, 1);
    g_standby = true;
};

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
    DBG("[VHW] %.2f kn -> PGN 128259 %s", speedKnots, ok ? "ok" : "FAILED");
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
    DBG("[VLW] total=%.3f nm  trip=%.3f nm -> PGN 128275 %s",
        totalNm, tripNm, ok ? "ok" : "FAILED");
}

// ---------------------------------------------------------------------------
// Sentence dispatcher — only caches values, never sends directly
// ---------------------------------------------------------------------------
static void handleNMEA0183Msg(const tNMEA0183Msg &msg)
{
    if (msg.IsMessageCode("VHW")) {
        g_speedKn = fieldDouble(msg, 4);

    } else if (msg.IsMessageCode("VLW")) {
        g_totalNm = fieldDouble(msg, 0);
        g_tripNm  = fieldDouble(msg, 2);
    }
}

// ---------------------------------------------------------------------------
// app_main
// ---------------------------------------------------------------------------
extern "C" void app_main(void)
{
    // Replace IDF's default LL-polling USB-Serial-JTAG console with the
    // proper driver. The polling console has no bus-reset ISR — once a host
    // disconnects, the peripheral wedges and Windows reports the device as
    // "not functioning" on the next open. The driver hooks bus-reset and
    // survives reconnects cleanly.
    // Non-blocking VFS: writes drop bytes when the host stops reading instead
    // of stalling on a ringbuffer. With the blocking driver path, one host
    // disconnect leaves stale state that makes Windows report "device not
    // functioning" on the next open.
    // TvR: This makes the reconnect better but after missing more chars it still wont reconnect
    usb_serial_jtag_vfs_use_nonblocking();

    // C3 Super Mini has a WS2812B on GPIO8 — drive it low to kill its idle draw
    gpio_set_direction(LED_PIN,     GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);

    // CAN Transceiver standby pin
    gpio_set_direction(TWAI_SB_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(TWAI_SB_PIN, 0);

    disableUnusedPeripherals();

    DBG("[boot] EML-2 N2K bridge starting");
    DBG("[boot] reset_reason=%d", (int)esp_reset_reason());
    DBG("[boot] NMEA0183 UART1 RX=GPIO%d  %d baud  invert=%d",
        NMEA0183_RX_PIN, NMEA0183_BAUD, (int)NMEA0183_INVERT);
    DBG("[boot] TWAI TX=GPIO%d  RX=GPIO%d", TWAI_TX_PIN, TWAI_RX_PIN);

    // UART1 on XTAL clock so the peripheral survives light sleep — the FIFO
    // keeps capturing characters while the CPU is asleep. APB clock would be
    // gated on sleep entry and we'd lose data.
    uart_config_t cfg = {
        .baud_rate           = NMEA0183_BAUD,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk          = UART_SCLK_XTAL,
    };
    uart_driver_install(UART_NUM_1, 256, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &cfg);
    uart_set_pin(UART_NUM_1, UART_PIN_NO_CHANGE, NMEA0183_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (NMEA0183_INVERT)
        uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV);

    // Wake the CPU once ~10 characters' worth of RX edges have accumulated
    uart_set_wakeup_threshold(UART_NUM_1, NMEA0183_WAKEUP_THRESHOLD);
    esp_sleep_enable_uart_wakeup(UART_NUM_1);

    NMEA0183.Open();
    NMEA0183.SetMessageStream(&nmea0183Uart);

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

    if (usbHostConnected())
    {
        DBG("[boot] USB-JTAG host detected. Disabling sleep.");
        g_usbActive = true;
    };

    DBG("[boot] ready");

    bool cdcDisabled = false;

    while (1)
    {
        uint32_t now = millis();

        gpio_set_level(LED_PIN, 0);

        // Drain all buffered NMEA sentences into the cache
        tNMEA0183Msg inMsg;
        while (NMEA0183.GetMessage(inMsg))
        {
            handleNMEA0183Msg(inMsg);
        };

        // Send if due — wake TWAI only for the transmission window
        if(now >= g_nextVhwMs)
        {
            can_wake();
            sendWaterSpeed(g_speedKn);
            g_nextVhwMs = now + VHW_INTERVAL_MS;
        };
        if(now >= g_nextVlwMs)
        {
            can_wake();
            sendDistanceLog(g_totalNm, g_tripNm);
            g_nextVlwMs = now + VLW_INTERVAL_MS;
        };
        if(!g_standby)
        {
            NMEA2000.ParseMessages();   // handle N2K address-claiming while bus is active
        };

        // Manual light sleep — bypasses the FreeRTOS auto-sleep machinery which is
        // blocked by PM locks held by the UART and USB-Serial-JTAG drivers.
        // Wakes on either the timer expiry (next send) or a UART RX edge.
        now = millis();
        uint32_t sleepMs = MIN(g_nextVhwMs - now, g_nextVlwMs - now);

        // Stay awake while a USB host is enumerated (developer with a terminal
        // attached). Also stay awake during the post-boot grace window so the
        // NMEA 2000 address claim completes with the controller running and
        // listening for counter-claims.
        bool inGrace   = (now < BOOT_GRACE_MS);
        if (g_usbActive || inGrace)
        {
            vTaskDelay(pdMS_TO_TICKS(sleepMs));
            continue;
        };

        // First commit to sleeping for the rest of this session: USB peripheral
        // is going dark anyway on sleep entry, flush stdio cleanly.
        if (!cdcDisabled)
        {
            disableUsbCDC();
            cdcDisabled = true;
        };

        gpio_set_level(LED_PIN, 1);
        can_sleep();
        esp_sleep_enable_timer_wakeup((uint64_t)sleepMs * 1000ULL);
        esp_light_sleep_start();
    };
};
