#pragma once

#define LED_PIN             GPIO_NUM_8

// --- NMEA 0183 input ---------------------------------------------------------
// GPIO3 is NOT 5 V tolerant. If tapping the signal at 5 V on the PCB, add a
// voltage divider before the pin (e.g. 10 kΩ series + 20 kΩ to GND → 3.3 V).
#define NMEA0183_RX_PIN             GPIO_NUM_3  // UART1 RX
#define NMEA0183_BAUD               4800
#define NMEA0183_WAKEUP_THRESHOLD   30          // Chars is UART buffer before wake
#define NMEA0183_INVERT             false       // true = signal is logic-inverted (RS-232 style PCB tap)

// --- NMEA 2000 / TWAI output -------------------------------------------------
#define TWAI_TX_PIN         GPIO_NUM_1   // to CAN transceiver TXD
#define TWAI_RX_PIN         GPIO_NUM_0   // from CAN transceiver RXD
#define TWAI_SB_PIN         GPIO_NUM_2   // to CAN transceiver Rs (high = standby)

#define TWAI_RX_QUEUE_LEN   20
#define TWAI_TX_QUEUE_LEN   20

// Transceiver standby->normal settle time, waited after SB is driven low before
// the TX burst is released onto the bus. Adjust to your transceiver's datasheet.
#define TWAI_SB_WAKE_US     50

// --- PGN transmit intervals --------------------------------------------------
#define VHW_INTERVAL_MS     500      // Speed (limited by EML-2 sentence rate)
#define VLW_INTERVAL_MS     5000    // Distance log — changes slowly

// --- NMEA 2000 node identity -------------------------------------------------
#define N2K_SERIAL_NUMBER   "00000001"
#define N2K_PRODUCT_CODE    100
#define N2K_MODEL_ID        "EML-2 N2K Bridge"
#define N2K_SW_VERSION      "1.0.0.0"
#define N2K_HW_VERSION      "1.0.0.0"
#define N2K_DEVICE_UNIQUE   112233  // change per unit when deploying multiples
#define N2K_DEVICE_FUNCTION 140     // Speed
#define N2K_DEVICE_CLASS    60      // Navigation
#define N2K_MFR_CODE        2046    // self-assigned / unknown

// --- CPU frequency -----------------------------------------------------------
// 80 MHz is the minimum APB clock for reliable 250 kbps TWAI.
#define CPU_FREQ_MHZ        80
// Minimum frequency during light sleep idle (release build only).
#define CPU_FREQ_MIN_MHZ    10

// --- Boot grace window -------------------------------------------------------
// Skip light sleep for the first N ms after boot so the NMEA 2000 address
// claim has time to complete with the bus controller actually running.
// USB-host presence drives the longer "stay awake" decision separately.
#define BOOT_GRACE_MS       3000

// --- Debug serial (USB CDC) --------------------------------------------------
#define DEBUG_BAUD          115200
