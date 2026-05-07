#pragma once

// --- NMEA 0183 input ---------------------------------------------------------
// GPIO3 is NOT 5 V tolerant. If tapping the signal at 5 V on the PCB, add a
// voltage divider before the pin (e.g. 10 kΩ series + 20 kΩ to GND → 3.3 V).
#define NMEA0183_RX_PIN     3       // UART1 RX
#define NMEA0183_BAUD       4800
#define NMEA0183_INVERT     false   // true = signal is logic-inverted (RS-232 style PCB tap)

// --- NMEA 2000 / TWAI output -------------------------------------------------
#define TWAI_TX_PIN         GPIO_NUM_1   // to CAN transceiver TXD
#define TWAI_RX_PIN         GPIO_NUM_0   // from CAN transceiver RXD

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

// --- Debug serial (USB CDC) --------------------------------------------------
#define DEBUG_BAUD          115200
