#pragma once

// Minimal NMEA 2000 CAN driver using the ESP-IDF TWAI peripheral directly.
// Replaces ttlappalainen/NMEA2000_esp32 which does not compile on ESP32-C3
// because it includes soc/dport_reg.h (Xtensa/ESP32 only).

#include <NMEA2000.h>
#include <string.h>
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"

class tNMEA2000_TWAI : public tNMEA2000 {
public:
    tNMEA2000_TWAI(gpio_num_t txPin, gpio_num_t rxPin)
        : _txPin(txPin), _rxPin(rxPin) {}

protected:
    bool CANOpen() override {
        twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(_txPin, _rxPin, TWAI_MODE_NORMAL);
        twai_timing_config_t  t = TWAI_TIMING_CONFIG_250KBITS();
        twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
        return twai_start() == ESP_OK;
    }

    bool CANSendFrame(unsigned long id, unsigned char len,
                      const unsigned char *buf, bool wait_sent = true) override {
        twai_message_t msg = {};
        msg.extd             = 1;   // NMEA 2000 uses 29-bit extended frames
        msg.identifier       = id;
        msg.data_length_code = len;
        memcpy(msg.data, buf, len);
        return twai_transmit(&msg, wait_sent ? pdMS_TO_TICKS(10) : 0) == ESP_OK;
    }

    bool CANGetFrame(unsigned long &id, unsigned char &len,
                     unsigned char *buf) override {
        twai_message_t msg = {};
        if (twai_receive(&msg, 0) != ESP_OK) return false;  // non-blocking
        id  = msg.identifier;
        len = msg.data_length_code;
        memcpy(buf, msg.data, len);
        return true;
    }

private:
    gpio_num_t _txPin;
    gpio_num_t _rxPin;
};
