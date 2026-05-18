#pragma once

// Minimal tNMEA0183Stream backed by an ESP-IDF UART. The NMEA0183 library
// exposes a virtual tNMEA0183Stream class for non-Arduino builds whose only
// hard requirements are read() and write(). One UART -> one stream.

#include <stdint.h>
#include <stddef.h>
#include "driver/uart.h"
#include "NMEA0183Stream.h"

class tUartStream : public tNMEA0183Stream {
public:
    explicit tUartStream(uart_port_t port) : _port(port) {}

    int available() override
    {
        size_t n = 0;
        uart_get_buffered_data_len(_port, &n);
        return (int)n;
    }

    int read() override
    {
        uint8_t b;
        int n = uart_read_bytes(_port, &b, 1, 0);   // non-blocking
        return (n == 1) ? b : -1;
    }

    size_t write(const uint8_t *data, size_t size) override
    {
        int n = uart_write_bytes(_port, (const char*)data, size);
        return (n < 0) ? 0 : (size_t)n;
    }

private:
    uart_port_t _port;
};
