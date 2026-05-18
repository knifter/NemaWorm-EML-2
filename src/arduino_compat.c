// Tiny Arduino/AVR API shims for pure-IDF builds. Pulled in only because the
// upstream NMEA0183 library's message-encode path calls dtostrf() — even when
// we never construct outbound 0183 sentences. Pure read path doesn't touch it.

#include <stdio.h>
#include "esp_timer.h"

// dtostrf: classic AVR libc routine, %f formatter with fixed width + precision.
// AVR signature: char *dtostrf(double val, signed char width, unsigned char prec, char *s)
char *dtostrf(double val, signed char width, unsigned char prec, char *out)
{
    char fmt[16];
    snprintf(fmt, sizeof(fmt), "%%%d.%df", (int)width, (int)prec);
    sprintf(out, fmt, val);
    return out;
};


// NMEA2000 lib (N2kDef.h) declares `extern uint32_t millis()` on non-Arduino
// builds and expects us to provide it. Same definition serves the local code.
uint32_t millis()
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
};
