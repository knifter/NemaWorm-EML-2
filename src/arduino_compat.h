#pragma once

// Force-included into every translation unit (see build_flags: -include).
// Pulls in the few Arduino/AVR symbols upstream libs (NMEA0183, NMEA2000) use
// without declaring. The definitions live in arduino_compat.c / main.cpp.

#ifdef __cplusplus
extern "C" {
#endif

char *dtostrf(double val, signed char width, unsigned char prec, char *out);

#ifdef __cplusplus
}
#endif
