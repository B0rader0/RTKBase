#pragma once

int nmea_asprintf(char **strp, const char *fmt, ...);
int nmea_vasprintf(char **strp, const char *fmt, va_list args);
