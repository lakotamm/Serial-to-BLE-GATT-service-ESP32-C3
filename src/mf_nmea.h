// mf_nmea.h - builds $GPGGA / $GPRMC from an MfGps frame (used on the ESP32-C3).
//
// madflight's GPS state carries GPS week + time-of-week, not UTC, so the UTC time and
// date are derived here. GPS time runs ahead of UTC by the accumulated leap seconds.
// That offset is 18 s since 2017. If a leap second is ever added, change the constant.
//
// The 'GPS' and 'SSP' BLE characteristics carry GGA + RMC back to back (CRLF included),
// which is the "183 char" in the Vector Vario documentation.

#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "mf_ble_proto.h"

#define MF_GPS_LEAP_SECONDS 18

struct MfUtc { int year, month, day, hour, min, sec, cs; };

static inline void mf_gps_to_utc(uint16_t week, uint32_t tow_ms, MfUtc* u) {
  const int64_t GPS_EPOCH_UNIX = 315964800LL;   // 1980-01-06 00:00:00 UTC
  int64_t unix_s = GPS_EPOCH_UNIX + (int64_t)week * 604800LL + (int64_t)(tow_ms / 1000) - MF_GPS_LEAP_SECONDS;
  int64_t days = unix_s / 86400;
  int64_t rem  = unix_s % 86400;
  u->hour = (int)(rem / 3600);
  u->min  = (int)((rem % 3600) / 60);
  u->sec  = (int)(rem % 60);
  u->cs   = (int)((tow_ms % 1000) / 10);
  // days since 1970-01-01 -> civil date (Howard Hinnant's algorithm)
  days += 719468;
  int64_t era = (days >= 0 ? days : days - 146096) / 146097;
  unsigned doe = (unsigned)(days - era * 146097);
  unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  int64_t y = (int64_t)yoe + era * 400;
  unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  unsigned mp = (5 * doy + 2) / 153;
  unsigned d = doy - (153 * mp + 2) / 5 + 1;
  unsigned m = mp < 10 ? mp + 3 : mp - 9;
  u->year  = (int)(y + (m <= 2));
  u->month = (int)m;
  u->day   = (int)d;
}

// XOR of every byte between '$' and '*'
static inline uint8_t mf_nmea_checksum(const char* s) {
  uint8_t cs = 0;
  for (const char* p = s + 1; *p && *p != '*'; p++) cs ^= (uint8_t)*p;
  return cs;
}

// Appends "*CS\r\n" to a sentence already in buf (without checksum). Returns the new length, 0 on overflow.
static inline size_t mf_nmea_finish(char* buf, size_t cap, int len) {
  if (len < 0 || (size_t)len + 6 >= cap) return 0;
  int n = snprintf(buf + len, cap - len, "*%02X\r\n", mf_nmea_checksum(buf));
  return (n > 0) ? (size_t)len + (size_t)n : 0;
}

// lat/lon in 1e-7 deg -> "ddmm.mmmmm,N" / "dddmm.mmmmm,E". Integer math only (no float rounding).
static inline void mf_fmt_lat(char* o, size_t n, int32_t e7) {
  int64_t a = e7 < 0 ? -(int64_t)e7 : (int64_t)e7;
  int deg = (int)(a / 10000000); int64_t rem = a % 10000000;
  int64_t min_e5 = rem * 6 / 10;                     // minutes * 1e5
  snprintf(o, n, "%02d%02d.%05d,%c", deg, (int)(min_e5 / 100000), (int)(min_e5 % 100000), e7 < 0 ? 'S' : 'N');
}
static inline void mf_fmt_lon(char* o, size_t n, int32_t e7) {
  int64_t a = e7 < 0 ? -(int64_t)e7 : (int64_t)e7;
  int deg = (int)(a / 10000000); int64_t rem = a % 10000000;
  int64_t min_e5 = rem * 6 / 10;
  snprintf(o, n, "%03d%02d.%05d,%c", deg, (int)(min_e5 / 100000), (int)(min_e5 % 100000), e7 < 0 ? 'W' : 'E');
}

// Geoid separation is left empty on purpose (valid NMEA) - madflight's "undulation" sign
// convention is not worth guessing at, and no consumer here needs it.
static inline size_t mf_format_gga(char* out, size_t cap, const MfGps& g) {
  MfUtc u; mf_gps_to_utc(g.gps_week, g.tow_ms, &u);
  char lat[20], lon[20]; mf_fmt_lat(lat, sizeof(lat), g.lat_e7); mf_fmt_lon(lon, sizeof(lon), g.lon_e7);
  int q = (g.fix >= 4) ? 2 : (g.fix >= 2 ? 1 : 0);
  int len = snprintf(out, cap, "$GPGGA,%02d%02d%02d.%02d,%s,%s,%d,%02d,%.1f,%.1f,M,,M,,",
                     u.hour, u.min, u.sec, u.cs, lat, lon, q, (int)g.sat, g.hdop / 100.0, g.alt_mm / 1000.0);
  return mf_nmea_finish(out, cap, len);
}

static inline size_t mf_format_rmc(char* out, size_t cap, const MfGps& g) {
  MfUtc u; mf_gps_to_utc(g.gps_week, g.tow_ms, &u);
  char lat[20], lon[20]; mf_fmt_lat(lat, sizeof(lat), g.lat_e7); mf_fmt_lon(lon, sizeof(lon), g.lon_e7);
  double knots = g.sog_mms * 0.001 * 1.943844;
  double course = g.cog_e5 * 1e-5;
  int len = snprintf(out, cap, "$GPRMC,%02d%02d%02d.%02d,A,%s,%s,%.1f,%.1f,%02d%02d%02d,,,A",
                     u.hour, u.min, u.sec, u.cs, lat, lon, knots, course, u.day, u.month, u.year % 100);
  return mf_nmea_finish(out, cap, len);
}
