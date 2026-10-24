// mf_ble_encode.h - turns the SI values in MfState / MfGps into the exact byte layouts of the
// GATT characteristics listed for the Vector Vario service, Environmental Sensing, Battery.
// All multi-byte values are little-endian (BLE convention). Host-tested, see test_proto.cpp.
//
// Layouts follow the table in the Vector Vario developer documentation as given:
//   vario / netto / TAS / IAS  int32  dm/s   (0.1 m/s)
//   heading                    int16  whole degrees 0..359
//   roll / pitch               int16  0.1 deg
//   ground glide ratio         int16  GR * 10
//   load factor                int32  0.1 g
//   static pressure            uint32 0.1 Pa
//   air temperature            int16  0.01 degC
//   relative humidity          uint16 0.01 %
//   altitude (GNSS)            int24  cm
//   wind speed                 uint16 cm/s
//   wind origin direction      uint16 0.01 deg
//   battery level              uint8  %
// All of them return the number of bytes written to `o` (max 4).

#pragma once
#include <stdint.h>
#include <math.h>
#include "mf_ble_proto.h"

static inline long mf_rnd(float v, long lo, long hi) {
  if (isnan(v)) return 0;                       // a valid-flagged NaN is a sender bug; don't hit UB
  if (v > (float)hi) return hi;
  if (v < (float)lo) return lo;
  return lroundf(v);
}
static inline size_t mf_le16(uint8_t* o, int32_t v) { uint32_t u = (uint32_t)v; o[0] = u & 0xFF; o[1] = (u >> 8) & 0xFF; return 2; }
static inline size_t mf_le24(uint8_t* o, int32_t v) { uint32_t u = (uint32_t)v; o[0] = u & 0xFF; o[1] = (u >> 8) & 0xFF; o[2] = (u >> 16) & 0xFF; return 3; }
static inline size_t mf_le32(uint8_t* o, int32_t v) { uint32_t u = (uint32_t)v; o[0] = u & 0xFF; o[1] = (u >> 8) & 0xFF; o[2] = (u >> 16) & 0xFF; o[3] = (u >> 24) & 0xFF; return 4; }

// ---- Vector Vario service
static inline size_t mf_enc_vario(uint8_t* o, const MfState& s) { return mf_le32(o, (int32_t)mf_rnd(s.vario * 10.0f, -2000000L, 2000000L)); }
static inline size_t mf_enc_netto(uint8_t* o, const MfState& s) { return mf_le32(o, (int32_t)mf_rnd(s.netto * 10.0f, -2000000L, 2000000L)); }
static inline size_t mf_enc_tas  (uint8_t* o, const MfState& s) { return mf_le32(o, (int32_t)mf_rnd(s.tas   * 10.0f, -2000000L, 2000000L)); }
static inline size_t mf_enc_ias  (uint8_t* o, const MfState& s) { return mf_le32(o, (int32_t)mf_rnd(s.ias   * 10.0f, -2000000L, 2000000L)); }
static inline size_t mf_enc_heading(uint8_t* o, const MfState& s) {
  long h = mf_rnd(s.heading, -100000L, 100000L) % 360; if (h < 0) h += 360;
  return mf_le16(o, (int32_t)h);
}
static inline size_t mf_enc_roll (uint8_t* o, const MfState& s) { return mf_le16(o, (int32_t)mf_rnd(s.roll  * 10.0f, -1800, 1800)); }
static inline size_t mf_enc_pitch(uint8_t* o, const MfState& s) { return mf_le16(o, (int32_t)mf_rnd(s.pitch * 10.0f,  -900,  900)); }
static inline size_t mf_enc_glide(uint8_t* o, const MfState& s) { return mf_le16(o, (int32_t)mf_rnd(s.glide_ratio * 10.0f, -32767, 32767)); }
static inline size_t mf_enc_load (uint8_t* o, const MfState& s) { return mf_le32(o, (int32_t)mf_rnd(s.load_factor * 10.0f, 0, 2000000L)); }
static inline size_t mf_enc_vario_mode(uint8_t* o, const MfState& s) { o[0] = s.vario_mode ? 1 : 0; return 1; }

// ---- Environmental Sensing service
static inline size_t mf_enc_pressure(uint8_t* o, const MfState& s) { return mf_le32(o, (int32_t)mf_rnd(s.static_pressure * 10.0f, 0, 2147483000L)); }
static inline size_t mf_enc_temp    (uint8_t* o, const MfState& s) { return mf_le16(o, (int32_t)mf_rnd(s.air_temp * 100.0f, -32767, 32767)); }
static inline size_t mf_enc_humidity(uint8_t* o, const MfState& s) { return mf_le16(o, (int32_t)mf_rnd(s.humidity * 100.0f, 0, 10000)); }
static inline size_t mf_enc_wind_speed(uint8_t* o, const MfState& s) { return mf_le16(o, (int32_t)mf_rnd(s.wind_speed * 100.0f, 0, 65534)); }
static inline size_t mf_enc_wind_dir(uint8_t* o, const MfState& s) {
  float d = fmodf(s.wind_from, 360.0f); if (d < 0) d += 360.0f;
  long v = mf_rnd(d * 100.0f, 0, 36000); if (v >= 36000) v -= 36000;
  return mf_le16(o, (int32_t)v);
}
static inline size_t mf_enc_alt_gnss(uint8_t* o, const MfGps& g) {     // mm -> cm, int24
  long cm = lroundf(g.alt_mm / 10.0f);
  if (cm > 8388607L) cm = 8388607L;
  if (cm < -8388608L) cm = -8388608L;
  return mf_le24(o, (int32_t)cm);
}

// ---- Battery service
static inline size_t mf_enc_battery(uint8_t* o, const MfState& s) { o[0] = s.battery_pct > 100 ? 100 : s.battery_pct; return 1; }

// BLE "value is not known" markers, for characteristics we have no sensor for
static inline size_t mf_enc_unknown_u16(uint8_t* o) { o[0] = 0xFF; o[1] = 0xFF; return 2; }   // humidity
static inline size_t mf_enc_unknown_i16(uint8_t* o) { o[0] = 0x00; o[1] = 0x80; return 2; }   // temperature (0x8000)
