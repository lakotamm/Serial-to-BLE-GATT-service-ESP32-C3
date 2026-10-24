// esp32c3_vario_ble.ino - BLE bridge for the RP2350 variometer.
//
// Receives MfState / MfGps / INFO frames over UART (see mf_ble_proto.h) and republishes them
// as GATT characteristics that XCTrack understands: Device Information, Battery, Environmental
// Sensing (pressure, temperature, humidity, GNSS altitude, wind), the Vector Vario service and
// the FFE0/FFE1 serial service carrying $GPGGA + $GPRMC.
//
// Board:  ESP32C3 Dev Module (ESP32-C3 SuperMini), "USB CDC On Boot: Enabled" so Serial = USB debug.
// Needs:  Arduino-ESP32 core with the built-in BLE library (2.0.x or 3.x). Nothing else to install.
// Files:  this sketch folder must also contain mf_ble_proto.h, mf_nmea.h, mf_ble_encode.h.
//
// Wiring (3.3 V, common GND):
//   ESP32 GPIO4 (RX)  <-  RP2350 GPIO24 (UART1 TX)
//   ESP32 GPIO3 (TX)  ->  RP2350 GPIO25 (UART1 RX)
// GPIO2 / 8 / 9 are strapping pins on the C3 - not used here on purpose.

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#if __has_include(<esp_mac.h>)
#include <esp_mac.h>
#else
#include <esp_system.h>
#endif
#include "mf_ble_proto.h"
#include "mf_nmea.h"
#include "mf_ble_encode.h"

// ------------------------------------------------------------------ configuration
#define LINK_RX_PIN 4
#define LINK_TX_PIN 3
#define DEVICE_NAME "WindVario"          // your own name: XCTrack keys on the service UUID, not the name

#define STALE_MS 1500                    // no STATE frame for this long -> stop notifying (never repeat stale data)

// Minimum interval between notifications of each characteristic [ms]. XCTrack's own device
// reports pressure at 10 Hz and vario at 5 Hz, so going much faster only costs phone battery.
#define P_PRESSURE 100
#define P_VARIO    100
#define P_NETTO    100
#define P_TAS      200
#define P_IAS      200
#define P_HEADING  500
#define P_ROLL     200
#define P_PITCH    200
#define P_GLIDE    500
#define P_LOAD     200
#define P_WIND     500
#define P_TEMP     1000
#define P_BATTERY  2000
#define P_GPS      1000                  // GGA+RMC and GNSS altitude

// ------------------------------------------------------------------ UUIDs
#define UUID_VV_SERVICE "2fce4890-0197-47e0-a825-d4777b9a5d67"
#define UUID_VV_MODE    "2fce4903-0197-47e0-a825-d4777b9a5d67"
#define UUID_VV_VARIO   "2fce4891-0197-47e0-a825-d4777b9a5d67"
#define UUID_VV_NETTO   "2fce4902-0197-47e0-a825-d4777b9a5d67"
#define UUID_VV_TAS     "2fce4892-0197-47e0-a825-d4777b9a5d67"
#define UUID_VV_IAS     "2fce4893-0197-47e0-a825-d4777b9a5d67"
#define UUID_VV_HEADING "2fce4896-0197-47e0-a825-d4777b9a5d67"
#define UUID_VV_ROLL    "2fce4897-0197-47e0-a825-d4777b9a5d67"
#define UUID_VV_PITCH   "2fce4898-0197-47e0-a825-d4777b9a5d67"
#define UUID_VV_GLIDE   "2fce4900-0197-47e0-a825-d4777b9a5d67"
#define UUID_VV_LOAD    "2fce4895-0197-47e0-a825-d4777b9a5d67"
#define UUID_VV_GPS     "2fce4899-0197-47e0-a825-d4777b9a5d67"

// ------------------------------------------------------------------ state
static BLECharacteristic *c_sysid, *c_model, *c_serial, *c_fw, *c_hw, *c_mfr;
static BLECharacteristic *c_batt;
static BLECharacteristic *c_press, *c_temp, *c_hum, *c_altgnss, *c_wspd, *c_wdir;
static BLECharacteristic *c_mode, *c_vario, *c_netto, *c_tas, *c_ias, *c_hdg, *c_roll, *c_pitch, *c_glide, *c_load, *c_vvgps;
static BLECharacteristic *c_ssp;

static volatile bool g_connected = false;
static volatile uint16_t g_mtu = 23;

static MfParser g_parser;
static MfState  g_state;  static bool g_have_state = false; static uint32_t g_state_rx_ms = 0;
static MfGps    g_gps;    static bool g_have_gps   = false; static uint32_t g_gps_rx_ms   = 0;

// ------------------------------------------------------------------ small helpers
static bool due(uint32_t& last, uint32_t now, uint32_t period) {
  if ((uint32_t)(now - last) >= period) { last = now; return true; }
  return false;
}

// Always stores the value (so a READ works), notifies only when a central is connected.
static void put(BLECharacteristic* c, const uint8_t* d, size_t n, bool notify = true) {
  c->setValue((uint8_t*)d, n);
  if (notify && g_connected) c->notify();
}
static void setText(BLECharacteristic* c, const char* s) { c->setValue((uint8_t*)s, strlen(s)); }

// ------------------------------------------------------------------ BLE server
class ServerCb : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    g_connected = true;
  }

  void onDisconnect(BLEServer*) override {
    g_connected = false;
    g_mtu = 23;
    BLEDevice::startAdvertising();
  }

  void onMtuChanged(BLEServer*, ble_gap_conn_desc*, uint16_t mtu) override {
    g_mtu = mtu;
  }
};

// numHandles matters: every characteristic uses 2 handles, +1 more if it can notify (the CCCD),
// +1 for the service itself. The library default is 15, which silently drops characteristics
// from a bigger service. Always size it explicitly.
static BLECharacteristic* addChr(BLEService* s, BLEUUID uuid, bool notify) {
  BLECharacteristic* c = s->createCharacteristic(uuid, BLECharacteristic::PROPERTY_READ | (notify ? BLECharacteristic::PROPERTY_NOTIFY : 0));
  return c;
}

static void buildServices(BLEServer* server) {
  // Device Information 0x180A (6 chr, no CCCD -> 13 handles)
  BLEService* dis = server->createService(BLEUUID((uint16_t)0x180A), 20);
  c_sysid  = addChr(dis, BLEUUID((uint16_t)0x2A23), false);
  c_model  = addChr(dis, BLEUUID((uint16_t)0x2A24), false);
  c_serial = addChr(dis, BLEUUID((uint16_t)0x2A25), false);
  c_fw     = addChr(dis, BLEUUID((uint16_t)0x2A26), false);
  c_hw     = addChr(dis, BLEUUID((uint16_t)0x2A27), false);
  c_mfr    = addChr(dis, BLEUUID((uint16_t)0x2A29), false);

  // Battery 0x180F
  BLEService* bat = server->createService(BLEUUID((uint16_t)0x180F), 8);
  c_batt = addChr(bat, BLEUUID((uint16_t)0x2A19), true);

  // Environmental Sensing 0x181A (6 notifying chr -> 19 handles)
  BLEService* ess = server->createService(BLEUUID((uint16_t)0x181A), 30);
  c_press   = addChr(ess, BLEUUID((uint16_t)0x2A6D), true);
  c_temp    = addChr(ess, BLEUUID((uint16_t)0x2A6E), true);
  c_hum     = addChr(ess, BLEUUID((uint16_t)0x2A6F), true);
  c_altgnss = addChr(ess, BLEUUID((uint16_t)0x2AB3), true);
  c_wspd    = addChr(ess, BLEUUID((uint16_t)0x2A70), true);
  c_wdir    = addChr(ess, BLEUUID((uint16_t)0x2A71), true);

  // Vector Vario (11 notifying chr -> 34 handles)
  BLEService* vv = server->createService(BLEUUID(UUID_VV_SERVICE), 60);
  c_mode  = addChr(vv, BLEUUID(UUID_VV_MODE),    true);
  c_vario = addChr(vv, BLEUUID(UUID_VV_VARIO),   true);
  c_netto = addChr(vv, BLEUUID(UUID_VV_NETTO),   true);
  c_tas   = addChr(vv, BLEUUID(UUID_VV_TAS),     true);
  c_ias   = addChr(vv, BLEUUID(UUID_VV_IAS),     true);
  c_hdg   = addChr(vv, BLEUUID(UUID_VV_HEADING), true);
  c_roll  = addChr(vv, BLEUUID(UUID_VV_ROLL),    true);
  c_pitch = addChr(vv, BLEUUID(UUID_VV_PITCH),   true);
  c_glide = addChr(vv, BLEUUID(UUID_VV_GLIDE),   true);
  c_load  = addChr(vv, BLEUUID(UUID_VV_LOAD),    true);
  c_vvgps = addChr(vv, BLEUUID(UUID_VV_GPS),     true);

  // SSP 0xFFE0 / 0xFFE1
  BLEService* ssp = server->createService(BLEUUID((uint16_t)0xFFE0), 8);
  c_ssp = addChr(ssp, BLEUUID((uint16_t)0xFFE1), true);

  dis->start(); bat->start(); ess->start(); vv->start(); ssp->start();
}

static void setInitialValues() {
  // System ID: 40-bit manufacturer-defined id (from the MAC) + 24-bit OUI, both LSB first.
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_BT);
  const uint8_t sysid[8] = { mac[5], mac[4], mac[3], 0xFF, 0xFE, mac[2], mac[1], mac[0] };
  c_sysid->setValue((uint8_t*)sysid, sizeof(sysid));

  setText(c_mfr, "DIY"); setText(c_model, "WindVario"); setText(c_serial, "0000");
  setText(c_fw, "unknown"); setText(c_hw, "unknown");     // replaced by the RP2350's INFO frames

  // We have no humidity sensor: publish BLE's "not known" value and never notify it.
  uint8_t b[2];
  mf_enc_unknown_u16(b); c_hum->setValue(b, 2);
  mf_enc_unknown_i16(b); c_temp->setValue(b, 2);
}

// ------------------------------------------------------------------ what gets published, and how often
typedef size_t (*EncFn)(uint8_t*, const MfState&);
struct Out {
  BLECharacteristic** chr;
  uint32_t valid_bit;
  uint16_t period_ms;
  EncFn enc;
  uint32_t last;
};
static Out g_out[] = {
  { &c_press, MF_V_PRESSURE, P_PRESSURE, mf_enc_pressure,   0 },
  { &c_vario, MF_V_VARIO,    P_VARIO,    mf_enc_vario,      0 },
  { &c_netto, MF_V_NETTO,    P_NETTO,    mf_enc_netto,      0 },
  { &c_tas,   MF_V_TAS,      P_TAS,      mf_enc_tas,        0 },
  { &c_ias,   MF_V_IAS,      P_IAS,      mf_enc_ias,        0 },
  { &c_hdg,   MF_V_HEADING,  P_HEADING,  mf_enc_heading,    0 },
  { &c_roll,  MF_V_ROLL,     P_ROLL,     mf_enc_roll,       0 },
  { &c_pitch, MF_V_PITCH,    P_PITCH,    mf_enc_pitch,      0 },
  { &c_glide, MF_V_GLIDE,    P_GLIDE,    mf_enc_glide,      0 },
  { &c_load,  MF_V_LOAD,     P_LOAD,     mf_enc_load,       0 },
  { &c_wspd,  MF_V_WIND,     P_WIND,     mf_enc_wind_speed, 0 },
  { &c_wdir,  MF_V_WIND,     P_WIND,     mf_enc_wind_dir,   0 },
  { &c_temp,  MF_V_TEMP,     P_TEMP,     mf_enc_temp,       0 },
  { &c_hum,   MF_V_HUMIDITY, P_TEMP,     mf_enc_humidity,   0 },   // only ever fires if the RP2350 starts sending humidity
  { &c_batt,  MF_V_BATTERY,  P_BATTERY,  mf_enc_battery,    0 },
};

static void publish(uint32_t now) {
  static uint32_t t_mode = 0, t_gps = 0;
  static int last_mode = -1;
  static bool warned_mtu = false;
  uint8_t b[4];

  // 1. STATE-derived characteristics, only while the RP2350 is actually sending
  if (g_have_state && (uint32_t)(now - g_state_rx_ms) < STALE_MS) {
    const MfState& s = g_state;
    for (Out& o : g_out) {
      if ((s.valid & o.valid_bit) && due(o.last, now, o.period_ms)) {
        size_t n = o.enc(b, s);
        put(*o.chr, b, n);
      }
    }
    // vario mode: notify on change, and refresh now and then so a late subscriber sees it
    if (s.valid & MF_V_VARIO) {
      if (s.vario_mode != last_mode || due(t_mode, now, 5000)) {
        last_mode = s.vario_mode;
        size_t n = mf_enc_vario_mode(b, s);
        put(c_mode, b, n);
      }
    }
  }

  // 2. GPS: GGA + RMC for the Vector Vario GPS and SSP characteristics, plus GNSS altitude
  if (g_have_gps && (uint32_t)(now - g_gps_rx_ms) < STALE_MS && g_gps.fix >= 2 && due(t_gps, now, P_GPS)) {
    char nmea[200];
    size_t a = mf_format_gga(nmea, sizeof(nmea), g_gps);
    size_t r = a ? mf_format_rmc(nmea + a, sizeof(nmea) - a, g_gps) : 0;
    if (a && r) {
      size_t n = a + r;
      if (g_connected && n + 3 > g_mtu && !warned_mtu) {
        warned_mtu = true;
        Serial.printf("[BLE] MTU %u too small for the %u-byte GPS value - the phone has not negotiated a larger MTU\n", g_mtu, (unsigned)n);
      }
      put(c_vvgps, (const uint8_t*)nmea, n);
      put(c_ssp,   (const uint8_t*)nmea, n);
    }
    size_t n = mf_enc_alt_gnss(b, g_gps);
    put(c_altgnss, b, n);
  }
}

// ------------------------------------------------------------------ frames from the RP2350
static void handleFrame() {
  const uint8_t* p = g_parser.payload;
  const uint8_t len = g_parser.len;
  switch (g_parser.type) {
    case MF_MSG_STATE:
      if (len >= sizeof(MfState)) { memcpy(&g_state, p, sizeof(MfState)); g_have_state = true; g_state_rx_ms = millis(); }
      break;
    case MF_MSG_GPS:
      if (len >= sizeof(MfGps)) { memcpy(&g_gps, p, sizeof(MfGps)); g_have_gps = true; g_gps_rx_ms = millis(); }
      break;
    case MF_MSG_INFO:
      if (len >= 2) {
        BLECharacteristic* c = nullptr;
        switch (p[0]) {
          case MF_INFO_MANUFACTURER: c = c_mfr; break;
          case MF_INFO_MODEL:        c = c_model; break;
          case MF_INFO_SERIAL:       c = c_serial; break;
          case MF_INFO_FIRMWARE:     c = c_fw; break;
          case MF_INFO_HARDWARE:     c = c_hw; break;
        }
        if (c) c->setValue((uint8_t*)(p + 1), len - 1);
      }
      break;
  }
}

static void sendStatus(uint32_t now) {
  static uint32_t t = 0;
  if (!due(t, now, 1000)) return;
  MfStatus s;
  s.proto = MF_PROTO_VERSION;
  s.ble_connected = g_connected ? 1 : 0;
  s.mtu = g_mtu;
  s.rx_ok = (uint16_t)g_parser.ok;
  s.rx_err = (uint16_t)g_parser.crc_err;
  uint8_t buf[MF_MAX_FRAME];
  size_t n = mf_frame_encode(buf, MF_MSG_STATUS, &s, sizeof(s));
  Serial1.write(buf, n);
}

static void debugPrint(uint32_t now) {
  static uint32_t t = 0;
  if (!due(t, now, 5000)) return;
  Serial.printf("[BLE] link ok=%lu crc_err=%lu  state_age=%ldms  connected=%d mtu=%u\n",
                (unsigned long)g_parser.ok, (unsigned long)g_parser.crc_err,
                g_have_state ? (long)(now - g_state_rx_ms) : -1L, g_connected ? 1 : 0, g_mtu);
}

// ------------------------------------------------------------------ Arduino entry points
void setup() {
  Serial.begin(115200);
  Serial1.setRxBufferSize(1024);                       // must come before begin()
  Serial1.begin(MF_LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);

  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(247);                              // GGA+RMC is ~150 bytes; needs MTU >= 186
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCb());
  buildServices(server);
  setInitialValues();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLEUUID(UUID_VV_SERVICE));
  adv->addServiceUUID(BLEUUID((uint16_t)0x181A));
  adv->setScanResponse(true);                          // name goes into the scan response, UUIDs into the advert
  BLEDevice::startAdvertising();
  Serial.println("[BLE] advertising");
}

void loop() {
  while (Serial1.available() > 0) {
    int b = Serial1.read();
    if (b >= 0 && g_parser.feed((uint8_t)b)) handleFrame();
  }
  const uint32_t now = millis();
  publish(now);
  sendStatus(now);
  debugPrint(now);
  delay(2);
}
