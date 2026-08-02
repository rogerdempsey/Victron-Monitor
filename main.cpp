// Victron BLE Monitor - ESP32
//
// Scans Victron "Instant Readout" BLE advertisements from a SmartShunt
// (battery monitor) and a SmartSolar/BlueSolar MPPT, decrypts them, and
// serves a live dashboard - with a current-flow diagram - on a static-IP
// web page.
//
// Library: scottp/victronble (installed automatically via platformio.ini)

#include <Arduino.h>
#include <string.h>  // strstr, strncmp - used by jsonNumberField/jsonBoolField
#include <stdlib.h>  // strtod - used by jsonNumberField
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <time.h>
#include <sys/time.h>
#include "VictronBLE.h"
#include "esp_bt.h" // esp_bt_controller_mem_release() - see setup(), frees unused Classic BT RAM
#include <esp_task_wdt.h> // task watchdog - see setup()/loop(), auto-reboots on a stuck loop()
#include <esp_log.h> // esp_log_level_set() - see setup(), silences ESP-IDF component logging (WiFi/BLE/lwIP)
#include "secrets.h" // WIFI_SSID/PASSWORD, SOLAR_MAC/KEY, SHUNT_MAC/KEY - see secrets.h.example
#include <OneWire.h>
#include <DallasTemperature.h>

// Network hostname - shows up in your router's client list / DHCP leases.
const char* DEVICE_HOSTNAME = "VictronGPS";

// ---------------------------------------------------------------------
// >>> EDIT secrets.h, not this file - see secrets.h.example <<<
// ---------------------------------------------------------------------

// Static IP configuration. Adjust gateway/subnet if your router uses a
// different range (check your router's admin page if unsure).
IPAddress local_IP(192, 168, 8, 66);
IPAddress gateway(192, 168, 8, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(192, 168, 8, 1);
IPAddress secondaryDNS(8, 8, 8, 8);
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// NMEA0183 GPS input -> UDP broadcast for OpenCPN
//
// IMPORTANT: GPS receivers wired at 12V put out RS-422-level (or at
// least non-3.3V-safe) NMEA0183 signals on their data lines. NEVER wire
// those lines straight into these GPIOs - you need a level
// shifter/opto-isolator between the receiver and the ESP32 first (see
// README for the circuit). These GPIOs should only ever see clean
// 3.3V TTL out of that converter.
//
// GPIO34 is used because it's an input-only pin on the ESP32 -
// a good fit since the feed is receive-only from the ESP32's side,
// and it keeps this off any strapping/BLE-sensitive pins.
// ---------------------------------------------------------------------
#define GPS_RX_PIN     34
#define GPS_BAUD       4800

// Port OpenCPN listens on. In OpenCPN: Options -> Connections -> Add ->
// Network -> Protocol: UDP -> Address: 0.0.0.0 -> Port: (match below).
#define NMEA_UDP_PORT  10110

HardwareSerial gpsSerial(1);
WiFiUDP nmeaUdp;
IPAddress nmeaBroadcastIP;
// Fixed buffer, not a String - see pumpNmea() below for why.
char gpsLineBuf[128];
size_t gpsLineLen = 0;

// Tracks whether the NMEA0183 input is actively receiving sentences (i.e.
// the wiring/level-shifter is good and the receiver is talking), separate
// from whether the GPS currently has a satellite fix.
struct {
  bool everSeen = false;
  uint32_t lastSentence = 0;
} gpsLinkStatus;

// ---------------------------------------------------------------------
// AIS receiver - a separate network device that broadcasts data over
// UDP. This board deliberately does nothing with the actual data beyond
// noticing that packets are arriving - no parsing, no forwarding, just a
// connectivity indicator (the AIS pill), same idea as the GPS pill but
// for "is the AIS receiver alive and talking" instead of "are NMEA
// sentences arriving over the wired GPS UART".
//
// UDP broadcast rather than a TCP connection because the receiver only
// accepts one TCP client at a time - broadcast means this board (and
// anything else that wants the feed, like OpenCPN) can listen without
// fighting over that single slot.
// ---------------------------------------------------------------------
IPAddress AIS_IP(192, 168, 8, 75); // expected sender - packets from elsewhere on this port are ignored
const uint16_t AIS_UDP_PORT = 9000;
WiFiUDP aisUdp;

struct {
  bool everSeen = false;
  uint32_t lastByte = 0;
} aisLinkStatus;

// Non-blocking - parsePacket() returns 0 immediately if nothing's
// waiting. Just drains whatever's there and discards it - the only
// thing tracked is *that* a packet arrived, not its content, since the
// pill only ever needs to answer "is this thing alive".
void pollAis() {
  int packetSize = aisUdp.parsePacket();
  if (packetSize <= 0) return;
  uint8_t discard[256];
  while (aisUdp.available() > 0) {
    int n = aisUdp.read(discard, sizeof(discard));
    if (n <= 0) break;
  }
  if (aisUdp.remoteIP() != AIS_IP) return; // not from the sender we're expecting
  aisLinkStatus.everSeen = true;
  aisLinkStatus.lastByte = millis();
}

// Set from the GPS's own $--RMC sentences so the dashboard can show local
// time without needing internet/NTP access. Pacific time (with automatic
// PST/PDT handling) is computed from this at render time in handleData().
bool gpsTimeValid = false;
uint32_t gpsTimeLastSet = 0;

// Latest position/speed/course, parsed from the same $--RMC sentences as
// gpsTimeValid above - no separate GPS module or parser library needed,
// since RMC already carries lat/lon/speed/course alongside time+date.
struct {
  bool valid = false;
  uint32_t lastUpdate = 0;
  double lat = 0;
  double lon = 0;
  double speedKnots = 0;
  double courseDeg = 0;
} gpsFix;

// Returns the fieldIndex-th comma-separated field of an NMEA sentence
// (field 0 is the sentence ID itself, before the first comma).
String nmeaField(const String &s, int fieldIndex) {
  int start = 0;
  for (int i = 0; i < fieldIndex; i++) {
    int c = s.indexOf(',', start);
    if (c < 0) return "";
    start = c + 1;
  }
  int end = s.indexOf(',', start);
  if (end < 0) end = s.length();
  return s.substring(start, end);
}

// Days since 1970-01-01 for a UTC calendar date, using Howard Hinnant's
// days_from_civil algorithm. Avoids depending on timegm(), which isn't
// available in the ESP32 Arduino toolchain's libc.
static long daysFromEpoch(int year, int month, int day) {
  year -= month <= 2;
  long era = (year >= 0 ? year : year - 399) / 400;
  unsigned yoe = (unsigned)(year - era * 400);              // [0, 399]
  unsigned doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1; // [0, 365]
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;      // [0, 146096]
  return era * 146097 + (long)doe - 719468;
}

// Converts a UTC struct tm to an epoch time_t without relying on timegm()
// or the process's local timezone setting.
static time_t utcMktime(const struct tm &t) {
  long days = daysFromEpoch(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
  return (time_t)days * 86400L + t.tm_hour * 3600L + t.tm_min * 60L + t.tm_sec;
}

// If this line is a valid $--RMC sentence with a good fix, pulls the UTC
// time+date out of it and sets the ESP32's system clock. Everything else
// (webpage Pacific-time display, etc.) reads from time(), not from here
// directly, so it stays in sync automatically.
void maybeUpdateGpsTime(const String &line) {
  if (line.length() < 6 || line.charAt(0) != '$') return;
  if (line.substring(3, 6) != "RMC") return;      // e.g. $GPRMC / $GNRMC
  if (nmeaField(line, 2) != "A") return;          // 'A' = valid fix, 'V' = void

  String timeField = nmeaField(line, 1);          // hhmmss.ss
  String dateField = nmeaField(line, 9);          // ddmmyy
  if (timeField.length() < 6 || dateField.length() < 6) return;

  struct tm t = {};
  t.tm_hour = timeField.substring(0, 2).toInt();
  t.tm_min  = timeField.substring(2, 4).toInt();
  t.tm_sec  = timeField.substring(4, 6).toInt();
  t.tm_mday = dateField.substring(0, 2).toInt();
  t.tm_mon  = dateField.substring(2, 4).toInt() - 1;
  t.tm_year = dateField.substring(4, 6).toInt() + 100; // assumes 20xx

  time_t utcEpoch = utcMktime(t);
  if (utcEpoch < 1700000000) return; // sanity check: ignore bogus/pre-2023 dates

  struct timeval tv = { .tv_sec = utcEpoch, .tv_usec = 0 };
  settimeofday(&tv, nullptr);
  gpsTimeValid = true;
  gpsTimeLastSet = millis();
}

// Converts an NMEA0183 coordinate field (ddmm.mmmm for latitude,
// dddmm.mmmm for longitude) to signed decimal degrees.
double nmeaCoordToDecimal(const String &field, char hemisphere) {
  if (field.length() < 4) return 0;
  int degLen = (hemisphere == 'E' || hemisphere == 'W') ? 3 : 2;
  double deg = field.substring(0, degLen).toDouble();
  double minutes = field.substring(degLen).toDouble();
  double decimal = deg + minutes / 60.0;
  if (hemisphere == 'S' || hemisphere == 'W') decimal = -decimal;
  return decimal;
}

// If this line is a valid $--RMC sentence with a good fix, pulls
// lat/lon/speed/course out of it for anything that wants a position.
// Runs alongside maybeUpdateGpsTime() on the same sentences.
void maybeUpdateGpsPosition(const String &line) {
  if (line.length() < 6 || line.charAt(0) != '$') return;
  if (line.substring(3, 6) != "RMC") return;
  if (nmeaField(line, 2) != "A") return; // 'A' = valid fix

  String latField = nmeaField(line, 3);
  String nsField   = nmeaField(line, 4);
  String lonField = nmeaField(line, 5);
  String ewField   = nmeaField(line, 6);
  String speedField = nmeaField(line, 7);
  String courseField = nmeaField(line, 8);
  if (latField.length() < 4 || lonField.length() < 4) return;

  gpsFix.lat = nmeaCoordToDecimal(latField, nsField.charAt(0));
  gpsFix.lon = nmeaCoordToDecimal(lonField, ewField.charAt(0));
  gpsFix.speedKnots = speedField.toDouble();
  gpsFix.courseDeg = courseField.toDouble();
  gpsFix.valid = true;
  gpsFix.lastUpdate = millis();
}

// Computes the subnet broadcast address (e.g. 192.168.8.255) from our
// static IP + subnet mask, so UDP packets reach every listener on the
// LAN without needing to know OpenCPN's specific IP.
IPAddress calculateBroadcast(IPAddress ip, IPAddress mask) {
  IPAddress bcast;
  for (int i = 0; i < 4; i++) {
    bcast[i] = ip[i] | (~mask[i] & 0xFF);
  }
  return bcast;
}

// Reads whatever bytes are available from the GPS UART, assembles
// them into lines, and forwards each complete NMEA0183 sentence
// ($GPGGA... etc.) untouched over UDP as soon as it's terminated.
// Sentences are passed through byte-for-byte (no re-parsing/checksum
// recalculation) so OpenCPN sees exactly what the receiver sent.
//
// Accumulates into a fixed char buffer rather than an Arduino String.
// The old code did `lineBuf += c` for every single incoming byte, which
// on Arduino's String class means a heap alloc/copy/free on essentially
// every byte - at the GPS's 4800 baud that's hundreds of tiny heap
// operations per second, running non-stop, which fragments the ESP32's
// heap badly over hours/days of uptime. That's the most likely cause of
// the board getting sluggish the longer it's been running. Now a String
// is only built once per *complete* sentence (a few times a second),
// just to hand off to the existing helpers below that expect one.
void pumpNmea(HardwareSerial &port, char *lineBuf, size_t lineBufCap, size_t &lineLen,
              bool &everSeen, uint32_t &lastSentence) {
  while (port.available()) {
    char c = (char)port.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (lineLen > 1 && (lineBuf[0] == '$' || lineBuf[0] == '!')) {
        everSeen = true;
        lastSentence = millis();
        lineBuf[lineLen] = '\0';
        String sentence(lineBuf); // one alloc per complete sentence, not per byte
        maybeUpdateGpsTime(sentence);
        maybeUpdateGpsPosition(sentence);
        nmeaUdp.beginPacket(nmeaBroadcastIP, NMEA_UDP_PORT);
        nmeaUdp.write((const uint8_t*)lineBuf, lineLen);
        nmeaUdp.write((const uint8_t*)"\r\n", 2);
        nmeaUdp.endPacket();
      }
      lineLen = 0;
    } else if (lineLen >= lineBufCap - 1) {
      lineLen = 0; // discard noise/garbage line (overflow), same as before
    } else {
      lineBuf[lineLen++] = c;
    }
  }
}

VictronBLE victron;
WebServer server(80);
uint32_t bootFreeHeap = 0; // set once in setup(), see there for why

// Device MAC addresses and encryption keys come from secrets.h
// (SOLAR_MAC, SOLAR_KEY, SHUNT_MAC, SHUNT_KEY - see secrets.h.example)

// Latest data, updated by the BLE callback
struct {
  bool valid = false;
  uint32_t lastUpdate = 0;
  int8_t rssi = 0;
  VictronSolarData data;
} solarLatest;

struct {
  bool valid = false;
  uint32_t lastUpdate = 0;
  int8_t rssi = 0;
  VictronBatteryData data;
} shuntLatest;

// ---------------------------------------------------------------------
// Rolling 10-hour max solar charge current.
//
// Keeps the highest battery-charge current seen over the trailing ~10
// hours, without storing every individual sample. This buckets time
// into 10-minute slots (60 of them = 10 hours) and remembers only the
// max current seen within each slot; the overall max is the max across
// all still-valid slots.
//
// Each slot also stores which "epoch" (its absolute 10-minute slot
// number since boot) it currently holds data for. Since slots are
// reused every 10 hours (60 slots * 10 min), a slot whose stored epoch
// is too old (or unset) is treated as empty rather than stale data.
// ---------------------------------------------------------------------
const uint32_t SOLAR_CURRENT_BUCKET_MS = 10UL * 60 * 1000UL; // 10 minutes/bucket
const int SOLAR_CURRENT_NUM_BUCKETS = 60;                    // 60 * 10 min = 10 hours

struct {
  double maxCurrent[SOLAR_CURRENT_NUM_BUCKETS] = {0};
  uint32_t bucketEpoch[SOLAR_CURRENT_NUM_BUCKETS] = {0}; // 0 = never written
} solarCurrentHistory;

// Records one solar battery-current sample into the rolling history.
void recordSolarCurrentSample(double amps, uint32_t now) {
  // +1 so epoch is never 0, which is reserved to mean "never written".
  uint32_t epoch = now / SOLAR_CURRENT_BUCKET_MS + 1;
  int idx = epoch % SOLAR_CURRENT_NUM_BUCKETS;
  if (solarCurrentHistory.bucketEpoch[idx] != epoch) {
    // First sample in this slot, or the slot has wrapped around from an
    // old epoch - start it fresh rather than mixing with old data.
    solarCurrentHistory.bucketEpoch[idx] = epoch;
    solarCurrentHistory.maxCurrent[idx] = amps;
  } else if (amps > solarCurrentHistory.maxCurrent[idx]) {
    solarCurrentHistory.maxCurrent[idx] = amps;
  }
}

// Returns the highest solar charge current recorded in roughly the last
// 10 hours, or 0 if no samples have been recorded yet.
double solarMaxCurrentLast10h(uint32_t now) {
  uint32_t currentEpoch = now / SOLAR_CURRENT_BUCKET_MS + 1;
  double best = 0;
  bool any = false;
  for (int i = 0; i < SOLAR_CURRENT_NUM_BUCKETS; i++) {
    uint32_t epoch = solarCurrentHistory.bucketEpoch[i];
    if (epoch == 0) continue; // never written
    if (currentEpoch - epoch >= (uint32_t)SOLAR_CURRENT_NUM_BUCKETS) continue; // outside the 10h window
    if (!any || solarCurrentHistory.maxCurrent[i] > best) {
      best = solarCurrentHistory.maxCurrent[i];
      any = true;
    }
  }
  return any ? best : 0;
}

// ---------------------------------------------------------------------
// Diesel heater fuel-flow, pushed here over UDP by the standalone
// CDHeater ESP32 instead of this board polling it over HTTP. CDHeater
// broadcasts a small JSON payload every ~2s (see its own firmware);
// this board just listens and updates whenever a packet arrives, the
// same fire-and-forget pattern already used for the GPS/NMEA UDP
// broadcast above. That avoids opening a TCP connection every poll
// cycle (connect timeouts, occasional ECONNRESET, socket lifecycle) for
// a value that's only ever read locally on this one LAN anyway.
//
// Freshness needs no separate tracking here - same as the BLE devices
// above, "how long since we last heard from it" (now - lastUpdate) is
// enough, so there's no poll interval/backoff to manage.
//
// JSON is parsed with simple string-searching (matching the style used
// elsewhere in this file for NMEA0183) rather than pulling in
// ArduinoJson just for a handful of numeric fields.
// ---------------------------------------------------------------------
IPAddress HEATER_IP(192, 168, 8, 65); // expected sender - packets from elsewhere on this port are ignored
const uint16_t TELEMETRY_UDP_PORT = 2000;
WiFiUDP telemetryUdp;
char telemetryBuf[384];

struct {
  bool valid = false;
  uint32_t lastUpdate = 0;
  double rateLph = 0;
  double todayLiters = 0;
  double totalLiters = 0;
  uint32_t pulses = 0;
  // NMEA2000 fuel tank level, as relayed by CDHeater's /data endpoint
  // (CDHeater itself listens for PGN 127505 on the boat's N2K bus - see
  // diesel_heater_pulse_monitor.ino - this board just reads its JSON).
  bool tankSeen = false;   // has CDHeater ever seen a tank PGN since its own boot?
  bool tankStale = false;  // CDHeater's own staleness flag (no PGN in >30s)
  double tankPct = -1;     // % full, -1 = no reading
  double tankCapacityLiters = -1;
} heaterLatest;

// Icebox fridge/crisper temps, pushed here over UDP by the standalone
// Icebox Monitor ESP32 - same fire-and-forget pattern as the heater
// telemetry above, sharing the same telemetryUdp socket and
// pollTelemetry() below (distinguished by sender IP). Its /api/live and
// /api/history stay on that board too, still used by its own local
// dashboard page.
IPAddress ICEBOX_IP(192, 168, 8, 67); // expected sender - packets from elsewhere on this port are ignored

struct {
  bool valid = false;
  uint32_t lastUpdate = 0;
  double fridgeTempC = 0;
  double crisperTempC = 0;
  bool compressorOn = false;
  uint8_t dutyPercent = 0;
  uint32_t cycles = 0;
} iceboxLatest;

// ---------------------------------------------------------------------
// Rolling 1-hour max fuel tank level.
//
// Same bucketed-max approach as solarCurrentHistory above, just sized
// for a 1-hour window instead of 10 hours: 2-minute buckets, 30 of them.
// Recording the max (rather than the latest) over the last hour is
// useful because the tank level reading briefly dips as fuel sloshes
// under way - the max smooths that out and gives a steadier number.
// ---------------------------------------------------------------------
const uint32_t TANK_LEVEL_BUCKET_MS = 2UL * 60 * 1000UL; // 2 minutes/bucket
const int TANK_LEVEL_NUM_BUCKETS = 30;                    // 30 * 2 min = 1 hour

struct {
  double maxPct[TANK_LEVEL_NUM_BUCKETS] = {0};
  uint32_t bucketEpoch[TANK_LEVEL_NUM_BUCKETS] = {0}; // 0 = never written
} tankLevelHistory;

// Records one tank-level % sample into the rolling history. Only call
// this with fresh (non-stale) readings - see pollTelemetry().
void recordTankLevelSample(double pct, uint32_t now) {
  uint32_t epoch = now / TANK_LEVEL_BUCKET_MS + 1; // +1: 0 means "never written"
  int idx = epoch % TANK_LEVEL_NUM_BUCKETS;
  if (tankLevelHistory.bucketEpoch[idx] != epoch) {
    tankLevelHistory.bucketEpoch[idx] = epoch;
    tankLevelHistory.maxPct[idx] = pct;
  } else if (pct > tankLevelHistory.maxPct[idx]) {
    tankLevelHistory.maxPct[idx] = pct;
  }
}

// Returns the highest tank level % recorded in roughly the last hour,
// or -1 if no samples have been recorded yet.
double tankLevelMaxLast1h(uint32_t now) {
  uint32_t currentEpoch = now / TANK_LEVEL_BUCKET_MS + 1;
  double best = -1;
  for (int i = 0; i < TANK_LEVEL_NUM_BUCKETS; i++) {
    uint32_t epoch = tankLevelHistory.bucketEpoch[i];
    if (epoch == 0) continue; // never written
    if (currentEpoch - epoch >= (uint32_t)TANK_LEVEL_NUM_BUCKETS) continue; // outside the 1h window
    if (tankLevelHistory.maxPct[i] > best) best = tankLevelHistory.maxPct[i];
  }
  return best;
}

// Pulls the numeric value following "key": out of a small JSON blob.
// Good enough for the flat, fixed-key JSON CDHeater's /data returns;
// not a general-purpose parser.
//
// Searches the raw C string with strstr() instead of String::indexOf()
// on a concatenated "\"key\":" String - the old version built that
// needle with `String("\"") + key + "\":"`, which is 3 separate heap
// allocations per call. With ~13 of these calls every poll cycle
// (heater + icebox, every few seconds, forever), that adds up to a lot
// of small heap churn over hours/days of uptime. strstr() on the
// existing buffers does the same search with zero extra allocations.
double jsonNumberField(const String &json, const char* key) {
  char needle[40];
  int nlen = snprintf(needle, sizeof(needle), "\"%s\":", key);
  const char* hit = strstr(json.c_str(), needle);
  if (!hit) return 0;
  const char* p = hit + nlen;
  const char* start = p;
  while (*p && (isDigit(*p) || *p == '.' || *p == '-')) p++;
  if (p == start) return 0;
  return strtod(start, nullptr);
}

// Pulls the boolean value (true/false, unquoted) following "key": out of
// a small JSON blob - same string-searching style/limits as
// jsonNumberField() above, just for booleans.
bool jsonBoolField(const String &json, const char* key) {
  char needle[40];
  int nlen = snprintf(needle, sizeof(needle), "\"%s\":", key);
  const char* hit = strstr(json.c_str(), needle);
  if (!hit) return false;
  return strncmp(hit + nlen, "true", 4) == 0;
}

// Drains any pending telemetry UDP packets - both the diesel heater and
// icebox boards push here on the same shared port, told apart by which
// IP the packet came from. Non-blocking: parsePacket() returns 0
// immediately if nothing's waiting, so this is cheap to call every pass
// through loop(). Only handles one packet per call, same as the rest of
// this file's loop() - if several are queued they just get picked up on
// the next few iterations instead of one big batch, which is fine since
// loop() runs continuously and nothing here is time-critical to that
// degree.
void pollTelemetry() {
  int packetSize = telemetryUdp.parsePacket();
  if (packetSize <= 0) return;
  int len = telemetryUdp.read(telemetryBuf, sizeof(telemetryBuf) - 1);
  if (len <= 0) return;
  telemetryBuf[len] = '\0';
  IPAddress sender = telemetryUdp.remoteIP();
  String payload(telemetryBuf);

  if (sender == HEATER_IP) {
    heaterLatest.rateLph    = jsonNumberField(payload, "rate_lph");
    heaterLatest.todayLiters = jsonNumberField(payload, "today_l");
    heaterLatest.totalLiters = jsonNumberField(payload, "total_l");
    heaterLatest.pulses     = (uint32_t)jsonNumberField(payload, "pulses");
    heaterLatest.tankSeen   = jsonBoolField(payload, "tank_seen");
    heaterLatest.tankStale  = jsonBoolField(payload, "tank_stale");
    heaterLatest.tankPct    = jsonNumberField(payload, "tank_pct");
    heaterLatest.tankCapacityLiters = jsonNumberField(payload, "tank_capacity_l");
    heaterLatest.lastUpdate = millis();
    heaterLatest.valid = true;

    // Only feed fresh, real N2K readings into the 1-hour max - a stale
    // (frozen) value from CDHeater shouldn't get counted as a new sample.
    if (heaterLatest.tankSeen && !heaterLatest.tankStale) {
      recordTankLevelSample(heaterLatest.tankPct, heaterLatest.lastUpdate);
    }
  } else if (sender == ICEBOX_IP) {
    iceboxLatest.fridgeTempC  = jsonNumberField(payload, "fridge");
    iceboxLatest.crisperTempC = jsonNumberField(payload, "crisper");
    iceboxLatest.compressorOn = jsonBoolField(payload, "compressor");
    iceboxLatest.dutyPercent  = (uint8_t)jsonNumberField(payload, "duty");
    iceboxLatest.cycles       = (uint32_t)jsonNumberField(payload, "cycles");
    iceboxLatest.lastUpdate = millis();
    iceboxLatest.valid = true;
  }
  // else: not from a sender we recognize - ignored
}

// ---------------------------------------------------------------------
// Cabin/outside temperature - 3x DS18B20 probes on a single OneWire bus.
// Needs an external ~4.7kohm pull-up resistor from the data line to
// 3.3V (standard OneWire requirement - the ESP32 pin alone isn't enough).
// ---------------------------------------------------------------------
#define TEMP_PROBE_PIN 4
OneWire oneWire(TEMP_PROBE_PIN);
DallasTemperature tempSensors(&oneWire);

const int TEMP_PROBE_COUNT = 3;
const char* TEMP_PROBE_NAMES[TEMP_PROBE_COUNT] = {"Cabin1", "Cabin2", "Outside"};

// Each probe's 6-byte unique serial (as printed on the probe itself - the
// DS18B20 family code 0x28 and CRC byte that make up the rest of a full
// 64-bit ROM address aren't part of that printed number, so they get
// filled in below). Looking sensors up by their actual address, rather
// than by bus-discovery order, means it doesn't matter what order they
// enumerate in at boot or whether one drops on/off the bus temporarily -
// each reading is always matched to the physical probe it came from.
const uint8_t TEMP_PROBE_SERIAL[TEMP_PROBE_COUNT][6] = {
  {0x03, 0x09, 0x97, 0x94, 0x0A, 0x0E}, // Cabin1
  {0x03, 0x0E, 0x97, 0x94, 0x5C, 0xD6}, // Cabin2
  {0x03, 0x01, 0x97, 0x94, 0x11, 0x84}, // Outside
};
DeviceAddress tempProbeAddr[TEMP_PROBE_COUNT]; // built from the serials above in setup()

const uint32_t TEMP_POLL_INTERVAL_MS = 10000; // how often to start a new round of reads
const uint32_t TEMP_CONVERSION_MS = 750;      // DS18B20 12-bit conversion time

struct {
  bool valid = false; // has at least one full round completed since boot?
  uint32_t lastUpdate = 0;
  double celsius[TEMP_PROBE_COUNT] = {DEVICE_DISCONNECTED_C, DEVICE_DISCONNECTED_C, DEVICE_DISCONNECTED_C};
} tempLatest;

uint32_t lastTempPollStart = 0;
bool tempConversionPending = false;

// Non-blocking, in two phases spread across separate loop() iterations.
// DS18B20 conversion takes ~750ms - blocking loop() for that long would
// stall the web server, BLE scanning, and GPS forwarding (and risks
// tripping the task watchdog). So this kicks off a conversion and returns
// immediately, then just checks on a later call whether enough time has
// passed to go collect the results - never sits in a delay().
void pollTemps() {
  uint32_t now = millis();
  if (!tempConversionPending) {
    if (now - lastTempPollStart < TEMP_POLL_INTERVAL_MS) return;
    lastTempPollStart = now;
    tempSensors.requestTemperatures(); // non-blocking - see setWaitForConversion(false) in setup()
    tempConversionPending = true;
    return;
  }
  if (now - lastTempPollStart < TEMP_CONVERSION_MS) return; // still converting, check back later
  for (int i = 0; i < TEMP_PROBE_COUNT; i++) {
    tempLatest.celsius[i] = tempSensors.getTempC(tempProbeAddr[i]); // DEVICE_DISCONNECTED_C if that probe's off the bus
  }
  tempLatest.lastUpdate = now;
  tempLatest.valid = true;
  tempConversionPending = false;
}

const char* chargeStateName(uint8_t state) {
  switch (state) {
    case CHARGER_OFF:              return "Off";
    case CHARGER_LOW_POWER:        return "Low power";
    case CHARGER_FAULT:            return "Fault";
    case CHARGER_BULK:             return "Bulk";
    case CHARGER_ABSORPTION:       return "Absorption";
    case CHARGER_FLOAT:            return "Float";
    case CHARGER_STORAGE:          return "Storage";
    case CHARGER_EQUALIZE:         return "Equalize";
    case CHARGER_INVERTING:        return "Inverting";
    case CHARGER_POWER_SUPPLY:     return "Power supply";
    case CHARGER_EXTERNAL_CONTROL: return "External control";
    default:                       return "Unknown";
  }
}

void onVictronData(const VictronDevice* dev) {
  if (dev->deviceType == DEVICE_TYPE_SOLAR_CHARGER) {
    solarLatest.data = dev->solar;
    solarLatest.rssi = dev->rssi;
    solarLatest.lastUpdate = millis();
    solarLatest.valid = true;
    recordSolarCurrentSample(dev->solar.batteryCurrent, solarLatest.lastUpdate);
  } else if (dev->deviceType == DEVICE_TYPE_BATTERY_MONITOR) {
    shuntLatest.data = dev->battery;
    shuntLatest.rssi = dev->rssi;
    shuntLatest.lastUpdate = millis();
    shuntLatest.valid = true;
  }
}

// ---------------------------------------------------------------------
// Web pages
// ---------------------------------------------------------------------

const char PAGE_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Victron Monitor</title>
<style>
  :root {
    --bg: #0f1720; --card: #182634; --text: #e8eef4; --muted: #8ea0b3;
    --accent: #3ddc84; --accent2: #ffb020; --blue: #5b9dff; --bad: #ff5c5c;
    --border: #24384a; --wire: #2c4055;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; padding: 24px; background: var(--bg); color: var(--text);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Arial, sans-serif;
  }
  h1 { font-size: 1.3rem; font-weight: 600; margin: 0 0 20px 0; text-align:center; }
  .wrap { max-width: 1000px; margin: 0 auto; }

  /* ---------- Flow diagram ---------- */
  .flow-card {
    background: var(--card); border: 1px solid var(--border); border-radius: 16px;
    padding: 20px; margin-bottom: 20px;
  }
  .flow-wrap {
    position: relative; width: 100%; max-width: 720px; margin: 0 auto;
    aspect-ratio: 8 / 3;
  }
  .flow-wrap svg { position: absolute; inset: 0; width: 100%; height: 100%; }
  .flow-line { fill: none; stroke: var(--wire); stroke-width: 3; stroke-linecap: round; }
  .flow-dash {
    fill: none; stroke-width: 3; stroke-linecap: round;
    stroke-dasharray: 8 9; opacity: 0; transition: opacity 0.3s;
    animation-name: flowmove; animation-timing-function: linear; animation-iteration-count: infinite;
    animation-play-state: paused;
  }
  .flow-dash.running { opacity: 1; animation-play-state: running; }
  .flow-dash.reverse { animation-direction: reverse; }
  @keyframes flowmove { to { stroke-dashoffset: -170; } }

  .flow-node {
    position: absolute; transform: translate(-50%, -50%);
    background: var(--bg); border: 1px solid var(--border); border-radius: 12px;
    padding: 10px 14px; text-align: center; min-width: 118px;
    box-shadow: 0 0 0 4px var(--card);
  }
  .flow-node .icon { font-size: 1.4rem; line-height: 1; }
  .flow-node .title { font-size: 0.72rem; color: var(--muted); text-transform: uppercase; letter-spacing: 0.04em; margin-top: 2px; }
  .flow-node .amps { font-size: 1.35rem; font-weight: 700; font-variant-numeric: tabular-nums; margin-top: 2px; }
  .flow-node .sub { font-size: 0.75rem; color: var(--muted); margin-top: 1px; font-variant-numeric: tabular-nums; }
  .flow-node.solar .amps { color: var(--accent2); }
  .flow-node.load .amps { color: var(--blue); }
  .flow-node.engine .amps { color: #c084fc; }
  .flow-node.battery.charging .amps { color: var(--accent); }
  .flow-node.battery.discharging .amps { color: var(--bad); }
  .flow-node.battery .dir-arrow { font-size: 0.85rem; margin-right: 2px; }


  /* ---------- Detail cards ---------- */
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); gap: 18px; }
  .card { background: var(--card); border: 1px solid var(--border); border-radius: 14px; padding: 20px 22px; }
  .card h2 { margin: 0 0 4px 0; font-size: 1.05rem; display: flex; align-items: center; gap: 8px; }
  .dot { width: 9px; height: 9px; border-radius: 50%; background: var(--muted); display: inline-block; }
  .dot.ok { background: var(--accent); }
  .dot.stale { background: var(--bad); }
  .sub-line { color: var(--muted); font-size: 0.8rem; margin-bottom: 14px; }
  .rows { display: grid; grid-template-columns: 1fr auto; row-gap: 10px; column-gap: 12px; }
  .rows .label { color: var(--muted); font-size: 0.9rem; }
  .rows .value { font-weight: 600; font-size: 1rem; text-align: right; font-variant-numeric: tabular-nums; }
  .value.big { font-size: 1.4rem; color: var(--accent); }
  .value.highlight-current {
    font-size: 1.3rem; font-weight: 700; color: var(--accent2);
    background: rgba(255,176,32,0.12); padding: 3px 10px; border-radius: 8px;
  }
  .split { display: flex; flex-direction: column; gap: 18px; }
  .split .half { flex: 1 1 0; min-width: 0; }
  .split .half + .half { padding-top: 18px; border-top: 1px solid var(--border); }
  .split .rows { grid-template-columns: 1fr auto; }
  .section-label {
    color: var(--muted); font-size: 0.78rem; margin: 16px 0 10px 0;
    padding-top: 16px; border-top: 1px solid var(--border);
  }
  .footer { text-align: center; color: var(--muted); font-size: 0.75rem; margin-top: 24px; }

  /* ---------- GPS status bar ---------- */
  .status-bar {
    display: flex; align-items: center; justify-content: center; gap: 18px;
    flex-wrap: wrap; margin: 0 0 18px 0; font-size: 0.85rem;
  }
  .status-pill {
    font-weight: 700; letter-spacing: 0.03em; color: var(--muted);
    padding: 3px 10px; border-radius: 8px; border: 1px solid var(--border);
  }
  .status-pill.ok   { color: var(--accent); border-color: var(--accent); }
  .status-pill.bad   { color: var(--bad); border-color: var(--bad); }
  .status-time { color: var(--muted); font-variant-numeric: tabular-nums; }
</style>
</head>
<body>
<div class="wrap">
  <h1>&#9889; Victron Monitor</h1>

  <div class="status-bar">
    <span class="status-pill" id="gps-pill">GPS</span>
    <span class="status-pill" id="ais-pill">AIS</span>
    <span class="status-time" id="gps-time-text">GPS time: no fix</span>
  </div>

  <div class="flow-card">
    <div class="flow-wrap">
      <svg viewBox="0 0 320 120" preserveAspectRatio="none">
        <path class="flow-line" d="M60,24 L260,24"/>
        <path class="flow-line" d="M60,24 L160,100"/>
        <path class="flow-line" d="M160,100 L260,24"/>
        <path class="flow-line" d="M60,100 L160,100"/>
        <path id="dash-solar" class="flow-dash" d="M60,24 L260,24" style="stroke:#ffb020"/>
        <path id="dash-batt"  class="flow-dash" d="M60,24 L160,100" style="stroke:#3ddc84"/>
        <path id="dash-load"  class="flow-dash" d="M160,100 L260,24" style="stroke:#5b9dff"/>
        <path id="dash-engine" class="flow-dash" d="M60,100 L160,100" style="stroke:#c084fc"/>
      </svg>

      <div class="flow-node solar" style="left:18.75%; top:20%;">
        <div class="icon">&#9728;&#65039;</div>
        <div class="title">Solar</div>
        <div class="amps" id="fd-solar-amps">&mdash;</div>
        <div class="sub" id="fd-solar-sub">&mdash;</div>
      </div>

      <div class="flow-node load" style="left:81.25%; top:20%;">
        <div class="icon">&#9973;</div>
        <div class="title">Loads (est.)</div>
        <div class="amps" id="fd-load-amps">&mdash;</div>
        <div class="sub" id="fd-load-sub">&mdash;</div>
      </div>

      <div class="flow-node engine" style="left:18.75%; top:83.3%;">
        <div class="icon">&#9881;&#65039;</div>
        <div class="title">Engine</div>
        <div class="amps" id="fd-engine-amps">&mdash;</div>
        <div class="sub" id="fd-engine-sub">&mdash;</div>
      </div>

      <div class="flow-node battery" id="fd-batt-node" style="left:50%; top:83.3%;">
        <div class="icon">&#128267;</div>
        <div class="title">Battery</div>
        <div class="amps"><span class="dir-arrow" id="fd-batt-arrow"></span><span id="fd-batt-amps">&mdash;</span></div>
        <div class="sub" id="fd-batt-sub">&mdash;</div>
      </div>
    </div>
  </div>

  <div class="grid">
    <div class="card">
      <h2><span class="dot" id="shunt-dot"></span>Battery Shunt</h2>
      <div class="sub-line" id="shunt-age">waiting for data&hellip;</div>
      <div class="rows">
        <div class="label">Voltage</div><div class="value big" id="shunt-voltage">&mdash;</div>
        <div class="label">Current</div><div class="value highlight-current" id="shunt-current">&mdash;</div>
        <div class="label">Power</div><div class="value" id="shunt-power">&mdash;</div>
        <div class="label">State of charge</div><div class="value" id="shunt-soc">&mdash;</div>
        <div class="label">Consumed</div><div class="value highlight-current" id="shunt-consumed">&mdash;</div>
        <div class="label">Time remaining</div><div class="value" id="shunt-remaining">&mdash;</div>
        <div class="label">Starter Battery</div><div class="value" id="shunt-aux">&mdash;</div>
        <div class="label">Alarms</div><div class="value" id="shunt-alarms">&mdash;</div>
        <div class="label">Signal</div><div class="value" id="shunt-rssi">&mdash;</div>
      </div>
    </div>
    <div class="card">
      <h2><span class="dot" id="solar-dot"></span>Solar Charger</h2>
      <div class="sub-line" id="solar-age">waiting for data&hellip;</div>
      <div class="rows">
        <div class="label">Charge state</div><div class="value big" id="solar-state">&mdash;</div>
        <div class="label">Battery voltage</div><div class="value" id="solar-voltage">&mdash;</div>
        <div class="label">Charge current</div><div class="value highlight-current" id="solar-current">&mdash;</div>
        <div class="label">Max current (10h)</div><div class="value" id="solar-max-current">&mdash;</div>
        <div class="label">Yield today</div><div class="value" id="solar-yield">&mdash;</div>
      </div>
      <div class="section-label">Temperatures</div>
      <div class="rows">
        <div class="label">Cabin1</div><div class="value" id="temp-cabin1">&mdash;</div>
        <div class="label">Cabin2</div><div class="value" id="temp-cabin2">&mdash;</div>
        <div class="label">Outside</div><div class="value" id="temp-outside">&mdash;</div>
      </div>
    </div>
    <div class="card">
      <div class="split">
        <div class="half">
          <h2><span class="dot" id="heater-dot"></span>Diesel Heater</h2>
          <div class="sub-line" id="heater-age">waiting for data&hellip;</div>
          <div class="rows">
            <div class="label">Burn rate</div><div class="value highlight-current" id="heater-rate">&mdash;</div>
            <div class="label">Used today</div><div class="value" id="heater-today">&mdash;</div>
            <div class="label">Lifetime used</div><div class="value" id="heater-total">&mdash;</div>
            <div class="label">Tank level (1h max)</div><div class="value" id="heater-tank-pct">&mdash;</div>
          </div>
        </div>
        <div class="half">
          <h2><span class="dot" id="icebox-dot"></span>Icebox</h2>
          <div class="sub-line" id="icebox-age">waiting for data&hellip;</div>
          <div class="rows">
            <div class="label">Fridge</div><div class="value big" id="icebox-fridge">&mdash;</div>
            <div class="label">Crisper</div><div class="value big" id="icebox-crisper">&mdash;</div>
            <div class="label">Compressor</div><div class="value" id="icebox-compressor">&mdash;</div>
            <div class="label">Duty Cycle</div><div class="value" id="icebox-duty">&mdash;</div>
          </div>
        </div>
      </div>
    </div>
  </div>
  <div class="footer">Auto-refreshing every 2s &middot; 192.168.8.66 &middot; <span id="sys-stats">uptime &mdash;</span></div>
</div>

<script>
function ageStr(ms) {
  if (ms == null) return "no data yet";
  const s = Math.round(ms / 1000);
  if (s < 5) return "updated just now";
  if (s < 60) return "updated " + s + "s ago";
  return "updated " + Math.round(s / 60) + "m ago";
}
function setDot(id, valid, ageMs) {
  const el = document.getElementById(id);
  el.className = "dot " + (!valid ? "" : (ageMs < 15000 ? "ok" : "stale"));
}

// GPS pill: green when sentences are actively arriving, red otherwise.
function setPill(id, valid, ageMs) {
  const el = document.getElementById(id);
  const connected = valid && ageMs < 15000;
  el.className = "status-pill " + (connected ? "ok" : "bad");
}

// Duration (seconds) of one dash cycle, faster for bigger current.
function speedFor(amps) {
  const mag = Math.max(Math.abs(amps), 0.05);
  return Math.max(3.6, Math.min(18, 20 / mag));
}

function setFlow(el, active, amps, reverse) {
  el.classList.toggle("running", active);
  el.classList.toggle("reverse", !!reverse);
  if (active) el.style.animationDuration = speedFor(amps) + "s";
}

async function refresh() {
  try {
    const res = await fetch("/data");
    const d = await res.json();
    const THRESH = 0.05;

    // ---- detail cards (unchanged data) ----
    setDot("shunt-dot", d.shunt.valid, d.shunt.ageMs);
    document.getElementById("shunt-age").textContent = ageStr(d.shunt.valid ? d.shunt.ageMs : null);
    if (d.shunt.valid) {
      document.getElementById("shunt-voltage").textContent = d.shunt.voltage.toFixed(2) + " V";
      document.getElementById("shunt-current").textContent = d.shunt.current.toFixed(2) + " A";
      document.getElementById("shunt-power").textContent = (d.shunt.voltage * d.shunt.current).toFixed(1) + " W";
      document.getElementById("shunt-soc").textContent = d.shunt.soc.toFixed(1) + " %";
      document.getElementById("shunt-consumed").textContent = d.shunt.consumedAh.toFixed(1) + " Ah";
      document.getElementById("shunt-remaining").textContent = d.shunt.remainingMinutes >= 65535 ? "—" : d.shunt.remainingMinutes + " min";
      document.getElementById("shunt-aux").textContent = d.shunt.auxVoltage.toFixed(2) + " V";
      const alarms = [];
      if (d.shunt.alarmLowVoltage) alarms.push("Low V");
      if (d.shunt.alarmHighVoltage) alarms.push("High V");
      if (d.shunt.alarmLowSOC) alarms.push("Low SOC");
      if (d.shunt.alarmLowTemperature) alarms.push("Low temp");
      if (d.shunt.alarmHighTemperature) alarms.push("High temp");
      document.getElementById("shunt-alarms").textContent = alarms.length ? alarms.join(", ") : "none";
      document.getElementById("shunt-rssi").textContent = d.shunt.rssi + " dBm";
    }

    setPill("gps-pill", d.gps.valid, d.gps.ageMs);
    setPill("ais-pill", d.ais.valid, d.ais.ageMs);
    const gpsTimeEl = document.getElementById("gps-time-text");
    gpsTimeEl.textContent = (d.gpsTime.valid && d.gpsTime.ageMs < 15000)
      ? d.gpsTime.text + " PT"
      : "GPS time: no fix";

    setDot("solar-dot", d.solar.valid, d.solar.ageMs);
    document.getElementById("solar-age").textContent = ageStr(d.solar.valid ? d.solar.ageMs : null);
    if (d.solar.valid) {
      document.getElementById("solar-state").textContent = d.solar.chargeState;
      document.getElementById("solar-voltage").textContent = d.solar.batteryVoltage.toFixed(2) + " V";
      document.getElementById("solar-current").textContent = d.solar.batteryCurrent.toFixed(2) + " A";
      document.getElementById("solar-max-current").textContent = d.solar.maxCurrent10h.toFixed(2) + " A";
      // Victron reports yield as cumulative Wh; converting to Ah needs a
      // voltage to divide by, so this uses the current battery voltage as
      // an approximation (it's not perfectly exact, since voltage drifts
      // over the day, but it's close enough for a rough Ah figure).
      const yieldAh = d.solar.batteryVoltage > 0 ? d.solar.yieldToday / d.solar.batteryVoltage : 0;
      document.getElementById("solar-yield").textContent = yieldAh.toFixed(1) + " Ah";
    }

    // Cabin/outside temp probes - shown per-probe rather than gated behind
    // one "valid" flag, since each DS18B20 is wired independently and can
    // drop out on its own without affecting the other two.
    const tempEls = { cabin1: "temp-cabin1", cabin2: "temp-cabin2", outside: "temp-outside" };
    const tempVals = { cabin1: d.temps.cabin1C, cabin2: d.temps.cabin2C, outside: d.temps.outsideC };
    for (const key in tempEls) {
      const el = document.getElementById(tempEls[key]);
      const v = tempVals[key];
      el.textContent = (v === null) ? "no probe" : v.toFixed(1) + " \u00b0C";
    }

    setDot("heater-dot", d.heater.valid, d.heater.ageMs);
    document.getElementById("heater-age").textContent = ageStr(d.heater.valid ? d.heater.ageMs : null);
    if (d.heater.valid) {
      document.getElementById("heater-rate").textContent = d.heater.rateLph.toFixed(2) + " L/hr";
      document.getElementById("heater-today").textContent = d.heater.todayLiters.toFixed(3) + " L";
      document.getElementById("heater-total").textContent = d.heater.totalLiters.toFixed(3) + " L";

      const tankPctEl = document.getElementById("heater-tank-pct");
      if (!d.heater.tankValid || d.heater.tankMaxPct1h === null) {
        tankPctEl.textContent = "no data";
      } else {
        // "stale" here describes CDHeater's *current* live reading (its N2K
        // bus has gone quiet), not the 1h-max figure itself, which is still
        // a real value seen sometime in the last hour - flag it so it's
        // clear the number may not reflect what's happening right now.
        const staleTag = d.heater.tankStale ? " (last live reading stale)" : "";
        tankPctEl.textContent = d.heater.tankMaxPct1h.toFixed(0) + " %" + staleTag;
      }
    }

    setDot("icebox-dot", d.icebox.valid, d.icebox.ageMs);
    document.getElementById("icebox-age").textContent = ageStr(d.icebox.valid ? d.icebox.ageMs : null);
    if (d.icebox.valid) {
      document.getElementById("icebox-fridge").textContent = d.icebox.fridgeTemp.toFixed(1) + " \u00b0C";
      document.getElementById("icebox-crisper").textContent = d.icebox.crisperTemp.toFixed(1) + " \u00b0C";
      document.getElementById("icebox-compressor").textContent = d.icebox.compressorOn ? "ON" : "OFF";
      document.getElementById("icebox-duty").textContent = d.icebox.dutyPercent + " %";
    }

    // ---- flow diagram ----
    const solarOk = d.solar.valid, battOk = d.shunt.valid;
    const solarA = solarOk ? d.solar.batteryCurrent : 0;
    const battA  = battOk ? d.shunt.current : 0;      // + into battery, - out of battery
    const voltage = battOk ? d.shunt.voltage : (solarOk ? d.solar.batteryVoltage : 0);
    // Single-bus estimate: whatever solar isn't putting into the battery
    // must be going to the loads (or vice versa if battery is covering
    // a shortfall). This is a calculated figure, not a direct measurement.
    const loadA = (solarOk && battOk) ? Math.max(0, solarA - battA) : null;

    document.getElementById("fd-solar-amps").textContent = solarOk ? solarA.toFixed(2) + " A" : "—";
    document.getElementById("fd-solar-sub").textContent = solarOk ? d.solar.panelPower.toFixed(0) + " W" : "no data";
    setFlow(document.getElementById("dash-solar"), solarOk && solarA > THRESH, solarA, false);

    document.getElementById("fd-load-amps").textContent = loadA !== null ? loadA.toFixed(2) + " A" : "—";
    document.getElementById("fd-load-sub").textContent = loadA !== null ? (loadA * voltage).toFixed(0) + " W (calc.)" : "need both devices";
    // The battery->load edge should only light up when current is actually
    // flowing that way, i.e. while the battery is discharging. When solar is
    // covering the load (battA >= 0), that current flows solar->load
    // directly (shown on the orange solar edge instead), so this edge must
    // stay dark even though loadA (the total load estimate shown on the
    // node itself) is still nonzero in that case.
    const battToLoadA = (battOk && battA < -THRESH) ? -battA : 0;
    setFlow(document.getElementById("dash-load"), battToLoadA > THRESH, battToLoadA, false);

    // Engine (alternator) current - calculated, not measured. If the
    // battery's being charged with more current than solar alone is
    // supplying, the extra must be coming from somewhere else - on a
    // boat, that's the engine's alternator. Only meaningful while
    // actually charging (battA > 0) and only when both readings are
    // available, since it's a difference of two other numbers.
    const engineOk = solarOk && battOk;
    const engineA = (engineOk && battA > solarA) ? (battA - solarA) : 0;
    document.getElementById("fd-engine-amps").textContent = engineOk ? engineA.toFixed(2) + " A" : "—";
    document.getElementById("fd-engine-sub").textContent = engineOk ? (engineA * voltage).toFixed(0) + " W (calc.)" : "need both devices";
    setFlow(document.getElementById("dash-engine"), engineOk && engineA > THRESH, engineA, false);

    const battNode = document.getElementById("fd-batt-node");
    const battArrow = document.getElementById("fd-batt-arrow");
    battNode.classList.remove("charging", "discharging");
    if (battOk) {
      document.getElementById("fd-batt-amps").textContent = Math.abs(battA).toFixed(2) + " A";
      document.getElementById("fd-batt-sub").textContent = d.shunt.voltage.toFixed(2) + " V \u00b7 " + d.shunt.soc.toFixed(0) + "%";
      if (battA > THRESH) {
        battNode.classList.add("charging"); battArrow.textContent = "\u25b2";
      } else if (battA < -THRESH) {
        battNode.classList.add("discharging"); battArrow.textContent = "\u25bc";
      } else {
        battArrow.textContent = "";
      }
    } else {
      document.getElementById("fd-batt-amps").textContent = "—";
      document.getElementById("fd-batt-sub").textContent = "no data";
      battArrow.textContent = "";
    }
    // This edge is physically solar->battery, so it should only animate
    // (forward) while the battery is actually charging. During discharge no
    // current flows back toward the solar controller along this path — the
    // discharge current goes to the load instead (handled by the
    // battery->load edge above) — so this edge just goes dark.
    setFlow(document.getElementById("dash-batt"), battOk && battA > THRESH, battA, false);

    // Heap health - comparing against the boot-time baseline (captured once
    // in setup()) is what actually tells a leak apart from a stable-but-
    // tight baseline: a steadily widening gap from "boot" here means a
    // real leak; a gap that opens once early on and then holds flat is
    // just normal settling, not an ongoing problem.
    if (d.system) {
      const uptimeH = (d.system.uptimeSec / 3600).toFixed(1);
      const freeKb = (d.system.freeHeap / 1024).toFixed(0);
      const bootKb = (d.system.bootFreeHeap / 1024).toFixed(0);
      const maxAllocKb = (d.system.maxAllocHeap / 1024).toFixed(0);
      document.getElementById("sys-stats").textContent =
        `uptime ${uptimeH}h \u00b7 heap ${bootKb}kB at boot \u2192 ${freeKb}kB now \u00b7 ${maxAllocKb}kB largest block`;
    }

  } catch (e) {
    console.error(e);
  }
}
refresh();
setInterval(refresh, 2000);
</script>
</body>
</html>
)HTML";

void handleRoot() {
  // Ask the browser not to reuse this connection (HTTP keep-alive) - the
  // classic ESP32 WebServer library is documented to become unstable when
  // a single connection gets reused for a very large number of requests
  // over a long period. NOT forcing the ESP32 side closed too (via
  // client().stop()) anymore - that call can block waiting on a TCP close
  // handshake, and since the actual lockup this was meant to fix is still
  // happening even with this header in place, adding that blocking risk
  // isn't earning its keep. Investigating further before doing more here.
  server.sendHeader("Connection", "close");
  server.send_P(200, "text/html", PAGE_HTML);
}

void handleData() {
  uint32_t now = millis();
  char buf[2000];

  // Highest tank level % seen in the last hour.
  double tankMaxPct = tankLevelMaxLast1h(now);
  bool tankMaxValid = tankMaxPct >= 0;

  // JSON null (not a quoted string) when there's no reading yet, so the
  // dashboard's `=== null` check works the same way it does for
  // hours_remaining on the CDHeater board itself.
  char tankMaxPctStr[16] = "null";
  if (tankMaxValid) snprintf(tankMaxPctStr, sizeof(tankMaxPctStr), "%.1f", tankMaxPct);

  // Pacific-time string derived from the GPS-set system clock (see
  // maybeUpdateGpsTime()). TZ is configured in setup() so localtime_r()
  // handles the PST/PDT switch automatically.
  char gpsTimeStr[40] = "";
  if (gpsTimeValid) {
    time_t nowEpoch = time(nullptr);
    struct tm localTm;
    localtime_r(&nowEpoch, &localTm);
    strftime(gpsTimeStr, sizeof(gpsTimeStr), "%Y-%m-%d %I:%M:%S %p %Z", &localTm);
  }

  // JSON null (not a quoted string) for a probe that's disconnected or
  // hasn't completed a first reading yet - same "null vs formatted number"
  // pattern used for the heater's tank level above.
  char cabin1Str[16] = "null", cabin2Str[16] = "null", outsideStr[16] = "null";
  if (tempLatest.celsius[0] != DEVICE_DISCONNECTED_C) snprintf(cabin1Str, sizeof(cabin1Str), "%.1f", tempLatest.celsius[0]);
  if (tempLatest.celsius[1] != DEVICE_DISCONNECTED_C) snprintf(cabin2Str, sizeof(cabin2Str), "%.1f", tempLatest.celsius[1]);
  if (tempLatest.celsius[2] != DEVICE_DISCONNECTED_C) snprintf(outsideStr, sizeof(outsideStr), "%.1f", tempLatest.celsius[2]);

  int len = snprintf(buf, sizeof(buf),
    "{"
      "\"shunt\":{"
        "\"valid\":%s,\"ageMs\":%lu,\"rssi\":%d,"
        "\"voltage\":%.3f,\"current\":%.3f,\"soc\":%.1f,"
        "\"consumedAh\":%.2f,\"remainingMinutes\":%u,"
        "\"temperature\":%.1f,\"auxVoltage\":%.3f,"
        "\"alarmLowVoltage\":%s,\"alarmHighVoltage\":%s,"
        "\"alarmLowSOC\":%s,\"alarmLowTemperature\":%s,\"alarmHighTemperature\":%s"
      "},"
      "\"solar\":{"
        "\"valid\":%s,\"ageMs\":%lu,"
        "\"chargeState\":\"%s\",\"batteryVoltage\":%.3f,\"batteryCurrent\":%.3f,"
        "\"panelPower\":%.1f,\"yieldToday\":%u,"
        "\"maxCurrent10h\":%.3f"
      "},"
      "\"temps\":{\"valid\":%s,\"ageMs\":%lu,\"cabin1C\":%s,\"cabin2C\":%s,\"outsideC\":%s},"
      "\"gps\":{\"valid\":%s,\"ageMs\":%lu},"
      "\"ais\":{\"valid\":%s,\"ageMs\":%lu},"
      "\"gpsTime\":{\"valid\":%s,\"ageMs\":%lu,\"text\":\"%s\"},"
      "\"heater\":{\"valid\":%s,\"ageMs\":%lu,\"rateLph\":%.2f,\"todayLiters\":%.3f,\"totalLiters\":%.3f,\"pulses\":%lu,"
        "\"tankValid\":%s,\"tankStale\":%s,\"tankCapacityLiters\":%.1f,"
        "\"tankMaxPct1h\":%s"
      "},"
      "\"icebox\":{\"valid\":%s,\"ageMs\":%lu,\"fridgeTemp\":%.1f,\"crisperTemp\":%.1f,\"compressorOn\":%s,\"dutyPercent\":%u},"
      "\"system\":{\"uptimeSec\":%lu,\"freeHeap\":%u,\"minFreeHeap\":%u,\"maxAllocHeap\":%u,\"bootFreeHeap\":%u}"
    "}",
    shuntLatest.valid ? "true" : "false",
    (unsigned long)(shuntLatest.valid ? now - shuntLatest.lastUpdate : 0),
    shuntLatest.rssi,
    shuntLatest.data.voltage, shuntLatest.data.current, shuntLatest.data.soc,
    shuntLatest.data.consumedAh, shuntLatest.data.remainingMinutes,
    shuntLatest.data.temperature, shuntLatest.data.auxVoltage,
    shuntLatest.data.alarmLowVoltage ? "true" : "false",
    shuntLatest.data.alarmHighVoltage ? "true" : "false",
    shuntLatest.data.alarmLowSOC ? "true" : "false",
    shuntLatest.data.alarmLowTemperature ? "true" : "false",
    shuntLatest.data.alarmHighTemperature ? "true" : "false",
    solarLatest.valid ? "true" : "false",
    (unsigned long)(solarLatest.valid ? now - solarLatest.lastUpdate : 0),
    chargeStateName(solarLatest.data.chargeState),
    solarLatest.data.batteryVoltage, solarLatest.data.batteryCurrent,
    solarLatest.data.panelPower, solarLatest.data.yieldToday,
    solarMaxCurrentLast10h(now),
    tempLatest.valid ? "true" : "false",
    (unsigned long)(tempLatest.valid ? now - tempLatest.lastUpdate : 0),
    cabin1Str, cabin2Str, outsideStr,
    gpsLinkStatus.everSeen ? "true" : "false",
    (unsigned long)(gpsLinkStatus.everSeen ? now - gpsLinkStatus.lastSentence : 0),
    aisLinkStatus.everSeen ? "true" : "false",
    (unsigned long)(aisLinkStatus.everSeen ? now - aisLinkStatus.lastByte : 0),
    gpsTimeValid ? "true" : "false",
    (unsigned long)(gpsTimeValid ? now - gpsTimeLastSet : 0),
    gpsTimeStr,
    heaterLatest.valid ? "true" : "false",
    (unsigned long)(heaterLatest.valid ? now - heaterLatest.lastUpdate : 0),
    heaterLatest.rateLph,
    heaterLatest.todayLiters,
    heaterLatest.totalLiters,
    (unsigned long)heaterLatest.pulses,
    heaterLatest.tankSeen ? "true" : "false",
    heaterLatest.tankStale ? "true" : "false",
    heaterLatest.tankCapacityLiters,
    tankMaxPctStr,
    iceboxLatest.valid ? "true" : "false",
    (unsigned long)(iceboxLatest.valid ? now - iceboxLatest.lastUpdate : 0),
    iceboxLatest.fridgeTempC,
    iceboxLatest.crisperTempC,
    iceboxLatest.compressorOn ? "true" : "false",
    iceboxLatest.dutyPercent,
    (unsigned long)(millis() / 1000),
    (unsigned)ESP.getFreeHeap(),
    (unsigned)ESP.getMinFreeHeap(),
    (unsigned)ESP.getMaxAllocHeap(),
    (unsigned)bootFreeHeap
  );

  (void)len; // buf is null-terminated by snprintf; length arg not needed here
  // See handleRoot() for why - this is the endpoint that actually matters
  // most, since it's the one hit every 2s by the dashboard's auto-refresh.
  server.sendHeader("Connection", "close");
  server.send(200, "application/json", buf);
}

void setup() {
  // Suppress logging from the underlying ESP-IDF components themselves
  // (WiFi driver, Bluedroid/BLE stack, lwIP) - this is a completely
  // separate system from CORE_DEBUG_LEVEL in platformio.ini, which only
  // covers Arduino-layer log_e()/log_w() calls. This board runs BLE
  // scanning and WiFi continuously for hours at a time, and either
  // subsystem can still write occasional log lines straight to the UART
  // independent of that setting - given a completely dead lockup was
  // observed after ~7h with no Serial Monitor attached, and it recovered
  // the instant a monitor was reattached (with no page reload or power
  // cycle), an unread, filling UART buffer is still the leading
  // explanation, and CORE_DEBUG_LEVEL alone evidently wasn't the whole
  // story. Called before Serial.begin() since it doesn't depend on our
  // own Serial object being ready - it's IDF's own internal log routing.
  esp_log_level_set("*", ESP_LOG_NONE);

  Serial.begin(115200);
  delay(200);
  Serial.println("\nVictron BLE Monitor starting...");

  // ---- Wi-Fi with static IP ----
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_HOSTNAME); // must be set after mode(), before begin()
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Static IP configuration failed!");
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  // Bounded wait, not an infinite loop - if the router isn't up yet (e.g.
  // everything on the boat powering up at once after a full shutdown),
  // the old code would sit here forever with nothing running: no
  // dashboard, no GPS forwarding, nothing - until someone noticed and
  // power-cycled it. Instead, give it a minute, then reboot and try the
  // whole sequence again; if the router comes up a bit later, this way
  // the board recovers on its own instead of needing a manual nudge.
  const uint32_t WIFI_CONNECT_TIMEOUT_MS = 60000;
  uint32_t wifiWaitStart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - wifiWaitStart > WIFI_CONNECT_TIMEOUT_MS) {
      Serial.println("\nWiFi didn't connect within 60s - restarting to retry.");
      delay(200); // let the Serial line flush before reset
      ESP.restart();
    }
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected. IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Hostname: ");
  Serial.println(DEVICE_HOSTNAME);

  // No mDNS responder - everything on this network (this dashboard, the
  // heater board, the icebox board) uses a fixed static IP already, so
  // .local name resolution isn't needed. One less always-on service
  // competing for the board's limited heap.

  // ---- Timezone: US Pacific (auto PST/PDT), used to render GPS time ----
  setenv("TZ", "PST8PDT7,M3.2.0,M11.1.0/2", 1);
  tzset();

  // ---- Victron BLE ----
  // This project only ever does BLE scanning - never Classic Bluetooth -
  // but the ESP32's Bluedroid stack reserves RAM for Classic BT by default
  // regardless. Releasing it *before* the BLE controller comes up (inside
  // victron.begin() below) hands that otherwise-wasted memory back to the
  // heap - commonly tens of KB, which is a big deal on a board this tight
  // on RAM. Must be called before any BT controller init happens, and it's
  // one-way for the rest of this boot (can't go back to using Classic BT
  // without a reboot) - fine here since we never use it.
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

  victron.begin(5); // 5 second scan window
  victron.setCallback(onVictronData);
  victron.setMinInterval(1000);
  // victron.setDebug(true); // uncomment for verbose BLE/decrypt logging

  victron.addDevice("Solar Charger", SOLAR_MAC, SOLAR_KEY, DEVICE_TYPE_SOLAR_CHARGER);
  victron.addDevice("Battery Shunt", SHUNT_MAC, SHUNT_KEY, DEVICE_TYPE_BATTERY_MONITOR);

  // ---- GPS NMEA0183 -> UDP broadcast ----
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, -1);   // RX only, no TX pin
  nmeaBroadcastIP = calculateBroadcast(local_IP, subnet);
  nmeaUdp.begin(0); // ephemeral local port; we only ever send
  Serial.print("NMEA0183 UDP broadcast -> ");
  Serial.print(nmeaBroadcastIP);
  Serial.print(":");
  Serial.println(NMEA_UDP_PORT);

  // ---- Heater + icebox telemetry UDP listener ----
  telemetryUdp.begin(TELEMETRY_UDP_PORT);
  Serial.print("Listening for heater/icebox telemetry on UDP:");
  Serial.println(TELEMETRY_UDP_PORT);

  // ---- AIS receiver UDP listener ----
  aisUdp.begin(AIS_UDP_PORT);
  Serial.print("Listening for AIS broadcast on UDP:");
  Serial.println(AIS_UDP_PORT);

  // ---- Web server ----
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  Serial.println("Web server started.");
  Serial.println("Dashboard: http://192.168.8.66/");

  // Snapshot free heap right after everything's up, so later readings can
  // be compared against a real starting point instead of one-off numbers -
  // that's the only way to tell a genuine leak (steadily dropping from
  // here) apart from a stable-but-tight baseline (flat, just lower than
  // you'd like) after a report of sluggishness.
  bootFreeHeap = ESP.getFreeHeap();
  Serial.printf("Boot free heap: %u bytes\n", (unsigned)bootFreeHeap);

  // ---- Task watchdog: auto-reboot if loop() ever gets stuck ----
  // If any single pass through loop() takes longer than this, something's
  // genuinely wedged (a hung network call that ignored its timeout, a
  // library deadlock, whatever) - rather than needing someone to notice
  // the dashboard is dead and physically power-cycle the board, this
  // forces a reboot on its own. 20s is generous compared to anything
  // loop() should normally take (worst case today is roughly ~2s, from
  // the heater/icebox HTTP timeouts), so it won't fire during normal
  // operation, only a real hang.
  esp_task_wdt_deinit(); // clear any watchdog the Arduino core already set up, so init below doesn't fail
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = 20000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // let idle tasks feed it too, avoids false trips
    .trigger_panic = true                            // panic -> automatic reboot
  };
  esp_task_wdt_init(&wdtConfig);
  esp_task_wdt_add(NULL); // subscribe this task (loop())
  Serial.println("Task watchdog armed (20s).");

  // ---- DS18B20 temperature probes ----
  tempSensors.begin();
  tempSensors.setWaitForConversion(false); // non-blocking - see pollTemps()

  // Build each probe's full 8-byte address from its labeled 6-byte serial:
  // [0]=0x28 (DS18B20 family code), [1..6]=serial, [7]=CRC (computed, not
  // hand-transcribed, so a typo here can't silently produce a wrong CRC
  // that happens to still look plausible).
  for (int i = 0; i < TEMP_PROBE_COUNT; i++) {
    tempProbeAddr[i][0] = 0x28;
    for (int b = 0; b < 6; b++) tempProbeAddr[i][1 + b] = TEMP_PROBE_SERIAL[i][b];
    tempProbeAddr[i][7] = OneWire::crc8(tempProbeAddr[i], 7);
  }

  int probesFound = tempSensors.getDeviceCount();
  Serial.printf("Temp probes found: %d (expected %d)\n", probesFound, TEMP_PROBE_COUNT);
  // Confirm each labeled probe is actually present, rather than finding
  // out only when its dashboard row silently shows "no probe" later.
  for (int i = 0; i < TEMP_PROBE_COUNT; i++) {
    bool present = tempSensors.isConnected(tempProbeAddr[i]);
    Serial.printf("  %-8s ", TEMP_PROBE_NAMES[i]);
    for (uint8_t b = 0; b < 8; b++) Serial.printf("%02X", tempProbeAddr[i][b]);
    Serial.println(present ? " - found" : " - NOT FOUND (check wiring)");
  }
}

void loop() {
  esp_task_wdt_reset(); // feed the watchdog - must happen every pass through loop()
  victron.loop();
  pumpNmea(gpsSerial, gpsLineBuf, sizeof(gpsLineBuf), gpsLineLen, gpsLinkStatus.everSeen, gpsLinkStatus.lastSentence);
  pollAis();
  pollTelemetry();
  pollTemps();
  server.handleClient();
}
