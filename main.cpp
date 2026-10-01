// Victron BLE Monitor - ESP32
//
// Scans Victron "Instant Readout" BLE advertisements from a SmartShunt
// (battery monitor) and two SmartSolar/BlueSolar MPPTs (Port & Starboard),
// decrypts them, and serves a live dashboard with a current-flow diagram.
//
// Library: scottp/victronble (installed automatically via platformio.ini)

#include <Arduino.h>
#include <string.h>  // strstr, strncmp - used by jsonNumberField/jsonBoolField
#include <stdlib.h>  // strtod - used by jsonNumberField
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <time.h>
#include <sys/time.h>
#include <Preferences.h>
#include "VictronBLE.h"
#include "esp_bt.h" // esp_bt_controller_mem_release() - see setup(), frees unused Classic BT RAM
#include <esp_task_wdt.h> // task watchdog - see setup()/loop(), auto-reboots on a stuck loop()
#include <esp_log.h> // esp_log_level_set() - see setup(), silences ESP-IDF component logging (WiFi/BLE/lwIP)
#include "secrets.h" // WIFI_SSID/PASSWORD, SOLAR_PORT_MAC/KEY, SOLAR_STBD_MAC/KEY, SHUNT_MAC/KEY
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>


// Network hostname - shows up in your router's client list / DHCP leases.
const char* DEVICE_HOSTNAME = "VictronGPS";

// TCP Navigation Data Endpoints
const char* GPS_HOST = "192.168.8.178";
const uint16_t GPS_PORT = 10110;
WiFiClient gpsClient;
String gpsRxBuf = "";
uint32_t lastGpsReconnect = 0;

const char* AIS_HOST = "192.168.8.144";
const uint16_t AIS_PORT = 9000;
WiFiClient aisClient;
String aisRxBuf = "";
uint32_t lastAisReconnect = 0;

struct {
  bool everSeen = false;
  uint32_t lastSentence = 0;
} gpsLinkStatus;

struct {
  bool everSeen = false;
  uint32_t lastByte = 0;
} aisLinkStatus;

bool gpsTimeValid = false;
uint32_t gpsTimeLastSet = 0;

struct {
  bool valid = false;
  uint32_t lastUpdate = 0;
  double lat = 0;
  double lon = 0;
  double speedKnots = 0;
  double courseDeg = 0;
} gpsFix;



// ---------------------------------------------------------------------
// Astronomical Solar Calculation Structure & Function
// ---------------------------------------------------------------------
struct SunTimes {
  bool valid = false;
  time_t civilDawn; // Sun at -6 deg: civil twilight / bright enough to navigate
  time_t sunrise;   // Sun at -0.833 deg
  time_t sunset;    // Sun at -0.833 deg
  time_t civilDusk; // Sun at -6 deg
};


// Convert a UTC struct tm into a time_t Unix timestamp without using TZ
time_t utcMktime(const struct tm& tm) {
    // Days per month in a non-leap year
    static const int daysBeforeMonth[] = {
        0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334
    };

    int year = tm.tm_year + 1900;
    int month = tm.tm_mon;

    // Calculate leap years since Epoch (1970)
    int leapYears = (year - 1969) / 4 - (year - 1901) / 100 + (year - 1601) / 400;

    long days = (year - 1970) * 365 + leapYears + daysBeforeMonth[month] + (tm.tm_mday - 1);

    // Add extra leap day if current year is a leap year and past February
    bool isLeap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
    if (isLeap && month > 1) {
        days++;
    }

    return (time_t)(days * 86400 + tm.tm_hour * 3600 + tm.tm_min * 60 + tm.tm_sec);
}

SunTimes calculateSunTimes(double lat, double lon, time_t epochUtc) {
  SunTimes result;
  if (lat == 0.0 && lon == 0.0) return result;

  struct tm tmUtc;
  gmtime_r(&epochUtc, &tmUtc);

  int N = tmUtc.tm_yday + 1;

  auto getSunTime = [&](double zenith, bool isRising) -> time_t {
    double lngHour = lon / 15.0;
    double t = N + ((isRising ? 6.0 : 18.0) - lngHour) / 24.0;

    double M = (0.9856 * t) - 3.289;

    double L = M + (1.916 * sin(M * M_PI / 180.0)) + (0.020 * sin(2 * M * M_PI / 180.0)) + 282.634;
    L = fmod(L, 360.0);
    if (L < 0) L += 360.0;

    double RA = atan(0.91764 * tan(L * M_PI / 180.0)) * 180.0 / M_PI;
    RA = fmod(RA, 360.0);
    if (RA < 0) RA += 360.0;

    double Lquadrant  = floor(L / 90.0) * 90.0;
    double RAquadrant = floor(RA / 90.0) * 90.0;
    RA = RA + (Lquadrant - RAquadrant);
    RA /= 15.0;

    double sinDec = 0.39782 * sin(L * M_PI / 180.0);
    double cosDec = cos(asin(sinDec));

    double cosH = (cos(zenith * M_PI / 180.0) - (sinDec * sin(lat * M_PI / 180.0))) / (cosDec * cos(lat * M_PI / 180.0));
    if (cosH > 1.0 || cosH < -1.0) return 0;

    double H = isRising ? (360.0 - acos(cosH) * 180.0 / M_PI) : (acos(cosH) * 180.0 / M_PI);
    H /= 15.0;

    double T = H + RA - (0.06571 * t) - 6.622;

    double UT = T - lngHour;
    
    // Compute midnight UTC for the given day using utcMktime
    struct tm tmDay = {};
    tmDay.tm_year = tmUtc.tm_year;
    tmDay.tm_mon  = tmUtc.tm_mon;
    tmDay.tm_mday = tmUtc.tm_mday;
    
    time_t dayStartUtc = utcMktime(tmDay);

    return dayStartUtc + (time_t)(UT * 3600.0);
  };

  result.civilDawn = getSunTime(96.0, true);
  result.sunrise   = getSunTime(90.833, true);
  result.sunset    = getSunTime(90.833, false);
  result.civilDusk = getSunTime(96.0, false);
  result.valid     = (result.sunrise > 0 && result.sunset > 0);

  return result;
}

// ---------------------------------------------------------------------
// 24-Hour History Structures & Global Variables
// ---------------------------------------------------------------------
const uint32_t SOLAR_24H_INTERVAL_MS = 10UL * 60 * 1000UL;
const int SOLAR_24H_SIZE = 144;
struct SolarHistoryPoint {
  uint32_t timestamp;
  float portCurrent;
  float stbdCurrent;
};
SolarHistoryPoint solar24hHistory[SOLAR_24H_SIZE];
int solar24hHead = 0;
int solar24hCount = 0;
uint32_t lastSolar24hSampleAt = 0;

const uint32_t SHUNT_24H_INTERVAL_MS = 10UL * 60 * 1000UL;
const int SHUNT_24H_SIZE = 144;
struct ShuntHistoryPoint {
  uint32_t timestamp;
  float current;
};
ShuntHistoryPoint shunt24hHistory[SHUNT_24H_SIZE];
int shunt24hHead = 0;
int shunt24hCount = 0;
uint32_t lastShunt24hSampleAt = 0;

// Shunt current accumulator for 10-minute continuous averaging
float shuntSumCurrent = 0;
uint32_t shuntSampleCount = 0;

const uint32_t TEMP_24H_INTERVAL_MS = 10UL * 60 * 1000UL;
const int TEMP_24H_SIZE = 144;
struct TempHistoryPoint {
  uint32_t timestamp;
  float cabin1;
  float cabin2;
  float outside;
};
TempHistoryPoint temp24hHistory[TEMP_24H_SIZE];
int temp24hHead = 0;
int temp24hCount = 0;
uint32_t lastTemp24hSampleAt = 0;

// ---------------------------------------------------------------------
// Persistent 30-Day Solar Yield & Daily Peak Storage (NVS)
// ---------------------------------------------------------------------
Preferences prefs;

struct SolarDailyPoint {
  uint32_t epochDay; // Day index: epochSeconds / 86400
  float portMaxAmps;
  float stbdMaxAmps;
  float portYieldAh;
  float stbdYieldAh;
};

const int SOLAR_30D_SIZE = 30;
SolarDailyPoint solar30dHistory[SOLAR_30D_SIZE];
int solar30dCount = 0;
uint32_t lastEpochDay = 0;

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

static long daysFromEpoch(int year, int month, int day) {
  year -= month <= 2;
  long era = (year >= 0 ? year : year - 399) / 400;
  unsigned yoe = (unsigned)(year - era * 400);
  unsigned doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (long)doe - 719468;
}


void maybeUpdateGpsTime(const String &line) {
  if (line.length() < 6 || line.charAt(0) != '$') return;
  if (line.substring(3, 6) != "RMC") return;
  if (nmeaField(line, 2) != "A") return;

  String timeField = nmeaField(line, 1);
  String dateField = nmeaField(line, 9);
  if (timeField.length() < 6 || dateField.length() < 6) return;

  struct tm t = {};
  t.tm_hour = timeField.substring(0, 2).toInt();
  t.tm_min  = timeField.substring(2, 4).toInt();
  t.tm_sec  = timeField.substring(4, 6).toInt();
  t.tm_mday = dateField.substring(0, 2).toInt();
  t.tm_mon  = dateField.substring(2, 4).toInt() - 1;
  t.tm_year = dateField.substring(4, 6).toInt() + 100;

  time_t utcEpoch = utcMktime(t);
  if (utcEpoch < 1700000000) return;

  bool firstTimeSync = !gpsTimeValid;

  struct timeval tv = { .tv_sec = utcEpoch, .tv_usec = 0 };
  settimeofday(&tv, nullptr);
  gpsTimeValid = true;
  gpsTimeLastSet = millis();

  if (firstTimeSync) {
    uint32_t oldTimeSec = millis() / 1000;
    int32_t timeDelta = (int32_t)utcEpoch - (int32_t)oldTimeSec;

    for (int i = 0; i < solar24hCount; i++) solar24hHistory[i].timestamp += timeDelta;
    for (int i = 0; i < shunt24hCount; i++) shunt24hHistory[i].timestamp += timeDelta;
    for (int i = 0; i < temp24hCount; i++) temp24hHistory[i].timestamp += timeDelta;
  }
}

double nmeaCoordToDecimal(const String &field, char hemisphere) {
  if (field.length() < 4) return 0;
  int degLen = (hemisphere == 'E' || hemisphere == 'W') ? 3 : 2;
  double deg = field.substring(0, degLen).toDouble();
  double minutes = field.substring(degLen).toDouble();
  double decimal = deg + minutes / 60.0;
  if (hemisphere == 'S' || hemisphere == 'W') decimal = -decimal;
  return decimal;
}

void maybeUpdateGpsPosition(const String &line) {
  if (line.length() < 6 || line.charAt(0) != '$') return;
  if (line.substring(3, 6) != "RMC") return;
  if (nmeaField(line, 2) != "A") return;

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

void processNmeaSentence(const String &line) {
  uint32_t now = millis();
  if (line.startsWith("$")) {
    gpsLinkStatus.everSeen = true;
    gpsLinkStatus.lastSentence = now;
    maybeUpdateGpsTime(line);
    maybeUpdateGpsPosition(line);
  } else if (line.startsWith("!")) {
    aisLinkStatus.everSeen = true;
    aisLinkStatus.lastByte = now;
  }
}

const uint32_t GPS_TIMEOUT_MS = 10000; // 10-second timeout for missing data

void pollGpsTcp() {
  uint32_t now = millis();

  // 1. Force disconnect if the connection was lost or timed out due to no incoming data
  if (gpsClient.connected() && gpsLinkStatus.everSeen && (now - gpsLinkStatus.lastSentence > GPS_TIMEOUT_MS)) {
    Serial.println("[GPS] Connection timed out (no data received). Reconnecting...");
    gpsClient.stop();
  }

  // 2. Handle reconnection attempt
  if (!gpsClient.connected()) {
    if (now - lastGpsReconnect > 5000) {
      lastGpsReconnect = now;
      gpsClient.stop(); // Ensure socket resources are freed before connecting
      Serial.println("[GPS] Attempting TCP reconnect...");
      if (gpsClient.connect(GPS_HOST, GPS_PORT)) {
        Serial.println("[GPS] Connected successfully.");
        // Refresh lastSentence timestamp so it doesn't instantly trigger a timeout on reconnect
        gpsLinkStatus.lastSentence = millis(); 
      }
    }
    return;
  }

  // 3. Process incoming data
  while (gpsClient.available() > 0) {
    char c = gpsClient.read();
    if (c == '\r') continue;
    if (c == '\n') {
      gpsRxBuf.trim();
      if (gpsRxBuf.length() > 0) {
        processNmeaSentence(gpsRxBuf);
      }
      gpsRxBuf = "";
    } else {
      gpsRxBuf += c;
      if (gpsRxBuf.length() > 256) gpsRxBuf = "";
    }
  }
}

void pollAisTcp() {
  uint32_t now = millis();

  if (!aisClient.connected()) {
    if (now - lastAisReconnect > 5000) {
      lastAisReconnect = now;
      aisClient.stop();
      aisClient.connect(AIS_HOST, AIS_PORT);
    }
    return;
  }

  while (aisClient.available() > 0) {
    char c = aisClient.read();
    if (c == '\r') continue;
    if (c == '\n') {
      aisRxBuf.trim();
      if (aisRxBuf.length() > 0) {
        processNmeaSentence(aisRxBuf);
      }
      aisRxBuf = "";
    } else {
      aisRxBuf += c;
      if (aisRxBuf.length() > 256) aisRxBuf = "";
    }
  }
}

VictronBLE victron;
WebServer server(80);

struct SolarDataHolder {
  bool valid = false;
  uint32_t lastUpdate = 0;
  int8_t rssi = 0;
  VictronSolarData data;
};

SolarDataHolder solarPortLatest;
SolarDataHolder solarStbdLatest;

struct {
  bool valid = false;
  uint32_t lastUpdate = 0;
  int8_t rssi = 0;
  VictronBatteryData data;
} shuntLatest;

const uint32_t SOLAR_CURRENT_BUCKET_MS = 10UL * 60 * 1000UL;
const int SOLAR_CURRENT_NUM_BUCKETS = 60;

struct SolarCurrentTracker {
  double maxCurrent[SOLAR_CURRENT_NUM_BUCKETS] = {0};
  uint32_t bucketEpoch[SOLAR_CURRENT_NUM_BUCKETS] = {0};
};

SolarCurrentTracker solarPortHistoryTracker;
SolarCurrentTracker solarStbdHistoryTracker;

void recordSolarCurrentSample(SolarCurrentTracker &tracker, double amps, uint32_t now) {
  uint32_t epoch = now / SOLAR_CURRENT_BUCKET_MS + 1;
  int idx = epoch % SOLAR_CURRENT_NUM_BUCKETS;
  if (tracker.bucketEpoch[idx] != epoch) {
    tracker.bucketEpoch[idx] = epoch;
    tracker.maxCurrent[idx] = amps;
  } else if (amps > tracker.maxCurrent[idx]) {
    tracker.maxCurrent[idx] = amps;
  }
}

double solarMaxCurrentLast10h(const SolarCurrentTracker &tracker, uint32_t now) {
  uint32_t currentEpoch = now / SOLAR_CURRENT_BUCKET_MS + 1;
  double best = 0;
  bool any = false;
  for (int i = 0; i < SOLAR_CURRENT_NUM_BUCKETS; i++) {
    uint32_t epoch = tracker.bucketEpoch[i];
    if (epoch == 0) continue;
    if (currentEpoch - epoch >= (uint32_t)SOLAR_CURRENT_NUM_BUCKETS) continue;
    if (!any || tracker.maxCurrent[i] > best) {
      best = tracker.maxCurrent[i];
      any = true;
    }
  }
  return any ? best : 0;
}

IPAddress HEATER_IP(192, 168, 8, 65);
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
  bool tankSeen = false;
  bool tankStale = false;
  double tankPct = -1;
  double tankCapacityLiters = -1;
} heaterLatest;

IPAddress ICEBOX_IP(192, 168, 8, 67);

struct {
  bool valid = false;
  uint32_t lastUpdate = 0;
  double fridgeTempC = 0;
  double crisperTempC = 0;
  bool compressorOn = false;
  uint8_t dutyPercent = 0;
  uint32_t cycles = 0;
} iceboxLatest;

const uint32_t TANK_LEVEL_BUCKET_MS = 2UL * 60 * 1000UL;
const int TANK_LEVEL_NUM_BUCKETS = 30;

struct {
  double maxPct[TANK_LEVEL_NUM_BUCKETS] = {0};
  uint32_t bucketEpoch[TANK_LEVEL_NUM_BUCKETS] = {0};
} tankLevelHistory;

void recordTankLevelSample(double pct, uint32_t now) {
  uint32_t epoch = now / TANK_LEVEL_BUCKET_MS + 1;
  int idx = epoch % TANK_LEVEL_NUM_BUCKETS;
  if (tankLevelHistory.bucketEpoch[idx] != epoch) {
    tankLevelHistory.bucketEpoch[idx] = epoch;
    tankLevelHistory.maxPct[idx] = pct;
  } else if (pct > tankLevelHistory.maxPct[idx]) {
    tankLevelHistory.maxPct[idx] = pct;
  }
}

double tankLevelMaxLast1h(uint32_t now) {
  uint32_t currentEpoch = now / TANK_LEVEL_BUCKET_MS + 1;
  double best = -1;
  for (int i = 0; i < TANK_LEVEL_NUM_BUCKETS; i++) {
    uint32_t epoch = tankLevelHistory.bucketEpoch[i];
    if (epoch == 0) continue;
    if (currentEpoch - epoch >= (uint32_t)TANK_LEVEL_NUM_BUCKETS) continue;
    if (tankLevelHistory.maxPct[i] > best) best = tankLevelHistory.maxPct[i];
  }
  return best;
}

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

bool jsonBoolField(const String &json, const char* key) {
  char needle[40];
  int nlen = snprintf(needle, sizeof(needle), "\"%s\":", key);
  const char* hit = strstr(json.c_str(), needle);
  if (!hit) return false;
  return strncmp(hit + nlen, "true", 4) == 0;
}

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
}

#define TEMP_PROBE_PIN 4
OneWire oneWire(TEMP_PROBE_PIN);
DallasTemperature tempSensors(&oneWire);

const int TEMP_PROBE_COUNT = 3;
const char* TEMP_PROBE_NAMES[TEMP_PROBE_COUNT] = {"Cabin1", "Cabin2", "Outside"};

const uint8_t TEMP_PROBE_SERIAL[TEMP_PROBE_COUNT][6] = {
  {0xD6, 0x5C, 0x94, 0x97, 0x0E, 0x03},
  {0x0E, 0x0A, 0x94, 0x97, 0x09, 0x03},
  {0x84, 0x11, 0x94, 0x97, 0x01, 0x03},
};
DeviceAddress tempProbeAddr[TEMP_PROBE_COUNT];

const uint32_t TEMP_POLL_INTERVAL_MS = 10000;
const uint32_t TEMP_CONVERSION_MS = 750;

struct {
  bool valid = false;
  uint32_t lastUpdate = 0;
  double celsius[TEMP_PROBE_COUNT] = {DEVICE_DISCONNECTED_C, DEVICE_DISCONNECTED_C, DEVICE_DISCONNECTED_C};
} tempLatest;

uint32_t lastTempPollStart = 0;
bool tempConversionPending = false;

void pollTemps() {
  uint32_t now = millis();
  if (!tempConversionPending) {
    if (now - lastTempPollStart < TEMP_POLL_INTERVAL_MS) return;
    lastTempPollStart = now;
    tempSensors.requestTemperatures();
    tempConversionPending = true;
    return;
  }
  if (now - lastTempPollStart < TEMP_CONVERSION_MS) return;
  for (int i = 0; i < TEMP_PROBE_COUNT; i++) {
    tempLatest.celsius[i] = tempSensors.getTempC(tempProbeAddr[i]);
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
    if (strcmp(dev->name, "Port Solar") == 0) {
      solarPortLatest.data = dev->solar;
      solarPortLatest.rssi = dev->rssi;
      solarPortLatest.lastUpdate = millis();
      solarPortLatest.valid = true;
      recordSolarCurrentSample(solarPortHistoryTracker, dev->solar.batteryCurrent, solarPortLatest.lastUpdate);
      Serial.printf("[BLE] Port Solar Charger updated | RSSI: %d dBm\n", dev->rssi);
    } else if (strcmp(dev->name, "Starboard Solar") == 0) {
      solarStbdLatest.data = dev->solar;
      solarStbdLatest.rssi = dev->rssi;
      solarStbdLatest.lastUpdate = millis();
      solarStbdLatest.valid = true;
      recordSolarCurrentSample(solarStbdHistoryTracker, dev->solar.batteryCurrent, solarStbdLatest.lastUpdate);
      Serial.printf("[BLE] Starboard Solar Charger updated | RSSI: %d dBm\n", dev->rssi);
    }
  } else if (dev->deviceType == DEVICE_TYPE_BATTERY_MONITOR) {
    shuntLatest.data = dev->battery;
    shuntLatest.rssi = dev->rssi;
    shuntLatest.lastUpdate = millis();
    shuntLatest.valid = true;

    shuntSumCurrent += (float)dev->battery.current;
    shuntSampleCount++;
  }
}

// ---------------------------------------------------------------------
// NVS Helpers & Midnight Rollover Check
// ---------------------------------------------------------------------
void loadSolar30dHistory() {
  prefs.begin("solar30d", true);
  solar30dCount = prefs.getBytes("history", solar30dHistory, sizeof(solar30dHistory)) / sizeof(SolarDailyPoint);
  prefs.end();
  if (solar30dCount > SOLAR_30D_SIZE || solar30dCount < 0) solar30dCount = 0;
}

void saveSolar30dHistory() {
  prefs.begin("solar30d", false);
  prefs.putBytes("history", solar30dHistory, solar30dCount * sizeof(SolarDailyPoint));
  prefs.end();
}

// Static holders to track peak yield reached throughout the current day
static float portDailyPeakYieldAh = 0.0f;
static float stbdDailyPeakYieldAh = 0.0f;

void pollSolar30dHistory() {
  if (!gpsTimeValid) return;

  // Continuously track the highest yield reached today before midnight reset
  if (solarPortLatest.valid && solarPortLatest.data.batteryVoltage > 0) {
    float portAh = (float)solarPortLatest.data.yieldToday / solarPortLatest.data.batteryVoltage;
    if (portAh > portDailyPeakYieldAh) portDailyPeakYieldAh = portAh;
  }
  if (solarStbdLatest.valid && solarStbdLatest.data.batteryVoltage > 0) {
    float stbdAh = (float)solarStbdLatest.data.yieldToday / solarStbdLatest.data.batteryVoltage;
    if (stbdAh > stbdDailyPeakYieldAh) stbdDailyPeakYieldAh = stbdAh;
  }

  time_t nowEpoch = time(nullptr);
  struct tm localTm;
  localtime_r(&nowEpoch, &localTm);

  uint32_t currentLocalDay = (uint32_t)localTm.tm_mday;

  if (lastEpochDay == 0) {
    lastEpochDay = currentLocalDay;
    return;
  }

  // Midnight Rollover
  if (currentLocalDay != lastEpochDay) {
    uint32_t localEpochDay = (uint32_t)((nowEpoch - 28800L) / 86400L);

    SolarDailyPoint newPoint = {
      localEpochDay,
      (float)solarMaxCurrentLast10h(solarPortHistoryTracker, millis()),
      (float)solarMaxCurrentLast10h(solarStbdHistoryTracker, millis()),
      portDailyPeakYieldAh, // Log stored peak yield
      stbdDailyPeakYieldAh
    };

    if (solar30dCount < SOLAR_30D_SIZE) {
      solar30dHistory[solar30dCount++] = newPoint;
    } else {
      for (int i = 0; i < SOLAR_30D_SIZE - 1; i++) {
        solar30dHistory[i] = solar30dHistory[i + 1];
      }
      solar30dHistory[SOLAR_30D_SIZE - 1] = newPoint;
    }

    saveSolar30dHistory();
    
    // Reset trackers for the new day
    portDailyPeakYieldAh = 0.0f;
    stbdDailyPeakYieldAh = 0.0f;
    lastEpochDay = currentLocalDay;
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
    --accent: #3ddc84; --accent2: #ffb020; --orange: #ffb020; --blue: #5b9dff; --bad: #ff5c5c;
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
  .card h2, .card h1 { margin: 0 0 4px 0; font-size: 1.05rem; display: flex; align-items: center; justify-content: space-between; gap: 8px; }
  .card .header-title { display: flex; align-items: center; gap: 8px; }
  .dot { width: 9px; height: 9px; border-radius: 50%; background: var(--muted); display: inline-block; }
  .dot.ok, .dot.green-dot { background: var(--accent); }
  .dot.stale { background: var(--bad); }
  .sub-line { color: var(--muted); font-size: 0.8rem; margin-bottom: 14px; }
  .rows { display: grid; grid-template-columns: 1fr auto; row-gap: 10px; column-gap: 12px; }
  .rows .label { color: var(--muted); font-size: 0.9rem; }
  .rows .value { font-weight: 600; font-size: 1rem; text-align: right; font-variant-numeric: tabular-nums; }
  .value.big { font-size: 1.4rem; color: var(--accent); }
  .value.green { color: var(--accent); }
  .split { display: flex; flex-direction: column; gap: 18px; }
  .split .half { flex: 1 1 0; min-width: 0; }
  .split .half + .half { padding-top: 18px; border-top: 1px solid var(--border); }
  .split .rows { grid-template-columns: 1fr auto; }
  .section-label {
    color: var(--muted); font-size: 0.78rem; margin: 16px 0 10px 0;
    padding-top: 16px; border-top: 1px solid var(--border);
  }
  .sub-header-text { font-size: 0.9rem; font-weight: normal; color: var(--muted); margin-right: 0.6em; }

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
    <span class="status-pill" id="gps-time-text">--:--:-- --</span>
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
      <h2><span class="header-title"><span class="dot" id="shunt-dot"></span><a href="/shunt" style="color: inherit; text-decoration: none;">Battery Shunt</a></span></h2>
      <div class="sub-line" id="shunt-age">waiting for data&hellip;</div>
      <div class="rows">
        <div class="label">Voltage</div><div class="value" id="shunt-voltage">&mdash;</div>
        <div class="label">Current</div><div class="value big" id="shunt-current">&mdash;</div>
        <div class="label">State of charge</div><div class="value" id="shunt-soc">&mdash;</div>
        <div class="label">Consumed</div><div class="value green big" id="shunt-consumed">&mdash;</div>
        <div class="label">Starter Battery</div><div class="value" id="shunt-aux">&mdash;</div>
        <div class="label">Alarms</div><div class="value" id="shunt-alarms">&mdash;</div>
      </div>

      <!-- Sunrise / Sunset Status Indicator -->
      <div class="section-label" style="border-top: 1px solid var(--border); padding-top: 16px; margin-top: 16px;">
        <h2 style="margin: 0;">
          <span class="header-title">
            <span class="dot green-dot" id="sun-dot"></span>
            Sunrise / Sunset
          </span>
        </h2>
      </div>

      <div class="rows">
        <div class="label">Civil Dawn</div><div class="value green big" id="sun-dawn">&mdash;</div>
        <div class="label">Sunrise / Sunset</div><div class="value" id="sun-rise-set">&mdash; / &mdash;</div>
        <div class="label">Civil Dusk</div><div class="value green big" id="sun-dusk">&mdash;</div>
      </div>
    </div>
    <div class="card">
      <h2><span class="header-title"><span class="dot" id="solar-dot"></span><a href="/solar" style="color: inherit; text-decoration: none;">Solar Chargers</a></span><span class="sub-header-text">Port / Stbd</span></h2>
      <div class="sub-line" id="solar-age">waiting for data&hellip;</div>
      <div class="rows">
        <div class="label">Charge state</div><div class="value big" id="solar-state">&mdash; / &mdash;</div>
        <div class="label">Battery voltage (V)</div><div class="value" id="solar-voltage">&mdash; / &mdash;</div>
        <div class="label">Charge current (A)</div><div class="value green big" id="solar-current">&mdash; / &mdash;</div>
        <div class="label">Max current 10h (A)</div><div class="value" id="solar-max-current">&mdash; / &mdash;</div>
        <div class="label"><a href="/solar-yield" style="color: inherit; text-decoration: none; text-underline-offset: 3px;">Yield today (Ah)</a></div><div class="value" id="solar-yield">&mdash; / &mdash;</div>
      </div>

      <div class="section-label" style="border-top: 1px solid var(--border); padding-top: 16px; margin-top: 16px;">
      <h1 style="color: var(--text);"><span class="header-title"><span class="dot" id="temps-dot"></span><a href="/temps" style="color: inherit; text-decoration: none;">Temperatures</a></span></h1>
      </div>
      <div class="rows">
        <div class="label">Cabin1</div><div class="value green big" id="temp-cabin1">&mdash;</div>
        <div class="label">Cabin2</div><div class="value green big" id="temp-cabin2">&mdash;</div>
        <div class="label">Outside</div><div class="value green big" id="temp-outside">&mdash;</div>
      </div>
    </div>
    <div class="card">
      <div class="split">
        <div class="half">
          <h2><span class="header-title"><span class="dot" id="heater-dot"></span><a href="http://192.168.8.65/" style="color: inherit; text-decoration: none;">Diesel Heater</a></span></h2>
          <div class="sub-line" id="heater-age">waiting for data&hellip;</div>
          <div class="rows">
            <div class="label">Burn rate</div><div class="value green big" id="heater-rate">&mdash;</div>
            <div class="label">Used today</div><div class="value" id="heater-today">&mdash;</div>
            <div class="label">Lifetime used</div><div class="value" id="heater-total">&mdash;</div>
            <div class="label">Tank level (1h max)</div><div class="value" id="heater-tank-pct">&mdash;</div>
          </div>
        </div>
        <div class="half">
          <h2><span class="header-title"><span class="dot" id="icebox-dot"></span><a href="http://192.168.8.67/" style="color: inherit; text-decoration: none;">Icebox</a></span></h2>
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

function setPill(id, valid, ageMs, maxAgeMs = 15000) {
  const el = document.getElementById(id);
  const connected = valid && ageMs < maxAgeMs;
  el.className = "status-pill " + (connected ? "ok" : "bad");
}

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

    setDot("shunt-dot", d.shunt.valid, d.shunt.ageMs);
    document.getElementById("shunt-age").textContent = ageStr(d.shunt.valid ? d.shunt.ageMs : null);
   if (d.shunt.valid) {
     const voltageEl = document.getElementById("shunt-voltage");
     const voltageV = d.shunt.voltage;
     voltageEl.textContent = voltageV.toFixed(2) + " V";

  if (voltageV < 12.9) {
    voltageEl.style.color = "var(--bad)";      // Red
  } else if (voltageV < 13.0) {
    voltageEl.style.color = "var(--accent2)";  // Yellow / Orange
  } else {
    voltageEl.style.color = "var(--text)";     // Default light text
  }
      const currentEl = document.getElementById("shunt-current");
      const currentA = d.shunt.current;
      currentEl.textContent = currentA.toFixed(2) + " A";

      if (currentA < -THRESH) {
        currentEl.style.color = "var(--bad)";
      } else if (currentA > THRESH) {
        currentEl.style.color = "var(--accent)";
      } else {
        currentEl.style.color = "var(--text)";
      }
      document.getElementById("shunt-soc").textContent = d.shunt.soc.toFixed(1) + " %";
      
      const consumedEl = document.getElementById("shunt-consumed");
      const ah = d.shunt.consumedAh;
      consumedEl.textContent = ah.toFixed(1) + " Ah";

      if (ah <= -350) {
        consumedEl.style.color = "var(--bad)";
      } else if (ah <= -300) {
        consumedEl.style.color = "var(--orange)";
      } else if (ah <= -200) {
        consumedEl.style.color = "var(--accent2)";
      } else {
        consumedEl.style.color = "var(--accent)";
      }

      document.getElementById("shunt-aux").textContent = d.shunt.auxVoltage.toFixed(2) + " V";
      const alarms = [];
      if (d.shunt.alarmLowVoltage) alarms.push("Low V");
      if (d.shunt.alarmHighVoltage) alarms.push("High V");
      if (d.shunt.alarmLowSOC) alarms.push("Low SOC");
      if (d.shunt.alarmLowTemperature) alarms.push("Low temp");
      if (d.shunt.alarmHighTemperature) alarms.push("High temp");
      document.getElementById("shunt-alarms").textContent = alarms.length ? alarms.join(", ") : "none";
    }

    if (d.sun && d.sun.valid) {
      document.getElementById("sun-dawn").textContent = d.sun.dawn;
      document.getElementById("sun-rise-set").textContent = d.sun.rise + " / " + d.sun.set;
      document.getElementById("sun-dusk").textContent = d.sun.dusk;
    } else {
      document.getElementById("sun-dawn").textContent = "No GPS fix";
      document.getElementById("sun-rise-set").textContent = "— / —";
      document.getElementById("sun-dusk").textContent = "No GPS fix";
    }

    setPill("gps-pill", d.gps.valid, d.gps.ageMs);
    setPill("ais-pill", d.ais.valid, d.ais.ageMs, 60000);
    const gpsTimeValid = d.gpsTime.valid && d.gpsTime.ageMs < 15000;
    setPill("gps-time-text", d.gpsTime.valid, d.gpsTime.ageMs);
    document.getElementById("gps-time-text").textContent = gpsTimeValid
      ? d.gpsTime.text
      : "NO FIX";

    const portOk = d.solarPort && d.solarPort.valid;
    const stbdOk = d.solarStbd && d.solarStbd.valid;
    const solarValid = portOk || stbdOk;
    const minSolarAge = Math.min(portOk ? d.solarPort.ageMs : Infinity, stbdOk ? d.solarStbd.ageMs : Infinity);

    setDot("solar-dot", solarValid, minSolarAge);
    document.getElementById("solar-age").textContent = ageStr(solarValid ? minSolarAge : null);

    const portState = portOk ? d.solarPort.chargeState : "—";
    const stbdState = stbdOk ? d.solarStbd.chargeState : "—";
    document.getElementById("solar-state").textContent = portState + " / " + stbdState;

    const portVolts = portOk ? d.solarPort.batteryVoltage.toFixed(2) : "—";
    const stbdVolts = stbdOk ? d.solarStbd.batteryVoltage.toFixed(2) : "—";
    document.getElementById("solar-voltage").textContent = portVolts + " / " + stbdVolts;

    const portCurr = portOk ? d.solarPort.batteryCurrent : 0;
    const stbdCurr = stbdOk ? d.solarStbd.batteryCurrent : 0;
    const portCurrStr = portOk ? portCurr.toFixed(2) : "—";
    const stbdCurrStr = stbdOk ? stbdCurr.toFixed(2) : "—";
    document.getElementById("solar-current").textContent = portCurrStr + " / " + stbdCurrStr;

    const portMax = portOk ? d.solarPort.maxCurrent10h.toFixed(2) : "—";
    const stbdMax = stbdOk ? d.solarStbd.maxCurrent10h.toFixed(2) : "—";
    document.getElementById("solar-max-current").textContent = portMax + " / " + stbdMax;

    const portYieldAh = (portOk && d.solarPort.batteryVoltage > 0) ? (d.solarPort.yieldToday / d.solarPort.batteryVoltage) : 0;
    const stbdYieldAh = (stbdOk && d.solarStbd.batteryVoltage > 0) ? (d.solarStbd.yieldToday / d.solarStbd.batteryVoltage) : 0;
    const portYieldStr = portOk ? portYieldAh.toFixed(1) : "—";
    const stbdYieldStr = stbdOk ? stbdYieldAh.toFixed(1) : "—";
    document.getElementById("solar-yield").textContent = portYieldStr + " / " + stbdYieldStr;

    setDot("temps-dot", d.temps.valid, d.temps.ageMs);
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

    const solarOk = solarValid, battOk = d.shunt.valid;
    const solarA = portCurr + stbdCurr;
    const portW = portOk ? d.solarPort.panelPower : 0;
    const stbdW = stbdOk ? d.solarStbd.panelPower : 0;
    const solarW = portW + stbdW;

    const battA  = battOk ? d.shunt.current : 0;
    const voltage = battOk ? d.shunt.voltage : (portOk ? d.solarPort.batteryVoltage : (stbdOk ? d.solarStbd.batteryVoltage : 0));
    const loadA = (solarOk && battOk) ? Math.max(0, solarA - battA) : null;

    document.getElementById("fd-solar-amps").textContent = solarOk ? solarA.toFixed(2) + " A" : "—";
    document.getElementById("fd-solar-sub").textContent = solarOk ? solarW.toFixed(0) + " W" : "no data";
    setFlow(document.getElementById("dash-solar"), solarOk && solarA > THRESH, solarA, false);

    document.getElementById("fd-load-amps").textContent = loadA !== null ? loadA.toFixed(2) + " A" : "—";
    document.getElementById("fd-load-sub").textContent = loadA !== null ? (loadA * voltage).toFixed(0) + " W (calc.)" : "need both devices";
    const battToLoadA = (battOk && battA < -THRESH) ? -battA : 0;
    setFlow(document.getElementById("dash-load"), battToLoadA > THRESH, battToLoadA, false);

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
    setFlow(document.getElementById("dash-batt"), battOk && battA > THRESH, battA, false);

    if (d.system) {
      const uptimeH = (d.system.uptimeSec / 3600).toFixed(1);
      document.getElementById("sys-stats").textContent =
        `uptime ${uptimeH}h`;
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

// ---------------------------------------------------------------------
// Common Time Helper
// ---------------------------------------------------------------------
uint32_t currentTimestamp() {
  time_t t = time(nullptr);
  if (t > 1700000000L) return (uint32_t)t;
  return millis() / 1000;
}

// ---------------------------------------------------------------------
// Rolling 24-hour Solar Charge Current History (Port & Starboard)
// ---------------------------------------------------------------------
void pollSolarHistory() {
  if (!solarPortLatest.valid && !solarStbdLatest.valid) return;
  uint32_t now = millis();
  if (lastSolar24hSampleAt != 0 && now - lastSolar24hSampleAt < SOLAR_24H_INTERVAL_MS) return;
  lastSolar24hSampleAt = now;

  float portCurrent = solarPortLatest.valid ? (float)solarPortLatest.data.batteryCurrent : 0.0f;
  float stbdCurrent = solarStbdLatest.valid ? (float)solarStbdLatest.data.batteryCurrent : 0.0f;

  solar24hHistory[solar24hHead] = { currentTimestamp(), portCurrent, stbdCurrent };
  solar24hHead = (solar24hHead + 1) % SOLAR_24H_SIZE;
  if (solar24hCount < SOLAR_24H_SIZE) solar24hCount++;
}

char solarHistoryBuf[8192];

void handleSolarHistory() {
  int pos = 0;
  pos += snprintf(solarHistoryBuf + pos, sizeof(solarHistoryBuf) - pos, "[");
  int start = (solar24hHead - solar24hCount + SOLAR_24H_SIZE) % SOLAR_24H_SIZE;
  for (int i = 0; i < solar24hCount; i++) {
    if (pos >= (int)sizeof(solarHistoryBuf) - 64) break;
    int idx = (start + i) % SOLAR_24H_SIZE;
    pos += snprintf(solarHistoryBuf + pos, sizeof(solarHistoryBuf) - pos,
                     "%s{\"t\":%u,\"p\":%.2f,\"s\":%.2f}",
                     (i > 0) ? "," : "",
                     solar24hHistory[idx].timestamp,
                     solar24hHistory[idx].portCurrent,
                     solar24hHistory[idx].stbdCurrent);
  }
  pos += snprintf(solarHistoryBuf + pos, sizeof(solarHistoryBuf) - pos, "]");
  server.sendHeader("Connection", "close");
  server.send(200, "application/json", solarHistoryBuf);
}

const char SOLAR_PAGE_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Solar History (24h)</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4"></script>
<style>
  :root {
    --bg: #0f1720; --card: #182634; --text: #e8eef4; --muted: #8ea0b3;
    --accent2: #ffb020; --blue: #5b9dff; --border: #24384a;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; padding: 24px; background: var(--bg); color: var(--text);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Arial, sans-serif;
  }
  .wrap { max-width: 900px; margin: 0 auto; }
  .header { display: flex; align-items: center; justify-content: space-between; margin-bottom: 20px; }
  h1 { font-size: 1.3rem; font-weight: 600; margin: 0; }
  .back-btn {
    color: var(--muted); text-decoration: none; font-size: 0.9rem;
    padding: 6px 12px; border: 1px solid var(--border); border-radius: 8px; background: var(--card);
  }
  .back-btn:hover { color: var(--text); }
  .card { background: var(--card); border: 1px solid var(--border); border-radius: 14px; padding: 20px; }
</style>
</head>
<body>
<div class="wrap">
  <div class="header">
    <h1>Solar Charge Current (Last 24 Hours)</h1>
    <a href="/" class="back-btn">&larr; Dashboard</a>
  </div>
  <div class="card">
    <canvas id="solarChart" height="120"></canvas>
  </div>
</div>

<script>
let solarChart;
function fmtTime(ts) {
  const d = new Date(ts * 1000);
  return d.toLocaleTimeString([], {hour:'2-digit', minute:'2-digit'});
}
async function loadHistory() {
  try {
    const r = await fetch('/solar-history');
    const data = await r.json();
    const labels = data.map(p => fmtTime(p.t));
    const portCurrents = data.map(p => p.p);
    const stbdCurrents = data.map(p => p.s);

    if (!solarChart) {
      solarChart = new Chart(document.getElementById('solarChart'), {
        type: 'line',
        data: {
          labels,
          datasets: [
            {
              label: 'Port Solar (A)',
              data: portCurrents,
              borderColor: '#ffb020',
              backgroundColor: 'rgba(255, 176, 32, 0.1)',
              fill: false,
              tension: 0.3,
              pointRadius: 0
            },
            {
              label: 'Starboard Solar (A)',
              data: stbdCurrents,
              borderColor: '#5b9dff',
              backgroundColor: 'rgba(91, 157, 255, 0.1)',
              fill: false,
              tension: 0.3,
              pointRadius: 0
            }
          ]
        },
        options: {
          responsive: true,
          scales: {
            x: { ticks: { color: '#8ea0b3', maxTicksLimit: 12 } },
            y: { ticks: { color: '#8ea0b3' }, beginAtZero: true }
          },
          plugins: {
            legend: { labels: { color: '#e8eef4' } }
          }
        }
      });
    } else {
      solarChart.data.labels = labels;
      solarChart.data.datasets[0].data = portCurrents;
      solarChart.data.datasets[1].data = stbdCurrents;
      solarChart.update();
    }
  } catch (e) {
    console.error(e);
  }
}
loadHistory();
setInterval(loadHistory, 60000);
</script>
</body>
</html>
)HTML";

void handleSolarPage() {
  server.sendHeader("Connection", "close");
  server.send_P(200, "text/html", SOLAR_PAGE_HTML);
}

// ---------------------------------------------------------------------
// 30-Day Solar Yield & Peak Current Web Page & Endpoint
// ---------------------------------------------------------------------
char solar30dBuf[4096];

void handleSolar30dHistory() {
  int pos = 0;
  pos += snprintf(solar30dBuf + pos, sizeof(solar30dBuf) - pos, "[");
  for (int i = 0; i < solar30dCount; i++) {
    if (pos >= (int)sizeof(solar30dBuf) - 128) break;
    pos += snprintf(solar30dBuf + pos, sizeof(solar30dBuf) - pos,
                     "%s{\"day\":%u,\"pMax\":%.2f,\"sMax\":%.2f,\"pYield\":%.1f,\"sYield\":%.1f}",
                     (i > 0) ? "," : "",
                     solar30dHistory[i].epochDay,
                     solar30dHistory[i].portMaxAmps,
                     solar30dHistory[i].stbdMaxAmps,
                     solar30dHistory[i].portYieldAh,
                     solar30dHistory[i].stbdYieldAh);
  }
  pos += snprintf(solar30dBuf + pos, sizeof(solar30dBuf) - pos, "]");
  server.sendHeader("Connection", "close");
  server.send(200, "application/json", solar30dBuf);
}

const char YIELD_PAGE_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Solar Yield (30 Days)</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4"></script>
<style>
  :root {
    --bg: #0f1720; --card: #182634; --text: #e8eef4; --muted: #8ea0b3;
    --accent2: #ffb020; --blue: #5b9dff; --border: #24384a;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; padding: 24px; background: var(--bg); color: var(--text);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Arial, sans-serif;
  }
  .wrap { max-width: 900px; margin: 0 auto; }
  .header { display: flex; align-items: center; justify-content: space-between; margin-bottom: 20px; }
  h1 { font-size: 1.3rem; font-weight: 600; margin: 0; }
  .back-btn {
    color: var(--muted); text-decoration: none; font-size: 0.9rem;
    padding: 6px 12px; border: 1px solid var(--border); border-radius: 8px; background: var(--card);
  }
  .back-btn:hover { color: var(--text); }
  .card { background: var(--card); border: 1px solid var(--border); border-radius: 14px; padding: 20px; margin-bottom: 20px; }
</style>
</head>
<body>
<div class="wrap">
  <div class="header">
    <h1>Solar Daily Performance (Last 30 Days)</h1>
    <a href="/" class="back-btn">&larr; Dashboard</a>
  </div>
  <div class="card">
    <canvas id="yieldChart" height="120"></canvas>
  </div>
  <div class="card">
    <canvas id="maxAmpsChart" height="120"></canvas>
  </div>
</div>

<script>
function fmtDay(epochDay) {
  const d = new Date(epochDay * 86400 * 1000);
  return (d.getUTCMonth() + 1) + '/' + d.getUTCDate();
}

async function load30d() {
  try {
    const r = await fetch('/solar-yield-history');
    const data = await r.json();
    const labels = data.map(p => fmtDay(p.day));

    new Chart(document.getElementById('yieldChart'), {
      type: 'bar',
      data: {
        labels,
        datasets: [
          { label: 'Port Yield (Ah)', data: data.map(p => p.pYield), backgroundColor: '#ffb020' },
          { label: 'Stbd Yield (Ah)', data: data.map(p => p.sYield), backgroundColor: '#5b9dff' }
        ]
      },
      options: {
        responsive: true,
        scales: {
          x: { ticks: { color: '#8ea0b3' } },
          y: { ticks: { color: '#8ea0b3' }, beginAtZero: true }
        },
        plugins: {
          title: { display: true, text: 'Daily Yield (Ah)', color: '#e8eef4' },
          legend: { labels: { color: '#e8eef4' } }
        }
      }
    });

    new Chart(document.getElementById('maxAmpsChart'), {
      type: 'line',
      data: {
        labels,
        datasets: [
          { label: 'Port Peak (A)', data: data.map(p => p.pMax), borderColor: '#ffb020', backgroundColor: 'rgba(255, 176, 32, 0.1)', tension: 0.2 },
          { label: 'Stbd Peak (A)', data: data.map(p => p.sMax), borderColor: '#5b9dff', backgroundColor: 'rgba(91, 157, 255, 0.1)', tension: 0.2 }
        ]
      },
      options: {
        responsive: true,
        scales: {
          x: { ticks: { color: '#8ea0b3' } },
          y: { ticks: { color: '#8ea0b3' }, beginAtZero: true }
        },
        plugins: {
          title: { display: true, text: 'Daily Peak Current (A)', color: '#e8eef4' },
          legend: { labels: { color: '#e8eef4' } }
        }
      }
    });
  } catch (e) {
    console.error(e);
  }
}
load30d();
</script>
</body>
</html>
)HTML";

void handleYieldPage() {
  server.sendHeader("Connection", "close");
  server.send_P(200, "text/html", YIELD_PAGE_HTML);
}

// ---------------------------------------------------------------------
// Rolling 24-hour Battery Shunt Current History (10-min interval averaging)
// ---------------------------------------------------------------------
void pollShuntHistory() {
  uint32_t now = millis();
  
  if (lastShunt24hSampleAt != 0 && now - lastShunt24hSampleAt < SHUNT_24H_INTERVAL_MS) return;
  lastShunt24hSampleAt = now;

  if (shuntSampleCount > 0) {
    float avgCurrent = shuntSumCurrent / (float)shuntSampleCount;

    shunt24hHistory[shunt24hHead] = { currentTimestamp(), avgCurrent };
    shunt24hHead = (shunt24hHead + 1) % SHUNT_24H_SIZE;
    if (shunt24hCount < SHUNT_24H_SIZE) shunt24hCount++;

    shuntSumCurrent = 0;
    shuntSampleCount = 0;
  }
}

char shuntHistoryBuf[8192];

void handleShuntHistory() {
  int pos = 0;
  pos += snprintf(shuntHistoryBuf + pos, sizeof(shuntHistoryBuf) - pos, "[");
  int start = (shunt24hHead - shunt24hCount + SHUNT_24H_SIZE) % SHUNT_24H_SIZE;
  for (int i = 0; i < shunt24hCount; i++) {
    if (pos >= (int)sizeof(shuntHistoryBuf) - 64) break;
    int idx = (start + i) % SHUNT_24H_SIZE;
    pos += snprintf(shuntHistoryBuf + pos, sizeof(shuntHistoryBuf) - pos,
                     "%s{\"t\":%u,\"c\":%.2f}",
                     (i > 0) ? "," : "",
                     shunt24hHistory[idx].timestamp, shunt24hHistory[idx].current);
  }
  pos += snprintf(shuntHistoryBuf + pos, sizeof(shuntHistoryBuf) - pos, "]");
  server.sendHeader("Connection", "close");
  server.send(200, "application/json", shuntHistoryBuf);
}

const char SHUNT_PAGE_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Battery Shunt History (24h)</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4"></script>
<style>
  :root {
    --bg: #0f1720; --card: #182634; --text: #e8eef4; --muted: #8ea0b3;
    --accent: #3ddc84; --border: #24384a;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; padding: 24px; background: var(--bg); color: var(--text);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Arial, sans-serif;
  }
  .wrap { max-width: 900px; margin: 0 auto; }
  .header { display: flex; align-items: center; justify-content: space-between; margin-bottom: 20px; }
  h1 { font-size: 1.3rem; font-weight: 600; margin: 0; }
  .back-btn {
    color: var(--muted); text-decoration: none; font-size: 0.9rem;
    padding: 6px 12px; border: 1px solid var(--border); border-radius: 8px; background: var(--card);
  }
  .back-btn:hover { color: var(--text); }
  .card { background: var(--card); border: 1px solid var(--border); border-radius: 14px; padding: 20px; }
</style>
</head>
<body>
<div class="wrap">
  <div class="header">
    <h1>Battery Current (Last 24 Hours)</h1>
    <a href="/" class="back-btn">&larr; Dashboard</a>
  </div>
  <div class="card">
    <canvas id="shuntChart" height="120"></canvas>
  </div>
</div>

<script>
let shuntChart;
function fmtTime(ts) {
  const d = new Date(ts * 1000);
  return d.toLocaleTimeString([], {hour:'2-digit', minute:'2-digit'});
}
async function loadHistory() {
  try {
    const r = await fetch('/shunt-history');
    const data = await r.json();
    const labels = data.map(p => fmtTime(p.t));
    const currents = data.map(p => p.c);

    if (!shuntChart) {
      shuntChart = new Chart(document.getElementById('shuntChart'), {
        type: 'line',
        data: {
          labels,
          datasets: [{
            label: 'Current (A)',
            data: currents,
            borderColor: '#3ddc84',
            backgroundColor: 'rgba(61, 220, 132, 0.1)',
            fill: true,
            tension: 0.3,
            pointRadius: 0
          }]
        },
        options: {
          responsive: true,
          scales: {
            x: { ticks: { color: '#8ea0b3', maxTicksLimit: 12 } },
            y: { ticks: { color: '#8ea0b3' } }
          },
          plugins: {
            legend: { labels: { color: '#e8eef4' } }
          }
        }
      });
    } else {
      shuntChart.data.labels = labels;
      shuntChart.data.datasets[0].data = currents;
      shuntChart.update();
    }
  } catch (e) {
    console.error(e);
  }
}
loadHistory();
setInterval(loadHistory, 60000);
</script>
</body>
</html>
)HTML";

void handleShuntPage() {
  server.sendHeader("Connection", "close");
  server.send_P(200, "text/html", SHUNT_PAGE_HTML);
}

// ---------------------------------------------------------------------
// Rolling 24-hour Temperature History
// ---------------------------------------------------------------------
void pollTempHistory() {
  if (!tempLatest.valid) return;
  uint32_t now = millis();
  if (lastTemp24hSampleAt != 0 && now - lastTemp24hSampleAt < TEMP_24H_INTERVAL_MS) return;
  lastTemp24hSampleAt = now;

  float c1 = (tempLatest.celsius[0] != DEVICE_DISCONNECTED_C) ? (float)tempLatest.celsius[0] : -999.0f;
  float c2 = (tempLatest.celsius[1] != DEVICE_DISCONNECTED_C) ? (float)tempLatest.celsius[1] : -999.0f;
  float out = (tempLatest.celsius[2] != DEVICE_DISCONNECTED_C) ? (float)tempLatest.celsius[2] : -999.0f;

  temp24hHistory[temp24hHead] = { currentTimestamp(), c1, c2, out };
  temp24hHead = (temp24hHead + 1) % TEMP_24H_SIZE;
  if (temp24hCount < TEMP_24H_SIZE) temp24hCount++;
}

char tempHistoryBuf[12288];

void handleTempsHistory() {
  int pos = 0;
  pos += snprintf(tempHistoryBuf + pos, sizeof(tempHistoryBuf) - pos, "[");
  int start = (temp24hHead - temp24hCount + TEMP_24H_SIZE) % TEMP_24H_SIZE;
  for (int i = 0; i < temp24hCount; i++) {
    if (pos >= (int)sizeof(tempHistoryBuf) - 128) break;
    int idx = (start + i) % TEMP_24H_SIZE;
    pos += snprintf(tempHistoryBuf + pos, sizeof(tempHistoryBuf) - pos,
                     "%s{\"t\":%u,\"c1\":%.1f,\"c2\":%.1f,\"out\":%.1f}",
                     (i > 0) ? "," : "",
                     temp24hHistory[idx].timestamp,
                     temp24hHistory[idx].cabin1,
                     temp24hHistory[idx].cabin2,
                     temp24hHistory[idx].outside);
  }
  pos += snprintf(tempHistoryBuf + pos, sizeof(tempHistoryBuf) - pos, "]");
  server.sendHeader("Connection", "close");
  server.send(200, "application/json", tempHistoryBuf);
}

const char TEMPS_PAGE_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Temperature History (24h)</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4"></script>
<style>
  :root {
    --bg: #0f1720; --card: #182634; --text: #e8eef4; --muted: #8ea0b3;
    --accent: #3ddc84; --blue: #5b9dff; --orange: #ffb020; --border: #24384a;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; padding: 24px; background: var(--bg); color: var(--text);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Arial, sans-serif;
  }
  .wrap { max-width: 900px; margin: 0 auto; }
  .header { display: flex; align-items: center; justify-content: space-between; margin-bottom: 20px; }
  h1 { font-size: 1.3rem; font-weight: 600; margin: 0; }
  .back-btn {
    color: var(--muted); text-decoration: none; font-size: 0.9rem;
    padding: 6px 12px; border: 1px solid var(--border); border-radius: 8px; background: var(--card);
  }
  .back-btn:hover { color: var(--text); }
  .card { background: var(--card); border: 1px solid var(--border); border-radius: 14px; padding: 20px; }
</style>
</head>
<body>
<div class="wrap">
  <div class="header">
    <h1>Temperatures (Last 24 Hours)</h1>
    <a href="/" class="back-btn">&larr; Dashboard</a>
  </div>
  <div class="card">
    <canvas id="tempChart" height="120"></canvas>
  </div>
</div>

<script>
let tempChart;
function fmtTime(ts) {
  const d = new Date(ts * 1000);
  return d.toLocaleTimeString([], {hour:'2-digit', minute:'2-digit'});
}
async function loadHistory() {
  try {
    const r = await fetch('/temps-history');
    const data = await r.json();
    const labels = data.map(p => fmtTime(p.t));
    const c1Vals = data.map(p => p.c1 === -999 ? null : p.c1);
    const c2Vals = data.map(p => p.c2 === -999 ? null : p.c2);
    const outVals = data.map(p => p.out === -999 ? null : p.out);

    if (!tempChart) {
      tempChart = new Chart(document.getElementById('tempChart'), {
        type: 'line',
        data: {
          labels,
          datasets: [
            {
              label: 'Cabin1 (°C)',
              data: c1Vals,
              borderColor: '#3ddc84',
              backgroundColor: 'rgba(61, 220, 132, 0.1)',
              tension: 0.3,
              pointRadius: 0,
              spanGaps: true
            },
            {
              label: 'Cabin2 (°C)',
              data: c2Vals,
              borderColor: '#5b9dff',
              backgroundColor: 'rgba(91, 157, 255, 0.1)',
              tension: 0.3,
              pointRadius: 0,
              spanGaps: true
            },
            {
              label: 'Outside (°C)',
              data: outVals,
              borderColor: '#ffb020',
              backgroundColor: 'rgba(255, 176, 32, 0.1)',
              tension: 0.3,
              pointRadius: 0,
              spanGaps: true
            }
          ]
        },
        options: {
          responsive: true,
          scales: {
            x: { ticks: { color: '#8ea0b3', maxTicksLimit: 12 } },
            y: { ticks: { color: '#8ea0b3' } }
          },
          plugins: {
            legend: { labels: { color: '#e8eef4' } }
          }
        }
      });
    } else {
      tempChart.data.labels = labels;
      tempChart.data.datasets[0].data = c1Vals;
      tempChart.data.datasets[1].data = c2Vals;
      tempChart.data.datasets[2].data = outVals;
      tempChart.update();
    }
  } catch (e) {
    console.error(e);
  }
}
loadHistory();
setInterval(loadHistory, 60000);
</script>
</body>
</html>
)HTML";

void handleTempsPage() {
  server.sendHeader("Connection", "close");
  server.send_P(200, "text/html", TEMPS_PAGE_HTML);
}

void handleRoot() {
  server.sendHeader("Connection", "close");
  server.send_P(200, "text/html", PAGE_HTML);
}

void handleData() {
  uint32_t now = millis();
  char buf[2800];

  double tankMaxPct = tankLevelMaxLast1h(now);
  bool tankMaxValid = tankMaxPct >= 0;

  char tankMaxPctStr[16] = "null";
  if (tankMaxValid) snprintf(tankMaxPctStr, sizeof(tankMaxPctStr), "%.1f", tankMaxPct);

  char gpsTimeStr[40] = "";
  if (gpsTimeValid) {
    time_t nowEpoch = time(nullptr);
    struct tm localTm;
    localtime_r(&nowEpoch, &localTm);
    strftime(gpsTimeStr, sizeof(gpsTimeStr), "%I:%M:%S %p", &localTm);
  }

  char dawnStr[16] = "null", riseStr[16] = "null", setStr[16] = "null", duskStr[16] = "null";
  bool sunValid = false;
  if (gpsFix.valid && gpsTimeValid) {
    time_t nowUtc = time(nullptr);
    SunTimes st = calculateSunTimes(gpsFix.lat, gpsFix.lon, nowUtc);
    if (st.valid) {
      struct tm t;
      localtime_r(&st.civilDawn, &t); strftime(dawnStr, sizeof(dawnStr), "\"%I:%M %p\"", &t);
      localtime_r(&st.sunrise, &t);   strftime(riseStr, sizeof(riseStr), "\"%I:%M %p\"", &t);
      localtime_r(&st.sunset, &t);    strftime(setStr, sizeof(setStr), "\"%I:%M %p\"", &t);
      localtime_r(&st.civilDusk, &t);  strftime(duskStr, sizeof(duskStr), "\"%I:%M %p\"", &t);
      sunValid = true;
    }
  }

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
      "\"sun\":{\"valid\":%s,\"dawn\":%s,\"rise\":%s,\"set\":%s,\"dusk\":%s},"
      "\"solarPort\":{"
        "\"valid\":%s,\"ageMs\":%lu,"
        "\"chargeState\":\"%s\",\"batteryVoltage\":%.3f,\"batteryCurrent\":%.3f,"
        "\"panelPower\":%.1f,\"yieldToday\":%u,"
        "\"maxCurrent10h\":%.3f"
      "},"
      "\"solarStbd\":{"
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
    sunValid ? "true" : "false",
    dawnStr, riseStr, setStr, duskStr,
    solarPortLatest.valid ? "true" : "false",
    (unsigned long)(solarPortLatest.valid ? now - solarPortLatest.lastUpdate : 0),
    chargeStateName(solarPortLatest.data.chargeState),
    solarPortLatest.data.batteryVoltage, solarPortLatest.data.batteryCurrent,
    solarPortLatest.data.panelPower, solarPortLatest.data.yieldToday,
    solarMaxCurrentLast10h(solarPortHistoryTracker, now),
    solarStbdLatest.valid ? "true" : "false",
    (unsigned long)(solarStbdLatest.valid ? now - solarStbdLatest.lastUpdate : 0),
    chargeStateName(solarStbdLatest.data.chargeState),
    solarStbdLatest.data.batteryVoltage, solarStbdLatest.data.batteryCurrent,
    solarStbdLatest.data.panelPower, solarStbdLatest.data.yieldToday,
    solarMaxCurrentLast10h(solarStbdHistoryTracker, now),
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
    (unsigned long)(millis() / 1000)
  );

  (void)len;
  server.sendHeader("Connection", "close");
  server.send(200, "application/json", buf);
}

uint32_t wifiDownSince = 0;
uint32_t lastReconnectAttempt = 0;
const uint32_t WIFI_RECONNECT_RETRY_MS = 15000UL;
const uint32_t WIFI_FORCE_RESTART_MS   = 180000UL;

void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiDownSince = 0;
    return;
  }
  uint32_t now = millis();
  if (wifiDownSince == 0) {
    wifiDownSince = now;
    Serial.println("WiFi dropped - will attempt to reconnect");
  }
  if (now - wifiDownSince > WIFI_FORCE_RESTART_MS) {
    Serial.println("WiFi down for 3+ minutes - restarting");
    delay(200);
    ESP.restart();
  }
  if (now - lastReconnectAttempt > WIFI_RECONNECT_RETRY_MS) {
    lastReconnectAttempt = now;
    Serial.println("Attempting WiFi reconnect...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

void setup() {
  esp_log_level_set("*", ESP_LOG_NONE);

  loadSolar30dHistory();

  server.on("/solar", handleSolarPage);
  server.on("/solar-history", handleSolarHistory);
  server.on("/solar-yield", handleYieldPage);
  server.on("/solar-yield-history", handleSolar30dHistory);
  server.on("/shunt", handleShuntPage);
  server.on("/shunt-history", handleShuntHistory);
  server.on("/temps", handleTempsPage);
  server.on("/temps-history", handleTempsHistory);

  victron.setMinInterval(100);

  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  delay(200);
  Serial.println("\nVictron BLE Monitor starting...");

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");

  const uint32_t WIFI_CONNECT_TIMEOUT_MS = 60000;
  uint32_t wifiWaitStart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - wifiWaitStart > WIFI_CONNECT_TIMEOUT_MS) {
      Serial.println("\nWiFi didn't connect within 60s - restarting to retry.");
      delay(200);
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

  setenv("TZ", "PDT7", 1);
  tzset();

  #if !defined(CONFIG_IDF_TARGET_ESP32S3)
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  #endif

  victron.begin(5);
  victron.setCallback(onVictronData);
  victron.setMinInterval(1000);

  victron.addDevice("Port Solar", SOLAR_PORT_MAC, SOLAR_PORT_KEY, DEVICE_TYPE_SOLAR_CHARGER);
  victron.addDevice("Starboard Solar", SOLAR_STBD_MAC, SOLAR_STBD_KEY, DEVICE_TYPE_SOLAR_CHARGER);
  victron.addDevice("Battery Shunt", SHUNT_MAC, SHUNT_KEY, DEVICE_TYPE_BATTERY_MONITOR);

  Serial.printf("Connecting to GPS TCP endpoint %s:%d...\n", GPS_HOST, GPS_PORT);
  gpsClient.connect(GPS_HOST, GPS_PORT);

  Serial.printf("Connecting to AIS TCP endpoint %s:%d...\n", AIS_HOST, AIS_PORT);
  aisClient.connect(AIS_HOST, AIS_PORT);

  telemetryUdp.begin(TELEMETRY_UDP_PORT);
  Serial.print("Listening for heater/icebox telemetry on UDP:");
  Serial.println(TELEMETRY_UDP_PORT);

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  Serial.println("Web server started.");
  Serial.println("Dashboard: http://192.168.8.66/");

  esp_task_wdt_deinit();
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = 20000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdtConfig);
  esp_task_wdt_add(NULL);
  Serial.println("Task watchdog armed (20s).");

  tempSensors.begin();
  tempSensors.setWaitForConversion(false);

  for (int i = 0; i < TEMP_PROBE_COUNT; i++) {
    tempProbeAddr[i][0] = 0x28;
    for (int b = 0; b < 6; b++) tempProbeAddr[i][1 + b] = TEMP_PROBE_SERIAL[i][b];
    tempProbeAddr[i][7] = OneWire::crc8(tempProbeAddr[i], 7);
  }

  int probesFound = tempSensors.getDeviceCount();
  Serial.printf("Temp probes found: %d (expected %d)\n", probesFound, TEMP_PROBE_COUNT);
  for (int i = 0; i < TEMP_PROBE_COUNT; i++) {
    bool present = tempSensors.isConnected(tempProbeAddr[i]);
    Serial.printf("  %-8s ", TEMP_PROBE_NAMES[i]);
    for (uint8_t b = 0; b < 8; b++) Serial.printf("%02X", tempProbeAddr[i][b]);
    Serial.println(present ? " - found" : " - NOT FOUND (check wiring)");
  }
}

void loop() {
  esp_task_wdt_reset();
  maintainWiFi();

  pollGpsTcp();
  pollAisTcp();

  victron.loop();
  pollTelemetry();
  pollTemps();
  pollSolarHistory();
  pollSolar30dHistory();
  pollShuntHistory();
  pollTempHistory();
  server.handleClient();
}