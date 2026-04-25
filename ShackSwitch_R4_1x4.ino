/*
 * ShackSwitch_R4_1x4.ino  —  G0JKN ShackSwitch  —  1×4 single-radio antenna switch
 * Arduino Uno R4 WiFi
 *
 * Nigel Fenton  G0JKN  —  https://github.com/nigelfenton/shackswitch
 * MIT Licence
 *
 * Hardware
 *   Relays : D2–D5  (one active at a time, all others deactivated)
 *   Display: built-in 12×8 LED matrix  (shows active port digit 1–4)
 *
 * Network
 *   HTTP  port 8080   — web UI + REST API
 *   TCP   port 9007 — 4O3A Antenna Genius protocol (AetherSDR)
 *   UDP   port 9007 — discovery beacon every 5 s
 *
 * REST endpoints
 *   GET /                                  — status page
 *   GET /settings                          — settings page
 *   GET /status                            — JSON status
 *   GET /select?port=N                     — select antenna 1–4  (0 = all off)
 *   GET /setband?band=X [&input=1]         — auto-select from band map
 *   GET /kk1l/setband?input=1&band=X       — SmartSDR-compatible alias
 *   GET /names?n1=X&n2=X&n3=X&n4=X        — save all antenna names
 *   GET /bandmap?band=X&port=N             — assign band to port  (port=0 clears)
 *   GET /wifi?ssid=X&pass=Y               — save WiFi credentials + reboot
 *   GET /reset                             — factory reset
 *
 * ── EDIT BEFORE FIRST FLASH ───────────────────────────────────────────────────
 * Change DEFAULT_SSID and DEFAULT_PASSWORD to match your network.
 * These are written to EEPROM on a clean first boot only.
 * You can also update them later via /settings in the web UI.
 */

#include <WiFiS3.h>
#include <EEPROM.h>
#include "Arduino_LED_Matrix.h"

// ── Edit before first flash ────────────────────────────────────────────────────
#define DEFAULT_SSID     "tinkerbell"
#define DEFAULT_PASSWORD "disneybell"

// ── Relay pins ─────────────────────────────────────────────────────────────────
const int RELAY_PINS[4] = {2, 3, 4, 5};   // D2 = port 1 ... D5 = port 4
#define NUM_PORTS 4

// ── Network ────────────────────────────────────────────────────────────────────
#define HTTP_PORT  8080
#define AG_PORT    9007    // 4O3A Antenna Genius protocol — TCP server + UDP beacon

// ── EEPROM layout ──────────────────────────────────────────────────────────────
#define MAGIC_VALUE   0xC00A1C05u   // bump to reset stored config
#define E_MAGIC        0            // 4 bytes
#define E_SSID         4            // 64 bytes
#define E_PASS         68           // 64 bytes
#define E_NAMES        132          // 4 × 20 bytes = 80
#define E_BANDMAP      212          // NUM_BANDS bytes = 11
#define E_FLEX_IP      223          // 16 bytes — selected radio IP
#define E_FLEX_NAME    239          // 20 bytes — selected radio display name
#define E_ACTIVE_PORT  259          //  1 byte  — last selected port (0 = all off)
// total used: 260 bytes

// ── Band table ─────────────────────────────────────────────────────────────────
struct Band { const char* name; long lo; long hi; };  // kHz
const Band BANDS[] = {
  {"160m",  1800,  2000},
  {"80m",   3500,  4000},
  {"60m",   5250,  5450},
  {"40m",   7000,  7300},
  {"30m",  10100, 10150},
  {"20m",  14000, 14350},
  {"17m",  18068, 18168},
  {"15m",  21000, 21450},
  {"12m",  24890, 24990},
  {"10m",  28000, 29700},
  {"6m",   50000, 54000},
};
#define NUM_BANDS 11

// ── Global state ───────────────────────────────────────────────────────────────
char    g_ssid[64];
char    g_pass[64];
char    g_antName[4][20];
uint8_t g_bandMap[NUM_BANDS];  // port 1–4, 0 = unmapped
int     g_activePort = 0;      // 1–4 active, 0 = all off
char    g_band[8]    = "";     // e.g. "20m" or "" if unknown

ArduinoLEDMatrix matrix;
WiFiServer httpServer(HTTP_PORT);
WiFiServer agServer(AG_PORT);
WiFiUDP    agUdp;
WiFiClient agClient;
bool       agSubscribed = false;

unsigned long lastBeacon    = 0;
unsigned long lastWifiCheck = 0;

// ── Radio discovery ────────────────────────────────────────────────────────────
#define MAX_RADIOS 6
struct RadioEntry { char ip[16]; char name[20]; char serial[16]; unsigned long lastSeen; };
RadioEntry    g_radios[MAX_RADIOS];
int           g_radioCount  = 0;
char          g_flexIP[16]  = "";
char          g_flexName[20]= "";

WiFiClient    flexClient;
WiFiUDP       flexUdp;
String        flexLineBuf;
int           flexSeq       = 1;
unsigned long lastFlexCheck = 0;

// ── Forward declarations ───────────────────────────────────────────────────────
void agPushStatus();
void selectPort(int port);
void nxtSetPort(int port);
void nxtSetBand(const char* band);

// ── EEPROM helpers ─────────────────────────────────────────────────────────────
void eepromReadStr(int addr, char* buf, int maxLen) {
  for (int i = 0; i < maxLen; i++) buf[i] = EEPROM.read(addr + i);
  buf[maxLen - 1] = '\0';
}

void eepromWriteStr(int addr, const char* s, int maxLen) {
  int len = strlen(s);
  for (int i = 0; i < maxLen; i++)
    EEPROM.write(addr + i, i < len ? (uint8_t)s[i] : 0);
}

uint32_t eepromReadU32(int addr) {
  uint32_t v = 0;
  for (int i = 0; i < 4; i++) v |= (uint32_t)EEPROM.read(addr + i) << (i * 8);
  return v;
}

void eepromWriteU32(int addr, uint32_t v) {
  for (int i = 0; i < 4; i++) EEPROM.write(addr + i, (v >> (i * 8)) & 0xFF);
}

// ── Config ─────────────────────────────────────────────────────────────────────
void applyDefaults() {
  strncpy(g_ssid, DEFAULT_SSID, sizeof(g_ssid) - 1);
  strncpy(g_pass, DEFAULT_PASSWORD, sizeof(g_pass) - 1);
  for (int i = 0; i < NUM_PORTS; i++) snprintf(g_antName[i], 20, "Antenna %d", i + 1);
  memset(g_bandMap, 0, sizeof(g_bandMap));
}

void saveFlexConfig() {
  eepromWriteStr(E_FLEX_IP,   g_flexIP,   16);
  eepromWriteStr(E_FLEX_NAME, g_flexName, 20);
}

void saveActivePort() {
  EEPROM.write(E_ACTIVE_PORT, (uint8_t)g_activePort);
}

void saveConfig() {
  eepromWriteU32(E_MAGIC, MAGIC_VALUE);
  eepromWriteStr(E_SSID, g_ssid, 64);
  eepromWriteStr(E_PASS, g_pass, 64);
  for (int i = 0; i < NUM_PORTS; i++)
    eepromWriteStr(E_NAMES + i * 20, g_antName[i], 20);
  for (int i = 0; i < NUM_BANDS; i++)
    EEPROM.write(E_BANDMAP + i, g_bandMap[i]);
  saveFlexConfig();
}

void loadConfig() {
  if (eepromReadU32(E_MAGIC) != MAGIC_VALUE) {
    Serial.println(F("EEPROM: no valid config — writing defaults"));
    applyDefaults();
    saveConfig();
    return;
  }
  eepromReadStr(E_SSID, g_ssid, 64);
  eepromReadStr(E_PASS, g_pass, 64);
  for (int i = 0; i < NUM_PORTS; i++)
    eepromReadStr(E_NAMES + i * 20, g_antName[i], 20);
  for (int i = 0; i < NUM_BANDS; i++) {
    g_bandMap[i] = EEPROM.read(E_BANDMAP + i);
    if (g_bandMap[i] > NUM_PORTS) g_bandMap[i] = 0;
  }
  // Flex radio — appended fields, safe without magic bump
  eepromReadStr(E_FLEX_IP,   g_flexIP,   sizeof(g_flexIP));
  eepromReadStr(E_FLEX_NAME, g_flexName, sizeof(g_flexName));
  if ((uint8_t)g_flexIP[0]   >= 0xFE) g_flexIP[0]   = '\0';
  if ((uint8_t)g_flexName[0] >= 0xFE) g_flexName[0] = '\0';
  // Active port — appended field, safe without magic bump
  uint8_t ap = EEPROM.read(E_ACTIVE_PORT);
  g_activePort = (ap >= 1 && ap <= NUM_PORTS) ? (int)ap : 0;
}

// ── LED matrix ─────────────────────────────────────────────────────────────────
// 3-wide × 7-tall digit glyphs, centred in the 12×8 matrix at col 4, rows 0–6
const byte DIGIT_GLYPH[5][7][3] = {
  // 0 — all off (no port selected)
  {{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}},
  // 1
  {{0,1,0},{1,1,0},{0,1,0},{0,1,0},{0,1,0},{0,1,0},{1,1,1}},
  // 2
  {{1,1,1},{0,0,1},{0,0,1},{1,1,1},{1,0,0},{1,0,0},{1,1,1}},
  // 3
  {{1,1,1},{0,0,1},{0,0,1},{1,1,1},{0,0,1},{0,0,1},{1,1,1}},
  // 4
  {{1,0,1},{1,0,1},{1,1,1},{0,0,1},{0,0,1},{0,0,1},{0,0,1}},
};

void updateMatrix(int port) {
  byte frame[8][12] = {};
  if (port == 0) {
    // Earth/ground symbol — 5-wide bars centred at col 5
    frame[0][5] = 1; frame[1][5] = 1;             // stem
    for (int c = 3; c <= 7; c++) frame[2][c] = 1; // bar 1 (5 wide)
    for (int c = 4; c <= 6; c++) frame[3][c] = 1; // bar 2 (3 wide)
    frame[4][5] = 1;                               // bar 3 (1 wide)
  } else {
    for (int r = 0; r < 7; r++)
      for (int c = 0; c < 3; c++)
        frame[r][c + 4] = DIGIT_GLYPH[port][r][c];
  }
  matrix.renderBitmap(frame, 8, 12);
}

// ── Relay control ──────────────────────────────────────────────────────────────
void selectPort(int port) {
  if (port < 0 || port > NUM_PORTS) return;
  for (int i = 0; i < NUM_PORTS; i++)
    digitalWrite(RELAY_PINS[i], (port == i + 1) ? HIGH : LOW);
  g_activePort = port;
  saveActivePort();
  updateMatrix(port);
  nxtSetPort(port);
  agPushStatus();
  Serial.print(F("Port: "));
  Serial.println(port);
}

// ── Band helpers ───────────────────────────────────────────────────────────────
int bandIndex(const char* name) {
  for (int i = 0; i < NUM_BANDS; i++)
    if (strcmp(BANDS[i].name, name) == 0) return i;
  return -1;
}

int bandFromFreqHz(long hz) {
  long khz = hz / 1000;
  for (int i = 0; i < NUM_BANDS; i++)
    if (khz >= BANDS[i].lo && khz <= BANDS[i].hi) return i;
  return -1;
}

void setBand(const char* bandName) {
  int idx = bandIndex(bandName);
  if (idx < 0) return;
  strncpy(g_band, bandName, sizeof(g_band) - 1);
  nxtSetBand(g_band);
  Serial.print(F("Band: ")); Serial.println(g_band);
  int port = g_bandMap[idx];
  if (port >= 1 && port <= NUM_PORTS) selectPort(port);
  else agPushStatus();  // push band change even if no port mapped
}

// ── URL helpers ────────────────────────────────────────────────────────────────
void urlDecode(const char* src, char* dst, int maxLen) {
  int di = 0;
  for (int si = 0; src[si] && di < maxLen - 1; si++) {
    if (src[si] == '%' && src[si+1] && src[si+2]) {
      char hex[3] = {src[si+1], src[si+2], 0};
      dst[di++] = (char)strtol(hex, nullptr, 16);
      si += 2;
    } else if (src[si] == '+') {
      dst[di++] = ' ';
    } else {
      dst[di++] = src[si];
    }
  }
  dst[di] = '\0';
}

bool getParam(const char* query, const char* key, char* buf, int maxLen) {
  char searchKey[32];
  snprintf(searchKey, sizeof(searchKey), "%s=", key);
  const char* p = strstr(query, searchKey);
  if (!p) return false;
  p += strlen(searchKey);
  const char* end = strchr(p, '&');
  int len = end ? (int)(end - p) : (int)strlen(p);
  if (len >= maxLen) len = maxLen - 1;
  char raw[128];
  if (len >= (int)sizeof(raw)) len = sizeof(raw) - 1;
  strncpy(raw, p, len);
  raw[len] = '\0';
  urlDecode(raw, buf, maxLen);
  return true;
}

// ── WiFi ───────────────────────────────────────────────────────────────────────
void connectWifi() {
  if (strlen(g_ssid) == 0) { Serial.println(F("No SSID — skipping WiFi")); return; }
  Serial.print(F("WiFi: connecting to ")); Serial.println(g_ssid);
  WiFi.begin(g_ssid, g_pass);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) delay(300);
  if (WiFi.status() == WL_CONNECTED) {
    // Wait for DHCP — R4 sets WL_CONNECTED before IP is assigned
    unsigned long t2 = millis();
    while (WiFi.localIP() == IPAddress(0, 0, 0, 0) && millis() - t2 < 5000) delay(200);
    Serial.print(F("WiFi: connected  IP: ")); Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("WiFi: connect failed"));
  }
}

// ── AetherSDR AG protocol ──────────────────────────────────────────────────────
uint16_t antennaBandMask(int portNum) {
  uint16_t mask = 0;
  for (int i = 0; i < NUM_BANDS; i++)
    if (g_bandMap[i] == portNum) mask |= (uint16_t)(1 << i);
  return mask ? mask : 0x07FF;  // 0x07FF = all 11 bands if unmapped
}

// Returns the AG protocol band ID (1-based) for the current band, or 0 if none.
int agBandId() {
  if (!strlen(g_band)) return 0;
  int idx = bandIndex(g_band);
  return (idx >= 0) ? idx + 1 : 0;
}

void agPushStatus() {
  if (!agSubscribed || !agClient.connected()) return;
  char buf[80];
  snprintf(buf, sizeof(buf),
    "S0|port 1 auto=1 source=AUTO band=%d rxant=%d txant=%d tx=0 inhibit=0\r\n",
    agBandId(), g_activePort, g_activePort);
  agClient.print(buf);
}

void handleAgCommand(const String& raw) {
  int pipe = raw.indexOf('|');
  if (pipe < 1) return;
  int seq = raw.substring(1, pipe).toInt();
  String cmd = raw.substring(pipe + 1);
  cmd.trim();

  char r[160];

  if (cmd == "ping") {
    snprintf(r, sizeof(r), "R%d|00|pong\r\n", seq);
    agClient.print(r);

  } else if (cmd == "antenna list") {
    for (int i = 0; i < NUM_PORTS; i++) {
      uint16_t mask = antennaBandMask(i + 1);
      // Replace spaces with underscores — AetherSDR reverses this on display
      char safeName[21];
      strncpy(safeName, g_antName[i], sizeof(safeName) - 1);
      safeName[20] = '\0';
      for (int j = 0; safeName[j]; j++) if (safeName[j] == ' ') safeName[j] = '_';
      snprintf(r, sizeof(r), "R%d|00|antenna %d name=%s tx=%04X rx=%04X inband=0000\r\n",
               seq, i + 1, safeName, mask, mask);
      agClient.print(r);
    }
    snprintf(r, sizeof(r), "R%d|00|\r\n", seq);
    agClient.print(r);

  } else if (cmd == "band list") {
    // Band 0 = no band (required by AetherSDR)
    snprintf(r, sizeof(r), "R%d|00|band 0 name=None freq_start=0.000 freq_stop=0.000\r\n", seq);
    agClient.print(r);
    // Bands 1..NUM_BANDS map to BANDS[0..NUM_BANDS-1]; frequencies in MHz
    for (int i = 0; i < NUM_BANDS; i++) {
      snprintf(r, sizeof(r), "R%d|00|band %d name=%s freq_start=%.3f freq_stop=%.3f\r\n",
               seq, i + 1, BANDS[i].name,
               BANDS[i].lo / 1000.0f, BANDS[i].hi / 1000.0f);
      agClient.print(r);
    }
    snprintf(r, sizeof(r), "R%d|00|\r\n", seq);
    agClient.print(r);

  } else if (cmd.startsWith("port get")) {
    // Extract requested port number; R4 only has port 1
    int portNum = cmd.substring(9).toInt();
    if (portNum == 1) {
      snprintf(r, sizeof(r),
        "R%d|00|port 1 auto=1 source=AUTO band=%d rxant=%d txant=%d tx=0 inhibit=0\r\n",
        seq, agBandId(), g_activePort, g_activePort);
      agClient.print(r);
    }
    snprintf(r, sizeof(r), "R%d|00|\r\n", seq);
    agClient.print(r);

  } else if (cmd.startsWith("sub ") || cmd.startsWith("subscribe")) {
    agSubscribed = true;
    snprintf(r, sizeof(r), "R%d|00|ok\r\n", seq);
    agClient.print(r);
    agPushStatus();

  } else if (cmd.startsWith("port set")) {
    // e.g. "port set 1 rxant=2 txant=2"
    int pos = cmd.indexOf("rxant=");
    if (pos >= 0) {
      int ant = cmd.substring(pos + 6).toInt();
      if (ant >= 0 && ant <= NUM_PORTS) selectPort(ant);
    }
    snprintf(r, sizeof(r), "R%d|00|ok\r\n", seq);
    agClient.print(r);

  } else {
    snprintf(r, sizeof(r), "R%d|FF|unknown\r\n", seq);
    agClient.print(r);
  }
}

void agLoop() {
  // Accept new connection when idle
  if (!agClient.connected()) {
    agSubscribed = false;
    WiFiClient nc = agServer.available();
    if (nc) {
      agClient = nc;
      agClient.print(F("V1.0 AG\r\n"));
      Serial.println(F("AG: client connected"));
    }
  }

  // Read and dispatch incoming command lines
  if (agClient.connected() && agClient.available()) {
    String line = agClient.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      Serial.print(F("AG< ")); Serial.println(line);
      handleAgCommand(line);
    }
  }

  // UDP discovery beacon every 5 s
  if (millis() - lastBeacon >= 5000 && WiFi.status() == WL_CONNECTED) {
    lastBeacon = millis();
    IPAddress bcast = WiFi.localIP();
    bcast[3] = 255;  // assumes /24 subnet — works for 10.0.0.x
    char beacon[128];
    snprintf(beacon, sizeof(beacon),
      "AG ip=%s port=%d v=1.0 serial=G0JKN-SS-R4 name=ShackSwitch ports=1 antennas=%d webport=8080\r\n",
      WiFi.localIP().toString().c_str(), AG_PORT, NUM_PORTS);
    agUdp.beginPacket(bcast, AG_PORT);
    agUdp.print(beacon);
    agUdp.endPacket();
  }
}

// ── FlexRadio discovery + SmartSDR band tracking ──────────────────────────────

// Extract a space/newline-terminated value from a null-terminated key=value string
bool extractField(const char* buf, const char* key, char* out, int maxLen) {
  const char* p = strstr(buf, key);
  if (!p) return false;
  p += strlen(key);
  const char* end = strpbrk(p, " \n\r");
  int len = end ? (int)(end - p) : (int)strlen(p);
  if (len <= 0 || len >= maxLen) { out[0] = '\0'; return false; }
  strncpy(out, p, len);
  out[len] = '\0';
  return true;
}

// Binary-safe search — finds needle in buf even when buf contains null bytes
int memfind(const char* buf, int bufLen, const char* needle, int needleLen) {
  for (int i = 0; i <= bufLen - needleLen; i++)
    if (memcmp(buf + i, needle, needleLen) == 0) return i;
  return -1;
}

// Handle one UDP discovery beacon from a FlexRadio (VITA-49 binary framed)
void parseDiscovery(char* buf, int len) {
  // FlexRadio beacons have a ~28-byte binary VITA-49 header then ASCII fields.
  // Use binary-safe search so null bytes in the header don't fool strstr.
  int start = memfind(buf, len, "model=FLEX", 10);
  if (start < 0) return;

  // From the ASCII payload onward, buf is null-terminated (pkt[n]='\0' in flexLoop)
  const char* ascii = buf + start;

  // Use the UDP source IP — more reliable than parsing it from the packet
  String ipStr = flexUdp.remoteIP().toString();
  char ip[16];
  ipStr.toCharArray(ip, sizeof(ip));

  char name[20]="", serial[16]="";
  if (!extractField(ascii, "nickname=", name, sizeof(name)) || name[0]=='\0')
    extractField(ascii, "model=", name, sizeof(name));
  extractField(ascii, "serial=", serial, sizeof(serial));

  // Update existing entry (matched by serial, then IP)
  for (int i = 0; i < g_radioCount; i++) {
    bool match = (serial[0] && strcmp(g_radios[i].serial, serial) == 0)
              || strcmp(g_radios[i].ip, ip) == 0;
    if (match) {
      if (strcmp(g_radios[i].ip, ip) != 0) {
        // IP changed (DHCP) — update and force reconnect if this is active radio
        if (strcmp(g_flexIP, g_radios[i].ip) == 0) {
          strncpy(g_flexIP, ip, sizeof(g_flexIP) - 1);
          flexClient.stop();
        }
        strncpy(g_radios[i].ip, ip, sizeof(g_radios[i].ip) - 1);
      }
      strncpy(g_radios[i].name, name, sizeof(g_radios[i].name) - 1);
      g_radios[i].lastSeen = millis();
      return;
    }
  }

  // New radio
  if (g_radioCount >= MAX_RADIOS) return;
  strncpy(g_radios[g_radioCount].ip,     ip,     sizeof(g_radios[0].ip)     - 1);
  strncpy(g_radios[g_radioCount].name,   name,   sizeof(g_radios[0].name)   - 1);
  strncpy(g_radios[g_radioCount].serial, serial, sizeof(g_radios[0].serial) - 1);
  g_radios[g_radioCount].lastSeen = millis();
  Serial.print(F("Radio: ")); Serial.print(name);
  Serial.print(F(" @ ")); Serial.println(ip);

  // Auto-select first discovered radio if none configured
  if (g_flexIP[0] == '\0') {
    strncpy(g_flexIP,   ip,   sizeof(g_flexIP)   - 1);
    strncpy(g_flexName, name, sizeof(g_flexName) - 1);
    saveFlexConfig();
    Serial.println(F("Auto-selected"));
  }
  g_radioCount++;
}

// Parse a SmartSDR TCP status line for RF_frequency (SmartSDR v3 uses capital RF)
void parseFlexLine(const String& line) {
  int pos = line.indexOf("RF_frequency=");
  if (pos < 0) pos = line.indexOf("rf_frequency=");
  if (pos < 0) return;
  int eq = line.indexOf('=', pos);
  float mhz = line.substring(eq + 1).toFloat();
  if (mhz < 1.0) return;
  long hz  = (long)(mhz * 1000000.0 + 0.5);
  int  idx = bandFromFreqHz(hz);
  if (idx >= 0 && strcmp(g_band, BANDS[idx].name) != 0)
    setBand(BANDS[idx].name);
}

void flexLoop() {
  // ── Passive UDP discovery (port 4992) ─────────────────────────────────────
  int pktLen = flexUdp.parsePacket();
  if (pktLen > 0) {
    char pkt[512];
    int  n = flexUdp.read(pkt, min(pktLen, 511));
    pkt[n] = '\0';
    parseDiscovery(pkt, n);
  }

  // ── TCP band-tracking connection ───────────────────────────────────────────
  if (g_flexIP[0] == '\0') return;

  if (!flexClient.connected()) {
    if (millis() - lastFlexCheck >= 10000) {
      lastFlexCheck = millis();
      flexLineBuf = "";
      flexSeq = 1;
      Serial.print(F("Flex: connecting to ")); Serial.println(g_flexIP);
      if (flexClient.connect(g_flexIP, 4992)) {
        delay(300);
        char cmd[24];
        snprintf(cmd, sizeof(cmd), "C%d|sub slice all\n", flexSeq++);
        flexClient.print(cmd);
        Serial.print(F("Flex: sent [")); Serial.print(cmd); Serial.println(']');
      } else {
        Serial.println(F("Flex: connect failed"));
      }
    }
    return;
  }

  // ── Read SmartSDR data ─────────────────────────────────────────────────────
  while (flexClient.available()) {
    char ch = flexClient.read();
    if (ch == '\n') {
      if (flexLineBuf.length() > 0) {
        parseFlexLine(flexLineBuf);
        flexLineBuf = "";
      }
    } else if (ch != '\r') {
      if (flexLineBuf.length() < 511) flexLineBuf += ch;
    }
  }
}

// ── Nextion display driver (Serial1 = D0/D1, page 0 = splash, page 1 = main) ──
// Touch events use printh 23 02 54 NN in HMI Touch Release — NOT standard 0x65 events.
// NN = port number 0x01–0x08.  ON pic = 5, OFF pic = 14 (HMI image library IDs).

#define NXT_BAUD    9600
#define NXT_PIC_ON  5
#define NXT_PIC_OFF 14

void nxtSend(const char* cmd) {
  Serial1.print(cmd);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
}

void nxtSetBand(const char* band) {
  // tBand not on new page 1 — kept for future use, silently ignored if absent
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "tBand.txt=\"%s\"", strlen(band) ? band : "--");
  nxtSend(cmd);
}

void nxtSetPort(int port) {
  // Set b0–b(NUM_PORTS-1) .pic and .pic2 to ON or OFF image based on active port.
  // Both attributes written so button image is consistent in released and pressed states.
  for (int i = 1; i <= NUM_PORTS; i++) {
    int pic = (port == i) ? NXT_PIC_ON : NXT_PIC_OFF;
    char cmd[24];
    snprintf(cmd, sizeof(cmd), "b%d.pic=%d", i - 1, pic);
    nxtSend(cmd);
    snprintf(cmd, sizeof(cmd), "b%d.pic2=%d", i - 1, pic);
    nxtSend(cmd);
  }
}

void nxtInit() {
  // Serial1 already started in setup() before connectWifi() so splash shows during boot.
  // Here we switch from splash to page 1 and push initial state.
  nxtSend("page 1");
  delay(500);
  nxtSend("page 1");   // second send — guards against Nextion missing it on warm reboot
  delay(100);
  nxtSend("t0.txt=\"G0JKN ShackSwitch v1.5\"");
  nxtSend("t1.txt=\"1 x 4\"");
  // Push antenna names into dual-state button bt attribute
  for (int i = 0; i < NUM_PORTS; i++) {
    char cmd[40];
    snprintf(cmd, sizeof(cmd), "b%d.bt=\"%s\"", i, g_antName[i]);
    nxtSend(cmd);
  }
  // Hide unused button slots (HMI has 8 slots; R4 uses NUM_PORTS of them)
  for (int i = NUM_PORTS; i < 8; i++) {
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "vis b%d,0", i);
    nxtSend(cmd);
  }
  // Restore relay to last saved port (0 = all off is safe default)
  selectPort(g_activePort);
  Serial.print(F("Nextion init — restoring port "));
  Serial.println(g_activePort);
}

void nxtLoop() {
  // Parse 4-byte printh sequences: 0x23 0x02 0x54 NN
  // Sent by HMI button Touch Release events. NN = port 0x01–0x08.
  while (Serial1.available() >= 4) {
    if (Serial1.peek() != 0x23) { Serial1.read(); continue; }
    uint8_t buf[4];
    Serial1.readBytes(buf, 4);
    if (buf[1] != 0x02 || buf[2] != 0x54) continue;
    int port = buf[3];
    if (port >= 1 && port <= NUM_PORTS)
      selectPort(g_activePort == port ? 0 : port);  // tap active port = deselect
  }
}

// ── HTTP helpers ───────────────────────────────────────────────────────────────
void httpRedirect(WiFiClient& c, const char* to) {
  c.print(F("HTTP/1.1 302 Found\r\nLocation: "));
  c.print(to);
  c.print(F("\r\nConnection: close\r\n\r\n"));
}

// ── HTTP: /status JSON ─────────────────────────────────────────────────────────
void sendStatusJson(WiFiClient& c) {
  String body = "{\"port\":";
  body += g_activePort;
  body += ",\"band\":\"";
  body += g_band;
  body += "\",\"ip\":\"";
  body += WiFi.localIP().toString();
  body += "\",\"antennas\":[";
  for (int i = 0; i < NUM_PORTS; i++) {
    if (i) body += ",";
    body += "{\"id\":";
    body += (i + 1);
    body += ",\"name\":\"";
    body += g_antName[i];
    body += "\"}";
  }
  body += "]}";
  c.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\nContent-Length: "));
  c.print(body.length());
  c.print(F("\r\n\r\n"));
  c.print(body);
}

// ── HTTP: main status page ─────────────────────────────────────────────────────
void sendMainPage(WiFiClient& c) {
  c.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"));
  c.print(F("<!DOCTYPE html><html><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>G0JKN ShackSwitch</title><style>"
    "*{box-sizing:border-box;margin:0;padding:0}"
    "body{background:#09101f;color:#dde6f0;font-family:'Segoe UI',sans-serif;padding:16px}"
    "h1{color:#00d8ef;font-size:18px;margin-bottom:4px}"
    ".sub{color:#6b8099;font-size:12px;margin-bottom:16px}"
    ".nav{display:flex;gap:16px;margin-bottom:16px}"
    ".nav a{color:#6b8099;text-decoration:none;font-size:14px;padding-bottom:4px}"
    ".nav a.on{color:#00d8ef;border-bottom:2px solid #00d8ef}"
    ".card{background:#0d1628;border:1px solid #1c2a40;border-radius:8px;padding:16px;margin-bottom:12px}"
    ".lbl{font-size:11px;color:#6b8099;text-transform:uppercase;letter-spacing:.5px;margin-bottom:8px}"
    ".band{font-size:32px;font-weight:bold;color:#00d8ef}"
    ".row{display:flex;flex-wrap:wrap;gap:8px;margin-top:4px}"
    ".btn{flex:1;min-width:80px;padding:14px 8px;font-size:14px;font-weight:bold;"
    "background:#0f1e30;color:#dde6f0;border:1px solid #1c2a40;border-radius:6px;"
    "cursor:pointer;text-align:center;text-decoration:none}"
    ".btn:hover{border-color:#00d8ef}"
    ".btn.on{background:#0a2a2e;border-color:#00d8ef;color:#00d8ef}"
    ".pnum{font-size:11px;color:#6b8099;margin-top:4px}"
    "</style></head><body>"));
  c.print(F("<h1>G0JKN ShackSwitch &mdash; 1&times;4</h1>"));
  c.print(F("<div class='sub'>"));
  c.print(WiFi.localIP().toString());
  c.print(F("</div>"));
  c.print(F("<div class='nav'><a href='/' class='on'>Status</a>"
            "<a href='/settings'>Settings</a></div>"));

  // Band card
  c.print(F("<div class='card'><div class='lbl'>Active Band</div>"
            "<div class='band' id='bnd'>"));
  c.print(strlen(g_band) ? g_band : "&mdash;");
  c.print(F("</div></div>"));

  // Antenna buttons — clicking the active button deselects (port=0)
  c.print(F("<div class='card'><div class='lbl'>Select Antenna</div><div class='row'>"));
  for (int i = 0; i < NUM_PORTS; i++) {
    bool active = (g_activePort == i + 1);
    c.print(F("<a id='p"));
    c.print(i + 1);
    c.print(F("' href='/select?port="));
    c.print(active ? 0 : i + 1);   // toggle: click active = deselect
    c.print(active ? F("' class='btn on'>") : F("' class='btn'>"));
    c.print(g_antName[i]);
    c.print(F("<div class='pnum'>PORT "));
    c.print(i + 1);
    c.print(F("</div></a>"));
  }
  c.print(F("</div></div>"));

  // Smart poll — only update DOM when port or band actually changes
  c.print(F("<script>"
    "var lp="));
  c.print(g_activePort);
  c.print(F(",lb='"));
  c.print(g_band);
  c.print(F("';"
    "function poll(){"
    "fetch('/status').then(r=>r.json()).then(d=>{"
    "if(d.port!==lp||d.band!==lb){"
    "lp=d.port;lb=d.band;"
    "var bd=d.band||'\u2014';"
    "document.getElementById('bnd').textContent=bd;"
    "for(var i=1;i<=4;i++){"
    "var b=document.getElementById('p'+i);"
    "if(b){b.className='btn'+(d.port===i?' on':'');"
    "b.href='/select?port='+(d.port===i?0:i);}}"
    "}}).catch(()=>{});"
    "setTimeout(poll,3000);}"
    "poll();"
    "</script></body></html>"));
}

// ── HTTP: settings page ────────────────────────────────────────────────────────
void sendSettingsPage(WiFiClient& c) {
  c.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"));
  c.print(F("<!DOCTYPE html><html><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>ShackSwitch Settings</title><style>"
    "*{box-sizing:border-box;margin:0;padding:0}"
    "body{background:#09101f;color:#dde6f0;font-family:'Segoe UI',sans-serif;padding:16px}"
    "h1{color:#00d8ef;font-size:18px;margin-bottom:4px}"
    ".sub{color:#6b8099;font-size:12px;margin-bottom:12px}"
    ".nav{display:flex;gap:16px;margin-bottom:12px}"
    ".nav a{color:#6b8099;text-decoration:none;font-size:14px;padding-bottom:4px}"
    ".nav a.on{color:#00d8ef;border-bottom:2px solid #00d8ef}"
    ".tabs{display:flex;border-bottom:2px solid #1c2a40;margin-bottom:16px}"
    ".tb{padding:8px 16px;color:#6b8099;text-decoration:none;font-size:14px;"
    "border-bottom:3px solid transparent;margin-bottom:-2px}"
    ".tb.on{color:#00d8ef;border-bottom-color:#00d8ef}"
    ".tb:hover{color:#dde6f0}"
    ".tp{display:none}"
    ".card{background:#0d1628;border:1px solid #1c2a40;border-radius:8px;padding:16px;margin-bottom:12px}"
    "h2{color:#6b8099;font-size:11px;text-transform:uppercase;letter-spacing:1px;"
    "border-bottom:1px solid #1c2a40;padding-bottom:6px;margin-bottom:12px}"
    "label{display:block;font-size:12px;color:#6b8099;margin:10px 0 4px}"
    "input[type=text],input[type=password]{width:100%;padding:8px;background:#0f1e30;"
    "color:#dde6f0;border:1px solid #1c2a40;border-radius:4px;font-size:14px}"
    "input:focus{outline:none;border-color:#00d8ef}"
    ".save{display:inline-block;margin-top:12px;padding:8px 20px;"
    "background:#00d8ef;color:#000;font-weight:bold;border:none;border-radius:4px;"
    "cursor:pointer;font-size:14px;text-decoration:none}"
    ".danger{display:inline-block;margin-top:12px;padding:8px 20px;"
    "background:#7f1d1d;color:#fca5a5;border:1px solid #ef4444;border-radius:4px;"
    "font-weight:bold;cursor:pointer;font-size:14px;text-decoration:none}"
    "table{width:100%;border-collapse:collapse;font-size:13px}"
    "th{color:#6b8099;padding:4px 8px;text-align:left;border-bottom:1px solid #1c2a40}"
    "td{padding:6px 8px}"
    "select{background:#0f1e30;color:#dde6f0;border:1px solid #1c2a40;"
    "border-radius:4px;padding:4px 8px;font-size:13px}"
    ".mwrap{overflow-x:auto}"
    ".mtbl{border-collapse:separate;border-spacing:3px}"
    ".mbh{color:#6b8099;font-size:10px;text-align:center;padding:3px 6px;"
    "background:#0b1220;border-radius:3px;min-width:50px;white-space:nowrap}"
    ".mband{color:#00d8ef;font-size:12px;font-weight:700;"
    "padding:3px 10px 3px 0;text-align:right;white-space:nowrap}"
    ".mdot{display:block;width:50px;height:30px;text-align:center;line-height:30px;"
    "border-radius:4px;font-size:18px;color:rgba(255,255,255,.18);"
    "background:rgba(255,255,255,.04);text-decoration:none}"
    ".mdot:hover{background:rgba(0,216,239,.2)}"
    ".mdot.on{background:#00d8ef;color:#0a0e14;font-size:14px}"
    "</style></head><body>"));

  c.print(F("<h1>G0JKN ShackSwitch &mdash; Settings</h1>"));
  c.print(F("<div class='sub'>"));
  c.print(WiFi.localIP().toString());
  c.print(F("</div>"));
  c.print(F("<div class='nav'><a href='/'>Status</a>"
            "<a href='/settings' class='on'>Settings</a></div>"));

  // Tab bar
  c.print(F("<div class='tabs'>"
    "<a href='#antennas' class='tb'>Antennas</a>"
    "<a href='#bandmap' class='tb'>Antenna Map</a>"
    "<a href='#radios' class='tb'>Radios</a>"
    "<a href='#wifi' class='tb'>WiFi</a>"
    "</div>"));

  // ── Tab: Antennas ──────────────────────────────────────────────────────────
  c.print(F("<div id='antennas' class='tp'><div class='card'><h2>Antenna Names</h2>"
    "<form action='/names' method='get'>"));
  for (int i = 0; i < NUM_PORTS; i++) {
    c.print(F("<label>Port "));
    c.print(i + 1);
    c.print(F("</label><input type='text' name='n"));
    c.print(i + 1);
    c.print(F("' value='"));
    c.print(g_antName[i]);
    c.print(F("'>"));
  }
  c.print(F("<input type='submit' class='save' value='Save'></form></div></div>"));

  // ── Tab: Antenna Map ───────────────────────────────────────────────────────
  c.print(F("<div id='bandmap' class='tp'><div class='card'><h2>Antenna Map</h2>"
    "<div class='mwrap'><table class='mtbl'><tr><th></th>"
    "<th class='mbh'>&mdash;</th>"));
  for (int p = 1; p <= NUM_PORTS; p++) {
    c.print(F("<th class='mbh'>"));
    c.print(p);
    c.print(F(" &middot; "));
    c.print(g_antName[p - 1]);
    c.print(F("</th>"));
  }
  c.print(F("</tr>"));
  for (int i = 0; i < NUM_BANDS; i++) {
    c.print(F("<tr><td class='mband'>"));
    c.print(BANDS[i].name);
    c.print(F("</td><td><a href='/bandmap?band="));
    c.print(BANDS[i].name);
    c.print(F("&port=0' class='mdot"));
    if (g_bandMap[i] == 0) c.print(F(" on"));
    c.print(F("'>"));
    c.print(g_bandMap[i] == 0 ? F("&#9679;") : F("&middot;"));
    c.print(F("</a></td>"));
    for (int p = 1; p <= NUM_PORTS; p++) {
      c.print(F("<td><a href='/bandmap?band="));
      c.print(BANDS[i].name);
      c.print(F("&port="));
      c.print(p);
      c.print(F("' class='mdot"));
      if (g_bandMap[i] == p) c.print(F(" on"));
      c.print(F("'>"));
      c.print(g_bandMap[i] == p ? F("&#9679;") : F("&middot;"));
      c.print(F("</a></td>"));
    }
    c.print(F("</tr>"));
  }
  c.print(F("</table></div></div></div>"));

  // ── Tab: Radios ────────────────────────────────────────────────────────────
  c.print(F("<div id='radios' class='tp'><div class='card'><h2>Discovered Radios</h2>"));
  if (g_radioCount == 0) {
    c.print(F("<p style='color:#6b8099;font-size:13px'>Listening for FlexRadio beacons&hellip;</p>"));
  } else {
    c.print(F("<table><tr><th>Name</th><th>IP</th><th>Status</th><th></th></tr>"));
    for (int i = 0; i < g_radioCount; i++) {
      bool following = (strcmp(g_flexIP, g_radios[i].ip) == 0);
      bool stale     = (millis() - g_radios[i].lastSeen > 30000);
      c.print(F("<tr><td>"));
      c.print(g_radios[i].name);
      c.print(F("</td><td style='color:#6b8099'>"));
      c.print(g_radios[i].ip);
      c.print(F("</td><td>"));
      if (following)
        c.print(F("<span style='color:#00d8ef;font-size:12px'>&#9679; Following</span>"));
      else if (stale)
        c.print(F("<span style='color:#6b8099;font-size:12px'>Offline</span>"));
      else
        c.print(F("<span style='color:#22d46a;font-size:12px'>Active</span>"));
      c.print(F("</td><td>"));
      if (!following) {
        c.print(F("<a href='/selectradio?ip="));
        c.print(g_radios[i].ip);
        c.print(F("&name="));
        c.print(g_radios[i].name);
        c.print(F("' class='save' style='padding:4px 12px;font-size:12px'>Follow</a>"));
      }
      c.print(F("</td></tr>"));
    }
    c.print(F("</table>"));
  }
  c.print(F("</div>"));  // card

  c.print(F("<div class='card'><h2>Manual Entry</h2>"
    "<form action='/selectradio' method='get'>"
    "<label>IP Address</label>"
    "<input type='text' name='ip' placeholder='e.g. 10.0.0.250' value='"));
  c.print(g_flexIP);
  c.print(F("'><label>Display Name</label>"
    "<input type='text' name='name' placeholder='e.g. FLEX-6700' value='"));
  c.print(g_flexName);
  c.print(F("'><input type='submit' class='save' value='Follow'>"
    "</form></div></div>"));  // card + tab pane

  // ── Tab: WiFi + factory reset ──────────────────────────────────────────────
  c.print(F("<div id='wifi' class='tp'><div class='card'><h2>WiFi</h2>"
    "<form action='/wifi' method='get'>"
    "<label>SSID</label><input type='text' name='ssid' value='"));
  c.print(g_ssid);
  c.print(F("'><label>Password</label><input type='password' name='pass' value='"));
  c.print(g_pass);
  c.print(F("'><input type='submit' class='save' value='Save &amp; Reboot'></form></div>"));

  c.print(F("<div class='card'><h2>Factory Reset</h2>"
    "<p style='font-size:13px;color:#6b8099;margin-bottom:8px'>"
    "Clears antenna names, band map, and WiFi credentials.</p>"
    "<a href='/reset' class='danger' "
    "onclick=\"return confirm('Reset all settings to defaults?')\">Factory Reset</a>"
    "</div></div>"));

  // Tab switching JS — reads hash on load so save-redirects land on correct tab
  c.print(F("<script>"
    "function showTab(t){"
    "document.querySelectorAll('.tp').forEach(e=>e.style.display='none');"
    "document.querySelectorAll('.tb').forEach(e=>e.classList.remove('on'));"
    "document.getElementById(t).style.display='block';"
    "document.querySelector('.tb[href=\"#'+t+'\"]').classList.add('on');}"
    "document.querySelectorAll('.tb').forEach(function(a){"
    "a.addEventListener('click',function(e){"
    "e.preventDefault();showTab(this.href.split('#')[1]);});});"
    "showTab(location.hash.slice(1)||'antennas');"
    "</script></body></html>"));
}

// ── HTTP: Nextion layout templates (800×480, screenshot for HMI button images) ─
void sendNextionTemplate(WiFiClient& c, bool allOn) {
  c.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"));
  c.print(F("<!DOCTYPE html><html><head><meta charset='UTF-8'>"
    "<title>Nextion Template</title><style>"
    "*{box-sizing:border-box;margin:0;padding:0}"
    "body{width:800px;height:480px;overflow:hidden;"
    "background:#09101f;color:#dde6f0;"
    "font-family:'Segoe UI',sans-serif;padding:12px;"
    "display:flex;flex-direction:column;gap:8px}"
    ".hdr{display:flex;justify-content:space-between;"
    "align-items:center;height:44px;flex-shrink:0}"
    ".title{color:#00d8ef;font-size:18px;font-weight:bold}"
    ".sub{color:#6b8099;font-size:12px;margin-top:2px}"
    ".clk{color:#00d8ef;font-size:26px;font-weight:bold}"
    ".band{flex-shrink:0;height:40px;background:#0d1628;"
    "border:1px solid #1c2a40;border-radius:6px;"
    "display:flex;align-items:center;padding:0 16px;gap:14px}"
    ".blbl{color:#6b8099;font-size:10px;text-transform:uppercase;"
    "letter-spacing:1px}"
    ".bval{color:#00d8ef;font-size:22px;font-weight:bold}"
    ".grid{flex:1;display:grid;grid-template-columns:repeat(4,1fr);gap:8px}"
    ".btn{background:#0f1e30;border:1px solid #1c2a40;border-radius:6px;"
    "display:flex;flex-direction:column;align-items:center;"
    "justify-content:center;gap:8px}"
    ".btn.on{background:#0a2a2e;border:2px solid #00d8ef}"
    ".bport{font-size:13px;font-weight:bold;"
    "text-transform:uppercase;letter-spacing:1px}"
    ".bport.off{color:#6b8099}"
    ".bport.on{color:#00d8ef}"
    ".st{flex-shrink:0;height:20px;display:flex;align-items:center}"
    ".st span{color:#6b8099;font-size:11px}"
    "</style></head><body>"));

  // Header
  c.print(F("<div class='hdr'><div>"
    "<div class='title'>G0JKN ShackSwitch &mdash; 1&times;4</div>"
    "<div class='sub'>"));
  c.print(WiFi.localIP().toString());
  c.print(F("</div></div><div class='clk' id='clk'>--:--z</div></div>"));

  // Band bar
  c.print(F("<div class='band'>"
    "<span class='blbl'>Active Band</span>"
    "<span class='bval'>&mdash;</span></div>"));

  // Two rows of 4 buttons — port number only, name area left blank for Nextion text overlay
  for (int row = 0; row < 2; row++) {
    c.print(F("<div class='grid'>"));
    for (int col = 0; col < 4; col++) {
      int i = row * 4 + col + 1;
      c.print(F("<div class='btn"));
      if (allOn) c.print(F(" on"));
      c.print(F("'>"
        "<div style='height:40px'></div>"));  // blank space for antenna name overlay
      c.print(F("<div class='bport "));
      c.print(allOn ? F("on") : F("off"));
      c.print(F("'>Port "));
      c.print(i);
      c.print(F("</div></div>"));
    }
    c.print(F("</div>"));
  }

  // Status bar
  c.print(F("<div class='st'><span>"));
  c.print(allOn ? F("ALL ON &mdash; active button template") : F("ALL OFF &mdash; inactive button template"));
  c.print(F(" &mdash; screenshot at 800&times;480</span></div>"
    "<script>"
    "function utc(){var d=new Date();"
    "var h=('0'+d.getUTCHours()).slice(-2);"
    "var m=('0'+d.getUTCMinutes()).slice(-2);"
    "document.getElementById('clk').textContent=h+':'+m+'z';}"
    "utc();setInterval(utc,30000);"
    "</script></body></html>"));
}

// ── HTTP request handler ───────────────────────────────────────────────────────
void handleHttp(WiFiClient& client) {
  unsigned long t = millis();

  // Read request line
  String reqLine;
  while (client.connected() && millis() - t < 2000) {
    if (!client.available()) continue;
    char ch = client.read();
    if (ch == '\n') break;
    if (ch != '\r') reqLine += ch;
  }

  // Drain headers (look for blank line)
  String hbuf;
  while (client.connected() && millis() - t < 3000) {
    if (!client.available()) continue;
    hbuf += (char)client.read();
    if (hbuf.endsWith("\r\n\r\n")) break;
    if (hbuf.length() > 1024) break;  // safety limit
  }

  // Parse: "GET /path?query HTTP/1.1"
  int s1 = reqLine.indexOf(' ');
  int s2 = reqLine.indexOf(' ', s1 + 1);
  if (s1 < 0 || s2 < 0) { client.stop(); return; }
  String url  = reqLine.substring(s1 + 1, s2);
  int    qPos = url.indexOf('?');
  String path  = (qPos >= 0) ? url.substring(0, qPos) : url;
  String query = (qPos >= 0) ? url.substring(qPos + 1) : "";

  char qBuf[256]; query.toCharArray(qBuf, sizeof(qBuf));
  char vBuf[64];

  Serial.print(F("HTTP ")); Serial.println(url);

  // ── Route ─────────────────────────────────────────────────────────────────
  if (path == "/" || path == "/index.html") {
    sendMainPage(client);

  } else if (path == "/settings") {
    sendSettingsPage(client);

  } else if (path == "/status") {
    sendStatusJson(client);

  } else if (path == "/select") {
    if (getParam(qBuf, "port", vBuf, sizeof(vBuf))) selectPort(atoi(vBuf));
    httpRedirect(client, "/");

  } else if (path == "/setband" || path == "/kk1l/setband") {
    if (getParam(qBuf, "band", vBuf, sizeof(vBuf))) setBand(vBuf);
    sendStatusJson(client);

  } else if (path == "/names") {
    for (int i = 1; i <= NUM_PORTS; i++) {
      char key[4]; snprintf(key, sizeof(key), "n%d", i);
      if (getParam(qBuf, key, vBuf, sizeof(vBuf))) {
        strncpy(g_antName[i-1], vBuf, 19);
        g_antName[i-1][19] = '\0';
      }
    }
    saveConfig();
    httpRedirect(client, "/settings#antennas");

  } else if (path == "/bandmap") {
    char bBuf[8], pBuf[4];
    if (getParam(qBuf, "band", bBuf, sizeof(bBuf)) &&
        getParam(qBuf, "port", pBuf, sizeof(pBuf))) {
      int idx = bandIndex(bBuf);
      int  p  = atoi(pBuf);
      if (idx >= 0 && p >= 0 && p <= NUM_PORTS) {
        g_bandMap[idx] = (uint8_t)p;
        saveConfig();
      }
    }
    httpRedirect(client, "/settings#bandmap");

  } else if (path == "/wifi") {
    char sBuf[64], paBuf[64];
    if (getParam(qBuf, "ssid", sBuf, sizeof(sBuf)) &&
        getParam(qBuf, "pass", paBuf, sizeof(paBuf))) {
      strncpy(g_ssid, sBuf, sizeof(g_ssid) - 1);
      strncpy(g_pass, paBuf, sizeof(g_pass) - 1);
      saveConfig();
      client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"
        "<html><body style='background:#09101f;color:#dde6f0;"
        "font-family:sans-serif;padding:24px'>"
        "<h2 style='color:#00d8ef'>WiFi saved. Rebooting&hellip;</h2>"
        "<p style='color:#6b8099;margin-top:8px'>"
        "Reconnect to the new network and navigate to the new IP.</p>"
        "</body></html>"));
      client.flush();
      delay(1500);
      NVIC_SystemReset();
    } else {
      httpRedirect(client, "/settings");
    }

  } else if (path == "/selectradio") {
    char ipBuf[16], nameBuf[20];
    if (getParam(qBuf, "ip", ipBuf, sizeof(ipBuf))) {
      strncpy(g_flexIP,   ipBuf,   sizeof(g_flexIP)   - 1);
      if (getParam(qBuf, "name", nameBuf, sizeof(nameBuf)))
        strncpy(g_flexName, nameBuf, sizeof(g_flexName) - 1);
      saveFlexConfig();
      flexClient.stop();
      lastFlexCheck = 0;   // reconnect immediately on next flexLoop()
      Serial.print(F("Radio selected: ")); Serial.print(g_flexName);
      Serial.print(F(" @ ")); Serial.println(g_flexIP);
    }
    httpRedirect(client, "/settings#radios");

  } else if (path == "/nextion/off") {
    sendNextionTemplate(client, false);

  } else if (path == "/nextion/on") {
    sendNextionTemplate(client, true);

  } else if (path == "/reset") {
    selectPort(0);
    g_band[0] = '\0';
    applyDefaults();
    saveConfig();
    httpRedirect(client, "/");

  } else {
    client.print(F("HTTP/1.1 404 Not Found\r\nContent-Type: text/html\r\n"
      "Connection: close\r\n\r\n"
      "<html><body style='background:#09101f;color:#ef4444;padding:24px'>"
      "<h2>404</h2></body></html>"));
  }

  client.flush();
  delay(2);
  client.stop();
}

// ── setup ──────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("\n=== ShackSwitch R4 1x4 ==="));

  for (int i = 0; i < NUM_PORTS; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], LOW);
  }

  matrix.begin();
  updateMatrix(0);

  loadConfig();
  Serial.print(F("SSID: ")); Serial.println(g_ssid);

  Serial1.begin(NXT_BAUD);
  delay(200);
  nxtSend("page 0");   // force splash — visible while WiFi connects (~15s)

  connectWifi();
  nxtInit();           // switch to page 1, push initial state

  if (WiFi.status() == WL_CONNECTED) {
    httpServer.begin();
    agServer.begin();
    agUdp.begin(AG_PORT + 1);  // local port 9008 — avoids clash with TCP server on 9007
    int flexUdpOk = flexUdp.begin(4992);
    Serial.print(F("flexUdp port 4992: "));
    Serial.println(flexUdpOk ? F("OK") : F("FAILED"));
    Serial.print(F("Web UI: http://"));
    Serial.print(WiFi.localIP());
    Serial.print(F(":"));
    Serial.println(HTTP_PORT);
    Serial.print(F("AG TCP: "));
    Serial.print(WiFi.localIP());
    Serial.print(F(":"));
    Serial.println(AG_PORT);
    if (g_flexIP[0])  { Serial.print(F("Flex:   ")); Serial.println(g_flexIP); }
    else                Serial.println(F("Flex:   auto-discovering..."));
  }
}

// ── loop ───────────────────────────────────────────────────────────────────────
void loop() {
  // WiFi watchdog — reconnect every 30 s if dropped
  if (millis() - lastWifiCheck >= 30000) {
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println(F("WiFi lost — reconnecting"));
      connectWifi();
      if (WiFi.status() == WL_CONNECTED) {
        httpServer.begin();
        agServer.begin();
        agUdp.begin(AG_PORT + 1);
      }
    }
  }

  WiFiClient httpClient = httpServer.available();
  if (httpClient) handleHttp(httpClient);

  agLoop();
  flexLoop();
  nxtLoop();
}
