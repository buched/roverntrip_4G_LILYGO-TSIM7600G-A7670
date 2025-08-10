// base du code : Julien ANCELIN. ajout du mode de transmission UDP,BT,BLE et TCP : Buched, interface de configuration web
/*
=============================================================================================
* By: HnV CAD / Herve JOLIVET 
  Date: June 17th, 2025
  License: MIT.

* Souce: https://github.com/jancelin/physalia
* By: INRAE / Julien Ancelin & Romain Tourte, Quentin Lesniack
* License: GNU Affero General Public License v3.0

* Object: 
  - Connect to 4g LTE network and get RTCM data from CentipedeRTK caster as a Client with web interface control / compteur 
  - Transmit Lat long positions to the caster for base selection automatique

* Material:
  - ESP32 with Pcie + RJ45: Lilygo T-pcie
  - LTE 4G: SIM7600 / A7670 (ici SIM7600)
  - GNSS ZED-F9P
=============================================================================================
*/
// Select your modem:
#define TINY_GSM_MODEM_SIM7600
//#define TINY_GSM_MODEM_A7670

bool debuggprint = false;

//RTK connection
String webCaster = "crtk.net";
uint16_t webPort = 2101;
String webMount = "NEAR";
String webUser = "centipede";
String webPW = "centipede";
String webSSID = "";
String webSSIDPW = "";
uint16_t webTCPPort = 2102;
String webAPN    = "sl2sfr";
String webSIMPASS = "";
String webSIMUSER = "";
const bool transmitLocation = true; //Send gga to caster
unsigned long lastGGA = 0;
unsigned long timeSendGGA = 10000;

String ggaDefaut = "";

#define MAX_IDS 50
struct RTCM_ID_Counter {
  uint16_t id;
  uint32_t count;
};
RTCM_ID_Counter idCounters[MAX_IDS];
unsigned long lastDisplay = 0;
unsigned long timeStateCaster = 0;
unsigned long durationData = 0;
float secondsData = 0.0;
float kbData = 0.00;
float kbpsData = 0.00;
String signalRSSI = "0";
unsigned long countRtcm = 0;
// Mesure du temps de téléchargement
unsigned long startData = 0;
unsigned long totalData = 0;

int ACQUISION_PERIOD_4G = 120; // secondes

#ifdef TINY_GSM_MODEM_SIM7600
  #define UART_BAUD 115200   // Modem
  #define PIN_RX 26
  #define PIN_TX 27
  #define MODEM_PWRKEY 4
  #define MODEM_POWER 25
  #include <XPowersLib.h>
  #include <SoftwareSerial.h>
  #define RS232_BAUD 115200
  #define RS232_RX    18
  #define RS232_TX    5
  SoftwareSerial RS232Serial(RS232_RX, RS232_TX);
  #define GNSSBAUD 460800
  #define GNSS_TX    32 //=> vers RX GNSS
  #define GNSS_RX    33 //=> vers TX GNSS
  #define PIN_MODE_0    14
  #define PIN_MODE_1    15
  #define PIN_MODE_2    23
  #ifndef PMU_WIRE_PORT
  #define PMU_WIRE_PORT   Wire
  #endif
  XPowersLibInterface *PMU = NULL;
  const uint8_t i2c_sda = 21;
  const uint8_t i2c_scl = 22;
  const uint8_t PMU_IRQ = 35;
  bool pmu_irq = false;
#elif defined(TINY_GSM_MODEM_A7670)
  #define UART_BAUD 115200
  #define PIN_RX       27
  #define PIN_TX       26
  #define MODEM_PWRKEY   4
  #define MODEM_DTR      25
  #define BOARD_POWERON  12
  #define BAT_ADC_PIN 35
  #include <SoftwareSerial.h>
  #define RS232_BAUD 115200
  #define RS232_RX    18
  #define RS232_TX    5
  SoftwareSerial RS232Serial(RS232_RX, RS232_TX);
  #define GNSSBAUD 460800
  #define GNSS_TX    32
  #define GNSS_RX    33
  #define PIN_MODE_0    14
  #define PIN_MODE_1    15
  #define PIN_MODE_2    23
#endif

float lastBatPercent = 0.0;
float lastBatVoltage = 0.0;
int vref = 1100;
uint32_t timeStamp = 0;

//GSM----------------------------
#define TINY_GSM_RX_BUFFER 1024

#include <HardwareSerial.h>
HardwareSerial SerialAT(1);   // UART1 - Modem
HardwareSerial GNSSSerial(2); // UART2 - GNSS

// Debug TinyGSM
#define TINY_GSM_DEBUG Serial

// Connexion
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

#define GSM_PIN ""
const char apn[]      = "sl2sfr";
const char gprsUser[] = "";
const char gprsPass[] = "";

#include "TinyGsmClientfork.h"
#include <WiFiUdp.h>
#include <BluetoothSerial.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <string.h>

// ---- FreeRTOS (ajouts) ----
TaskHandle_t hTaskNTRIP = nullptr;
TaskHandle_t hTaskGNSS  = nullptr;
TaskHandle_t hTaskWeb   = nullptr;

SemaphoreHandle_t mtxGGA    = nullptr; // protège ggaDefaut + ggaOk
SemaphoreHandle_t mtxGNSSIO = nullptr; // protège GNSSSerial.write()

volatile bool ggaOk = false;

// ---------------------------

bool tcpConnected = false;
Preferences prefs;

#define MODE_UDP      0
#define MODE_BT       1
#define MODE_BLE      2
#define MODE_TCP      3
#define MODE_RS232    4

int outputMode = MODE_UDP;

WiFiUDP udp;

BluetoothSerial SerialBT;
BLEServer* pServer = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
bool deviceConnected = false;

const char* ap_ssid = "NTRIP_AP";
const char* ap_pass = "12345678";
IPAddress udpRemoteIp;
uint16_t udpRemotePort = 9999;

// BLE UUIDs UART Nordic
#define SERVICE_UUID   "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHAR_UUID_TX   "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#if TINY_GSM_USE_GPRS && not defined TINY_GSM_MODEM_HAS_GPRS
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS false
#define TINY_GSM_USE_WIFI true
#define TINY_GSM_POWERDOWN true
#endif
#if TINY_GSM_USE_WIFI && not defined TINY_GSM_MODEM_HAS_WIFI
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#define TINY_GSM_POWERDOWN true
#endif

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif

TinyGsmClient ntripClient(modem,2);

#include <ArduinoJson.h>
#include <PubSubClient.h>

#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h"
#else
#include <Base64.h>
#endif

long lastReconnectAttempt = 0;

/* CONFIG PERIOD DE CAPTATION EN RTK*/
bool state_fix = false;
long nb_millisecond_recorded = 0;
long lastState = 0;
long lastNetworkAttemps = 0;

void callback(char* topic, byte* payload, unsigned int length) {}

//GNSS Global variables
unsigned long lastReceivedRTCM_ms = 0;
const unsigned long maxTimeBeforeHangup_ms = 10000UL;

// WiFi + Web
#include <WiFi.h>
#include <WebServer.h>
const char* ssid = "RTCM_Monitor";
const char* password = "";
WebServer server(80);
WiFiServer *tcpServer = nullptr;
WiFiClient tcpClient;
unsigned long lastGgaTime = 0;
const unsigned long ggaInterval = 10000; // 10 sec

// ---------- Fonctions existantes ----------
void readpmu()
{
  #ifdef TINY_GSM_MODEM_SIM7600
  lastBatVoltage = PMU->getBattVoltage();
  lastBatVoltage = lastBatVoltage /1000;
  if (PMU->isBatteryConnect()) {
    lastBatPercent = PMU->getBatteryPercent();
  }
  #endif
}

float readBatteryPercent() {
  #ifdef TINY_GSM_MODEM_A7670
    uint16_t adcRaw = analogRead(BAT_ADC_PIN);
    float voltage = ((float)adcRaw / 4095.0) * 3.3;
    voltage *= 2.0;
    lastBatVoltage = voltage;
    float percent = (voltage - 3.0) / (4.2 - 3.0) * 100.0;
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    lastBatPercent = percent;
    return percent;
  #endif
  return 0;
}

void loadPreferences() {
  prefs.begin("ntripcfg", true); // lecture seule
  webCaster  = prefs.getString("caster",  webCaster);
  webPort    = prefs.getUShort("port",    webPort);
  webMount   = prefs.getString("mount",   webMount);
  webUser    = prefs.getString("user",    webUser);
  webPW      = prefs.getString("pass",    webPW);
  webSSID    = prefs.getString("ssid",    webSSID);
  webSSIDPW  = prefs.getString("ssidpw",  webSSIDPW);
  webTCPPort = prefs.getUShort("tcpport", webTCPPort);
  webAPN     = prefs.getString("apn", webAPN);
  webSIMPASS  = prefs.getString("simpass", webSIMPASS);
  webSIMUSER = prefs.getString("simuser", webSIMUSER);
  prefs.end();
}

// Extraction de l’ID RTCM et comptage
void parseRTCMMessage(uint8_t *msg, uint16_t length) {
  if (length < 2) return;
  uint16_t id = ((msg[3] << 4) | (msg[4] >> 4)) & 0x0FFF;
  for (int i = 0; i < MAX_IDS; i++) {
    if (idCounters[i].id == id) { idCounters[i].count++; return; }
    else if (idCounters[i].id == 0) { idCounters[i].id = id; idCounters[i].count = 1; return; }
  }
}

// Page HTML
void handleRoot() {
  String batHtml = "<p>Batterie : ";
  batHtml += String(lastBatPercent, 1) + "% (" + String(lastBatVoltage, 2) + "V)</p>";

  server.send(200, "text/html", R"rawliteral(
    <html>
      <head>
        <title>RTCM Monitor</title>
        <meta http-equiv='refresh' content=''>
        <link rel="shortcut icon" href="https://map.centipede-rtk.org/assets/favicon/favicon.ico">
        <style>
          body { font-family: sans-serif; padding: 20px; }
          table { border-collapse: collapse; }
          th, td { padding: 6px 12px; border: 1px solid #ccc; }
        </style>
        <script>
          setInterval(() => {
            fetch('/data').then(res => res.text()).then(html => {
              document.getElementById('table').innerHTML = html;
            });
          }, 1000);
        </script>
      </head>
      <body>
        <p><a href="/config">configuration du rover</a></p>
        )rawliteral" + batHtml + R"rawliteral(
        <h1>Etat GNSS RTK  -  RTCM </h1>
        <h2>Caster Connection: </h2>
        <p>Serveur : 
        )rawliteral"+ String(webCaster) +"</p><p>Port : " + String(webPort) + "</p><p> Mountpoint : " + String(webMount) + R"rawliteral(  </p>
        <table id='table'></table>
      </body>
    </html>
  )rawliteral");
}

// Route AJAX
void handleData() {
  String html = "<h2> Info connexion Debit actuel : </h2>";
  html +="<p> Signal RSSI :" + String(signalRSSI) + "dBm </p>";
  html +="<p> Débit :" + String(kbpsData) + "kbs </p>";
  html +="<p> Nbre connexion Caster : " + String(countRtcm) + "</p>";
  html += "<h2>Compteurs RTCM</h2> ";
  html +="<tr><th>Type</th><th>Compteur RTCM Trame</th></tr>";
  for (int i = 0; i < MAX_IDS; i++) {
    if (idCounters[i].id != 0) {
      html += "<tr><td>" + String(idCounters[i].id) + "</td><td>" + String(idCounters[i].count) + "</td></tr>";
    }
  }
  server.send(200, "text/html", html);
}

void handleConfig() {
  String html = "<html><head><title>Configuration - Rover GNSS</title><meta http-equiv='Content-Type' content='text/html; charset=utf-8'><style>fieldset { background-color: rgb(214 229 231 /35%); }</style></head><body><h1>Config NTRIP et WiFi</h1>";
  html += "<form action='/setconfig' method='get'>";
  html += "<fieldset><legend>Ntrip</legend>";
  html += "Caster Host: <input name='caster' value='" + webCaster + "'><br>";
  html += "Port: <input name='port' value='" + String(webPort) + "'><br>";
  html += "Mountpoint: <input name='mount' value='" + webMount + "'><br>";
  html += "NTRIP User: <input name='ntripuser' value='" + webUser + "'><br>";
  html += "NTRIP Password: <input name='ntrippw' type='password' value='" + webPW + "'><br>";
  html += "</fieldset>";
  html += "<fieldset><legend>Wifi</legend>";
  html += "WiFi SSID: <input name='ssid' value='" + webSSID + "'><br>";
  html += "WiFi Password: <input name='ssidpw' type='password' value='" + webSSIDPW + "'><br>";
  html += "</fieldset>";
  html += "<fieldset><legend>TCP Output</legend>";
  html += "TCP Port: <input name='tcpport' value='" + String(webTCPPort) + "'><br>";
  html += "</fieldset>";
  html += "<fieldset><legend>4G/LTE SIM</legend>";
  html += "APN: <input name='apn' value='" + webAPN + "'><br>";
  html += "PIN SIM: <input name='simpass' value='" + webSIMPASS + "'><br>";
  html += "SIM User: <input name='simuser' value='" + webSIMUSER + "'><br>";
  html += "</fieldset>";
  html += "<input type='submit' value='Valider'></form>";
  html += "<form action='/reboot' method='post'><button type='submit'>Redémarrer l'appareil</button></form><p><a href=\"/\">Retour</a></p></body></html>";
  server.send(200, "text/html", html);
}

void handleReboot() {
  server.send(200, "text/html", 
    "<html><head><title>Configuration - Rover GNSS</title>"
    "<meta http-equiv='Content-Type' content='text/html; charset=utf-8'>"
    "<meta http-equiv='refresh' content='10;URL=/'>"
    "</head><body>Redémarrage...</body></html>");
  delay(200);
  ESP.restart();
}

void savePreferences() {
  prefs.begin("ntripcfg", false);
  prefs.putString("caster",  webCaster);
  prefs.putUShort("port",    webPort);
  prefs.putString("mount",   webMount);
  prefs.putString("user",    webUser);
  prefs.putString("pass",    webPW);
  prefs.putString("ssid",    webSSID);
  prefs.putString("ssidpw",  webSSIDPW);
  prefs.putUShort("tcpport", webTCPPort);
  prefs.putString("apn", webAPN);
  prefs.putString("simpass", webSIMPASS);
  prefs.putString("simuser", webSIMUSER);
  prefs.end();
}

void handleSetConfig() {
  if (server.hasArg("caster"))    webCaster  = server.arg("caster");
  if (server.hasArg("port"))      webPort    = server.arg("port").toInt();
  if (server.hasArg("mount"))     webMount   = server.arg("mount");
  if (server.hasArg("ntripuser")) webUser    = server.arg("ntripuser");
  if (server.hasArg("ntrippw"))   webPW      = server.arg("ntrippw");
  if (server.hasArg("ssid"))      webSSID    = server.arg("ssid");
  if (server.hasArg("ssidpw"))    webSSIDPW  = server.arg("ssidpw");
  if (server.hasArg("tcpport"))   webTCPPort = server.arg("tcpport").toInt();
  if (server.hasArg("apn"))       webAPN     = server.arg("apn");
  if (server.hasArg("simpass"))   webSIMPASS = server.arg("simpass");
  if (server.hasArg("simuser"))   webSIMUSER = server.arg("simuser");
  savePreferences();
  server.sendHeader("Location", "/config", true);
  server.send(200, "text/html",
    "<html><head><meta http-equiv='refresh' content='1;URL=/config'><meta http-equiv='Content-Type' content='text/html; charset=utf-8'></head>"
    "<body><h1>Paramètres enregistrés !</h1><p>Retour à la configuration...</p></body></html>");
}

/// COMPTAGE / métriques
void displayCounters() {
  if (millis() - lastDisplay > 10000) {
    if (debuggprint) {
      Serial.println("\n--- Compteurs RTCM ---");
      for (int i = 0; i < MAX_IDS; i++) {
        if (idCounters[i].id != 0) {
          Serial.print("Type "); Serial.print(idCounters[i].id);
          Serial.print(" : "); Serial.println(idCounters[i].count);
        }
      }
    }
    int signal = modem.getSignalQuality();
    signalRSSI = String(signal);
    if (debuggprint) {
      Serial.print("Signal RSSI: "); Serial.println(signal);
    }
    durationData = millis() - lastDisplay;
    secondsData = durationData / 1000.0;
    kbData = totalData / 1024.0;
    kbpsData = kbData / secondsData;
    if (debuggprint) {
      Serial.print("Débit estimé : "); Serial.print(kbpsData, 2); Serial.println(" KB/s");
      Serial.print("Nombre perte connexion CASTER / GSM =  "); Serial.println(countRtcm);
    }
    lastDisplay = millis();
    startData = millis();
    totalData = 0;
  }
}

// CRC-24Q
uint32_t crc24q_table[256];
void init_crc24q_table() {
  const uint32_t POLY = 0x1864CFB;
  for (int i = 0; i < 256; i++) {
    uint32_t crc = i << 16;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x800000) crc = (crc << 1) ^ POLY;
      else crc <<= 1;
    }
    crc24q_table[i] = crc & 0xFFFFFF;
  }
}
uint32_t compute_crc24q(const uint8_t *data, size_t len) {
  uint32_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    uint8_t idx = ((crc >> 16) ^ data[i]) & 0xFF;
    crc = ((crc << 8) ^ crc24q_table[idx]) & 0xFFFFFF;
  }
  return crc;
}

void pushGPGGA() { // version non-RTOS (non utilisée par les tasks)
  if (ggaDefaut.length() < 10 || !ggaDefaut.startsWith("$")) {
    Serial.println("[NTRIP] Aucun GGA reçu du GNSS, envoi annulé.");
    return;
  }
  if (debuggprint) {
    Serial.print(F("Pushing GGA to server: "));
    Serial.print(ggaDefaut);
  }
  String ggaToSend = ggaDefaut;
  if (!ggaToSend.endsWith("\r\n")) ggaToSend += "\r\n";
  ntripClient.print(String("GET / HTTP/1.0\r\n" + ggaToSend + "\r\n"));
  lastGGA = millis();
}

inline bool hasPrefix(const char* buf, const char* p) {
  return strncmp(buf, p, strlen(p)) == 0;
}
bool isNmeaOfInterest(const char* buf) {
  return hasPrefix(buf,"$GNGGA") || hasPrefix(buf,"$GPGGA") ||
         hasPrefix(buf,"$GPRMC") || hasPrefix(buf,"$GNRMC") ||
         hasPrefix(buf,"$GPVTG") || hasPrefix(buf,"$GNVTG") ||
         hasPrefix(buf,"$GPZDA") || hasPrefix(buf,"$GNZDA");
}

void setupOutputMode() {
  pinMode(PIN_MODE_0, INPUT_PULLUP);
  pinMode(PIN_MODE_1, INPUT_PULLUP);
  pinMode(PIN_MODE_2, INPUT_PULLUP);
  int m0 = digitalRead(PIN_MODE_0);
  int m1 = digitalRead(PIN_MODE_1);
  int m2 = digitalRead(PIN_MODE_2);
  if (m0 && m1 && m2)      outputMode = MODE_UDP;  // 111
  else if (!m0 && m1 && m2) outputMode = MODE_BT;   // 011
  else if (m0 && !m1 && m2) outputMode = MODE_BLE;  // 101
  else if (!m0 && !m1 && m2)outputMode = MODE_TCP;  // 001
  else if (m0 && m1 && !m2) outputMode = MODE_RS232;// 110
  else                       outputMode = MODE_UDP;
}

void setupBTSerial() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(100);
  SerialBT.begin("ESP32_NTRIP");
  Serial.println("Bluetooth SPP démarré");
}

class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer)   { deviceConnected = true; }
  void onDisconnect(BLEServer* pServer){ deviceConnected = false; }
};

void setupBLE() {
  BLEDevice::init("ESP32_NTRIP_BLE");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(CHAR_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("BLE UART prêt");
}

void setupRS232() {
  RS232Serial.begin(RS232_BAUD);
  delay(100);
  Serial.println("RS232 initialisé sur GPIO " + String(RS232_RX) + "/" + String(RS232_TX) + " à " + String(RS232_BAUD) + " baud");
  RS232Serial.println("ESP32 NTRIP-RS232 Ready");
  RS232Serial.println("Firmware: " + String(__DATE__) + " " + String(__TIME__));
}

void sendOutput(const uint8_t* buf, size_t len) {
  if (outputMode == MODE_UDP) {
    udp.beginPacket(udpRemoteIp, udpRemotePort);
    udp.write(buf, len);
    udp.endPacket();
  }
  else if (outputMode == MODE_BT) {
    SerialBT.write(buf, len);
  }
  else if (outputMode == MODE_BLE && deviceConnected && pTxCharacteristic) {
    pTxCharacteristic->setValue((uint8_t*)buf, len);
    pTxCharacteristic->notify();
  }
  else if (outputMode == MODE_TCP) {
    if (tcpClient && tcpClient.connected()) {
      tcpClient.write(buf, len);
    }
  }
  else if (outputMode == MODE_RS232) {
    RS232Serial.write(buf, len);
  }
}

void initWifiUdpAuto() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(webSSID.c_str(), webSSIDPW.c_str());
  Serial.print("Connexion au WiFi : "); Serial.println(webSSID);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
    delay(200);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnecté à un réseau existant !");
    Serial.print("Adresse IP : "); Serial.println(WiFi.localIP());
    IPAddress ip = WiFi.localIP();
    IPAddress subnet = WiFi.subnetMask();
    udpRemoteIp = IPAddress(
      (ip[0] & subnet[0]) | (~subnet[0] & 0xFF),
      (ip[1] & subnet[1]) | (~subnet[1] & 0xFF),
      (ip[2] & subnet[2]) | (~subnet[2] & 0xFF),
      (ip[3] & subnet[3]) | (~subnet[3] & 0xFF)
    );

    Serial.print("Broadcast UDP calculé : "); Serial.println(udpRemoteIp);
    udp.begin(udpRemotePort);
  } else {
    Serial.println("\nImpossible de joindre le WiFi, création du point d'accès...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid, ap_pass);
    Serial.print("AP démarré : "); Serial.println(WiFi.softAPIP());
    udpRemoteIp = IPAddress(192, 168, 4, 255);
    Serial.print("Broadcast UDP AP : "); Serial.println(udpRemoteIp);
    udp.begin(udpRemotePort);
  }
}

void poweronmodem()
{
  #ifdef TINY_GSM_MODEM_SIM7600
    pinMode(MODEM_PWRKEY, OUTPUT);
    pinMode(MODEM_POWER, OUTPUT);
    digitalWrite(MODEM_POWER, HIGH);
    delay(100);
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(500);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(5000);
  #elif defined(TINY_GSM_MODEM_A7670)
    pinMode(BOARD_POWERON, OUTPUT);
    digitalWrite(BOARD_POWERON, HIGH);  
    pinMode(MODEM_PWRKEY, OUTPUT);
    pinMode(MODEM_DTR, OUTPUT);  
    digitalWrite(MODEM_DTR, LOW);
    delay(100);
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(100);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(1000);
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(5000);
  #else
    #error "Aucun modele selectionne!"
  #endif
}

void pmu_setup()
{
  #ifdef TINY_GSM_MODEM_SIM7600
    Wire.begin(i2c_sda, i2c_scl);
    if (!PMU) {
      PMU = new XPowersAXP2101(PMU_WIRE_PORT);
      if (!PMU->init()) {
        Serial.println("Warning: Failed to find AXP2101 power management");
        delete PMU; PMU = NULL;
      } else {
        Serial.println("AXP2101 PMU init succeeded, using AXP2101 PMU");
        PMU->setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_4V36);
        PMU->setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_2000MA);
      }
    }
    while (!PMU) {
      Serial.println("PMU not found! Check.");
      delay(1000);
    }
    PMU->setSysPowerDownVoltage(2600);
    PMU->setChargingLedMode(XPOWERS_CHG_LED_BLINK_1HZ);
    pinMode(PMU_IRQ, INPUT_PULLUP);
    attachInterrupt(PMU_IRQ, [] { pmu_irq = true; }, FALLING);
    if (PMU->getChipModel() == XPOWERS_AXP2101) {
      PMU->setProtectedChannel(XPOWERS_DCDC1);
      PMU->disablePowerOutput(XPOWERS_DCDC2);
      PMU->disablePowerOutput(XPOWERS_DCDC3);
      PMU->disablePowerOutput(XPOWERS_DCDC4);
      PMU->disablePowerOutput(XPOWERS_DCDC5);
      PMU->disablePowerOutput(XPOWERS_ALDO1);
      PMU->disablePowerOutput(XPOWERS_ALDO4);
      PMU->disablePowerOutput(XPOWERS_BLDO1);
      PMU->disablePowerOutput(XPOWERS_BLDO2);
      PMU->disablePowerOutput(XPOWERS_DLDO1);
      PMU->disablePowerOutput(XPOWERS_DLDO2);
      PMU->disablePowerOutput(XPOWERS_VBACKUP);
    }
    PMU->clearIrqStatus();
    PMU->disableInterrupt(XPOWERS_ALL_INT);
    PMU->enableInterrupt(XPOWERS_USB_INSERT_INT |
                         XPOWERS_USB_REMOVE_INT |
                         XPOWERS_BATTERY_INSERT_INT |
                         XPOWERS_BATTERY_REMOVE_INT |
                         XPOWERS_PWR_BTN_CLICK_INT |
                         XPOWERS_PWR_BTN_LONGPRESSED_INT);
    PMU->setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
  #endif
}

// ============== SETUP ==============
void setup()
{
  prefs.begin("ntripcfg", false);
  prefs.end();
  loadPreferences();
  init_crc24q_table();
  Serial.begin(115200);
  delay(200);

  // GNSS Serial 
  GNSSSerial.begin(GNSSBAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);
  delay(1000);

  // Modem
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(1500);
  poweronmodem();

  Serial.println("Initializing modem...");
  if (!modem.init()) {
    Serial.println("Failed to restart modem, attempting to continue without restarting");
  }
  delay(1000);
#ifdef TINY_GSM_MODEM_SIM7600
  pmu_setup();
#elif defined(TINY_GSM_MODEM_A7670)
  if (!modem.restart()) {
    Serial.println("modem.restart() echoue");
    while (true);
  }
#endif

  delay(1000);
  if (!modem.gprsConnect(webAPN.c_str(), webSIMUSER.c_str(), webSIMPASS.c_str())) {
    Serial.println("fail");
    delay(10000);
    return;
  }

  Serial.print("Waiting for network...");
  int lastNetworkAttemps = millis();
  int now = millis(); 

  while(!modem.waitForNetwork() && ( now - lastNetworkAttemps < ACQUISION_PERIOD_4G*1000 )) {
    Serial.println("fail to find network, waiting 10sec before retry");
    delay(10000);
    now = millis();
  }
  if (modem.isNetworkConnected()) Serial.println("Network connected");

  Serial.print(F("Connecting to ")); Serial.print(webAPN);
  if (!modem.gprsConnect(webAPN.c_str(), webSIMUSER.c_str(), webSIMPASS.c_str())) {
    Serial.println(" fail");
    delay(10000);
    return;
  }
  Serial.println(" success");
  if (modem.isGprsConnected()) Serial.println("GPRS connected");

  Serial.println(F("NTRIP testing"));

  setupOutputMode();

  if (outputMode == MODE_UDP) {
    initWifiUdpAuto();
    Serial.println("Mode selectionne: UDP (PIN1=HIGH, PIN2=HIGH, PIN3=HIGH)");
  } else if (outputMode == MODE_BT) {
    setupBTSerial();
    Serial.println("Mode selectionne: BT (PIN1=LOW, PIN2=HIGH, PIN3=HIGH)");
  } else if (outputMode == MODE_BLE) {
    setupBLE();
    Serial.println("Mode selectionne: BLE (PIN1=HIGH, PIN2=LOW, PIN3=HIGH)");
  } else if (outputMode == MODE_TCP) {
    initWifiUdpAuto();
    delay(2000);
    Serial.println("Mode selectionne: TCP (PIN1=LOW, PIN2=LOW, PIN3=HIGH)");
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi non connecté, impossible de démarrer le serveur TCP !");
    } else {
      tcpServer = new WiFiServer(webTCPPort);
      tcpServer->begin();
      tcpServer->setNoDelay(true);
      Serial.println("Serveur TCP démarré sur le port 2102");
    }
  } else if (outputMode == MODE_RS232) {
    initWifiUdpAuto();
    delay(2000);
    setupRS232();
    Serial.println("Mode selectionne: RS232 (PIN1=HIGH, PIN2=HIGH, PIN3=LOW)");
  }

  if (outputMode != MODE_BT && outputMode != MODE_BLE) {
    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.on("/config", handleConfig);
    server.on("/setconfig", handleSetConfig);
    server.on("/reboot", HTTP_POST, handleReboot);
    server.begin();
    delay(1000);
  }

  while (Serial.available()) Serial.read();

  // ===== FreeRTOS: mutex + tasks =====
  mtxGGA    = xSemaphoreCreateMutex();
  mtxGNSSIO = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(TaskNTRIP, "TaskNTRIP", 8192, nullptr, 3, &hTaskNTRIP, 0); // Core 0
  xTaskCreatePinnedToCore(TaskGNSSIO,"TaskGNSS",  6144, nullptr, 2, &hTaskGNSS,  1); // Core 1
  xTaskCreatePinnedToCore(TaskWeb,   "TaskWeb",   4096, nullptr, 1, &hTaskWeb,   0); // Core 0
}

// ============== LOOP (idle) ==============
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));

}

// -------------------- NTRIP / HTTP helpers --------------------

bool beginClient() {
  if (debuggprint) {
    Serial.print(F("Opening socket to ")); Serial.print(webCaster);
    Serial.print(F(" : ")); Serial.println(webPort);
  }

  if (!ntripClient.connect(webCaster.c_str(), webPort)) {
    if (debuggprint) Serial.println(F("Connection to caster failed"));
    return false;
  }
  countRtcm++;

  const int SERVER_BUFFER_SIZE = 512;
  char serverRequest[SERVER_BUFFER_SIZE];
  int n = snprintf(serverRequest, SERVER_BUFFER_SIZE,
                   "GET /%s HTTP/1.0\r\n"
                   "User-Agent: NTRIP Client v1.0\r\n",
                   webMount.c_str());
  if (n < 0 || n >= SERVER_BUFFER_SIZE) { ntripClient.stop(); return false; }

  char credentials[256]; credentials[0] = '\0';
  if (webUser.length() > 0) {
    String up = webUser + ":" + webPW;
    char userPass[up.length() + 1];
    up.toCharArray(userPass, sizeof(userPass));

  #if defined(ARDUINO_ARCH_ESP32)
    base64 b; String enc = b.encode(userPass);
    char encC[200]; enc.toCharArray(encC, sizeof(encC));
    snprintf(credentials, sizeof(credentials), "Authorization: Basic %s\r\n", encC);
  #else
    int encLen = base64_enc_len(strlen(userPass)); if (encLen > 180) encLen = 180;
    char encC[181] = {0}; base64_encode(encC, userPass, strlen(userPass));
    snprintf(credentials, sizeof(credentials), "Authorization: Basic %s\r\n", encC);
  #endif
  } else {
    strncpy(credentials, "Accept: */*\r\nConnection: close\r\n", sizeof(credentials)-1);
    credentials[sizeof(credentials)-1] = '\0';
  }

  size_t used = strlen(serverRequest);
  strncat(serverRequest, credentials, SERVER_BUFFER_SIZE - used - 1);
  used = strlen(serverRequest);
  strncat(serverRequest, "\r\n", SERVER_BUFFER_SIZE - used - 1);

  if (debuggprint) {
    Serial.print(F("serverRequest size: ")); Serial.println(strlen(serverRequest));
    Serial.println(F("Sending server request:")); Serial.println(serverRequest);
  }
  ntripClient.write((const uint8_t*)serverRequest, strlen(serverRequest));

  unsigned long t0 = millis();
  while (ntripClient.available() == 0) {
    if (millis() - t0 > 5000UL) { if (debuggprint) Serial.println(F("Caster timed out!")); ntripClient.stop(); return false; }
    delay(10);
  }

  int code = 0; char resp[512]; size_t pos = 0;
  while (ntripClient.available() && pos < sizeof(resp)-1) {
    resp[pos++] = ntripClient.read();
    if (code == 0) {
      if (strstr(resp, "200")) code = 200;
      if (strstr(resp, "401")) code = 401;
      if (strstr(resp, "403")) code = 403;
      if (strstr(resp, "404")) code = 404;
    }
    if (pos >= 4 && resp[pos-4]=='\r' && resp[pos-3]=='\n' && resp[pos-2]=='\r' && resp[pos-1]=='\n') break;
  }
  resp[pos] = '\0';
  if (code != 200) { if (debuggprint){ Serial.println(F("HTTP not 200")); Serial.println(resp);} ntripClient.stop(); return false; }

  lastReceivedRTCM_ms = millis();
  return true;
}

bool processConnection() {
  if (ntripClient.connected() == true) {
    uint8_t rtcmData[512 * 4];
    size_t rtcmCount = 0;

    static enum {WAIT_SYNC, READ_LENGTH_1, READ_LENGTH_2, READ_PAYLOAD} stateRtcmTrame = WAIT_SYNC;
    static uint16_t length = 0;
    static uint16_t indexRtcmTrame = 0;
    static uint8_t bufferRtcmTrame[2056];

    while (ntripClient.available()) {
      uint8_t b = ntripClient.read();
      rtcmData[rtcmCount++] = b;
      totalData = totalData+1;

      switch (stateRtcmTrame) {
          case WAIT_SYNC:
            if (b == 0xD3) {
              bufferRtcmTrame[0] = b;
              indexRtcmTrame = 1;
              stateRtcmTrame = READ_LENGTH_1;
            }
            break;

          case READ_LENGTH_1:
            bufferRtcmTrame[indexRtcmTrame++] = b;
            length = (b & 0x03) << 8;
            stateRtcmTrame = READ_LENGTH_2;
            break;

          case READ_LENGTH_2:
            bufferRtcmTrame[indexRtcmTrame++] = b;
            length |= b;
            length += 3; // + CRC
            stateRtcmTrame = READ_PAYLOAD;
            if (debuggprint && length > 1023) {Serial.println("Trame RTCM trop longue");}
            break;

          case READ_PAYLOAD:
            bufferRtcmTrame[indexRtcmTrame++] = b;
            if (indexRtcmTrame >= length + 3) {
              uint32_t crc_calc = compute_crc24q(bufferRtcmTrame, indexRtcmTrame - 3);
              uint32_t crc_recv = ((uint32_t)bufferRtcmTrame[indexRtcmTrame - 3] << 16) |
                                  ((uint32_t)bufferRtcmTrame[indexRtcmTrame - 2] << 8) |
                                  ((uint32_t)bufferRtcmTrame[indexRtcmTrame - 1]);
              if (crc_calc == crc_recv) {
                timeStateCaster = millis();
                // PROTÉGER l'accès série GNSS
                if (xSemaphoreTake(mtxGNSSIO, pdMS_TO_TICKS(50)) == pdTRUE) {
                  GNSSSerial.write(bufferRtcmTrame, indexRtcmTrame);
                  xSemaphoreGive(mtxGNSSIO);
                }
                parseRTCMMessage(bufferRtcmTrame, indexRtcmTrame);
              } else {
                if (debuggprint) {Serial.println("Trame RTCM ignorée : CRC invalide");}
              }
              indexRtcmTrame = 0;
              stateRtcmTrame = WAIT_SYNC;
            }
            break;
      }
    }

    if (rtcmCount > 0) {
      lastReceivedRTCM_ms = millis();
      if (debuggprint) {
        Serial.println();
        Serial.print(F("Pushed ")); Serial.print(rtcmCount); Serial.println(F(" RTCM bytes to GNSS "));
      }
    }
    displayCounters();
  } else {
    if (debuggprint) {Serial.println(F("Connection dropped!"));}
    return false;
  }

  if ((millis() - lastReceivedRTCM_ms) > maxTimeBeforeHangup_ms) {
    if (debuggprint) {Serial.println(F("RTCM timeout!"));}
    return false;
  }

  return true;
}

void closeConnection() {
  if (ntripClient.connected() == true) {
    ntripClient.stop();
  }
  if (debuggprint) {Serial.println(F("Disconnected!"));}
  ESP.restart(); // conserve ton comportement
}

// ------------ FreeRTOS tasks ------------

void pushGPGGA_RTOS() {
  String s;
  if (xSemaphoreTake(mtxGGA, pdMS_TO_TICKS(20)) == pdTRUE) {
    s = ggaDefaut;
    xSemaphoreGive(mtxGGA);
  }
  if (s.length() < 10 || s.charAt(0) != '$') return;

  String ggaToSend = s;
  if (!ggaToSend.endsWith("\r\n")) ggaToSend += "\r\n";
  ntripClient.print(String("GET / HTTP/1.0\r\n" + ggaToSend + "\r\n"));
  lastGGA = millis();
}

// Core 1 : lecture GNSS + diffusion
void TaskGNSSIO(void *pv) {
  static char nmea[256];
  for (;;) {
    if (GNSSSerial.available()) {
      int got = GNSSSerial.readBytesUntil('\n', (uint8_t*)nmea, sizeof(nmea)-2);
      if (got > 0) {
        nmea[got]   = '\n';
        nmea[got+1] = '\0';

        if (hasPrefix(nmea, "$GPGGA") || hasPrefix(nmea, "$GNGGA")) {
          if (xSemaphoreTake(mtxGGA, pdMS_TO_TICKS(20)) == pdTRUE) {
            ggaDefaut = String(nmea);
            ggaOk = true;
            xSemaphoreGive(mtxGGA);
          }
        }

        if (isNmeaOfInterest(nmea) && outputMode == MODE_TCP) {
          if ((!tcpClient || !tcpClient.connected()) && tcpServer) {
            WiFiClient c = tcpServer->available();
            if (c) { tcpClient = c; tcpConnected = true; Serial.println("Client TCP connecté !"); }
            else   { tcpConnected = false; }
          }
          if (tcpConnected && !tcpClient.connected()) {
            Serial.println("Client TCP déconnecté !");
            tcpClient.stop();
            tcpConnected = false;
          }
        }

        sendOutput((const uint8_t*)nmea, got+1);
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }
}

// Core 0 : socket NTRIP + RTCM → GNSS + push GGA + réseau
void TaskNTRIP(void *pv) {
  enum { open_connection, push_data_and_wait, close_connection, wait_reopen };
  int state = open_connection;

  for (;;) {
    switch (state) {
      case open_connection: {
        bool ready = false;
        if (xSemaphoreTake(mtxGGA, pdMS_TO_TICKS(20)) == pdTRUE) {
          ready = ggaOk;
          xSemaphoreGive(mtxGGA);
        }
        if (!ready) { vTaskDelay(pdMS_TO_TICKS(200)); break; }

        Serial.println(F("Connecting to the NTRIP caster..."));
        if (beginClient()) state = push_data_and_wait;
        else               vTaskDelay(pdMS_TO_TICKS(5000));
      } break;

      case push_data_and_wait: {
        if (!processConnection()) {
          state = close_connection;
          break;
        }
        static uint32_t last = 0;
        if (millis() - last > timeSendGGA) {
          last = millis();
          pushGPGGA_RTOS();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
      } break;

      case close_connection:
        closeConnection();
        state = wait_reopen;
        break;

      case wait_reopen:
        vTaskDelay(pdMS_TO_TICKS(1000));
        state = open_connection;
        break;
    }

    // Vérif réseau périodique
    static uint32_t lastNet = 0;
    if (millis() - lastNet > 2000) {
      lastNet = millis();
      if (!modem.isNetworkConnected()) {
        modem.waitForNetwork(true);
      }
      if (!modem.isGprsConnected()) {
        modem.gprsConnect(webAPN.c_str(), webSIMUSER.c_str(), webSIMPASS.c_str());
      }
    }
  }
}

// Core 0 : Web + métriques
void TaskWeb(void *pv) {
  readpmu();
  for (;;) {
    server.handleClient();
    static uint32_t last = 0;
    if (millis() - last > 1000) {
      last = millis();
      displayCounters();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
