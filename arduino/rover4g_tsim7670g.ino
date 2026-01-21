// base du code : Julien ANCELIN. ajout du mode de transmission UDP,BT,BLE et TCP : Buched, interface de configuration web
/*
=============================================================================================
* By: HnV CAD / Herve JOLIVET 
  Date: June 17th, 2025
  License: MIT.

* Souce: https://github.com/jancelin/physalia
* By: INRAE / Julien Ancelin & Romain Tourte, Quentin Lesniack
* License: GNU Affero General Public License v3.0
  
* GNSS code:
  By: SparkFun Electronics / Nathan Seidle & Paul Clark
  Date: January 13th, 2022
  License: MIT.
=============================================================================================
*/
#define TINY_GSM_MODEM_A7670  // https://lilygo.cc/products/t-sim-a7670e?variant=42737494458549

bool debuggprint = true;

//RTK connection
String webCaster = "crtk.net";
uint16_t webPort = 2101;
String webMount = "LRSEC";
String webUser = "centipede";
String webPW = "centipede";
String webSSID = "";
String webSSIDPW = "";
uint16_t webTCPPort = 2102;
String webAPN    = "orange";
String webSIMPASS = "orange";
String webSIMUSER = "orange";
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

int ACQUISION_PERIOD_4G = 120; // Temps ( en seconde ) pendant lequel on va chercher le network 4G avant de faire un deepsleep( TIME_TO_SLEEP )

  #define UART_BAUD 115200   // for modem only
  #define PIN_RX       27
  #define PIN_TX       26
  #define MODEM_PWRKEY   4
  #define MODEM_DTR      25
  #define BOARD_POWERON  12
  #define BAT_ADC_PIN 35

float lastBatPercent = 0.0;
float lastBatVoltage = 0.0;
int vref = 1100;
uint32_t timeStamp = 0;

//GSM----------------------------
// need enough space in the buffer for the entire response
// else data will be lost (and the http library will fail).
#define TINY_GSM_RX_BUFFER 1024

#include <HardwareSerial.h>
HardwareSerial SerialAT(1);  // UART1 - Modem
HardwareSerial GNSSSerial(2);      // UART2 - GNSS

#define GNSSBAUD 460800
#define GNSS_TX    34 //=> vers RX LG580P
#define GNSS_RX    32 // => vers TX LG580P

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG Serial

// Define how you're planning to connect to the internet.
// This is only needed for this example, not in other code.
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

// set GSM PIN, if any
#define GSM_PIN ""
// Your GPRS credentials, if any
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

bool tcpConnected = false;
Preferences prefs;

#define MODE_UDP      0
#define MODE_BT       1
#define MODE_BLE      2
#define MODE_TCP      3
#define PIN_MODE_0    14
#define PIN_MODE_1    15
#define PIN_MODE_2    23
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

// Just in case someone defined the wrong thing..
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
//TinyGsmClient mqttClient(modem,0);
TinyGsmClient ntripClient(modem,2);
//GSM---------------------------

#include <ArduinoJson.h>
#include <PubSubClient.h>


//The ESP32 core has a built in base64 library but not every platform does
//We'll use an external lib if necessary.
#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h" //Built-in ESP32 library
#else
#include <Base64.h> //nfriendly library from https://github.com/adamvr/arduino-base64, will work with any platform
#endif
 
//PubSubClient mqtt(mqttClient); //MQTT
long lastReconnectAttempt = 0;

/* CONFIG PERIOD DE CAPTATION EN RTK*/
bool state_fix = false;
long nb_millisecond_recorded = 0;
long lastState = 0;
long lastNetworkAttemps = 0;
void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
}


//GNSS Global variables
unsigned long lastReceivedRTCM_ms = 0;          //5 RTCM messages take approximately ~300ms to arrive at 115200bps
const unsigned long maxTimeBeforeHangup_ms = 10000UL; //If we fail to get a complete RTCM frame after 10s, then disconnect from caster

// Your WiFi connection credentials, if applicable
#include <WiFi.h>
#include <WebServer.h>
const char* ssid = "RTCM_Monitor";
const char* password = "";
WebServer server(80);
//WiFiServer tcpServer(2102);
WiFiServer *tcpServer = nullptr;
WiFiClient tcpClient;
unsigned long lastGgaTime = 0;
const unsigned long ggaInterval = 10000; // 10 sec

float readBatteryPercent() {
  #ifdef TINY_GSM_MODEM_A7670
    uint16_t adcRaw = analogRead(BAT_ADC_PIN);
    // Ajuste le facteur en fonction de ton diviseur de tension
    float voltage = ((float)adcRaw / 4095.0) * 3.3;  // 12 bits ADC, Vref 3.3 V

    // Si diviseur, par exemple 100k/100k (x2) : voltage *= 2;
    voltage *= 2.0;
    lastBatVoltage = voltage;
    // Bornes typiques LiPo
    float percent = (voltage - 3.0) / (4.2 - 3.0) * 100.0;
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    lastBatPercent = percent;
    return percent;
#endif
}


//=-=-=-=-=-=-=-=-=-=-=-=SOUS FUNCTION =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
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
  if (length < 2) return; // Trop court

  // Les 12 premiers bits après l'en-tête sont le Message ID
  uint16_t id = ((msg[3] << 4) | (msg[4] >> 4)) & 0x0FFF;

  // Incrément du compteur
  for (int i = 0; i < MAX_IDS; i++) {
    if (idCounters[i].id == id) {
      idCounters[i].count++;
      return;
    } else if (idCounters[i].id == 0) {
      idCounters[i].id = id;
      idCounters[i].count = 1;
      return;
    }
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

// Route AJAX : renvoie les compteurs en HTML brut
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

/// COMTPEUR INFORMATION 
void displayCounters() {
  if (millis() - lastDisplay > 10000) {
    if (debuggprint) {Serial.println("\n--- Compteurs RTCM ---");
      for (int i = 0; i < MAX_IDS; i++) {
        if (idCounters[i].id != 0) {
          Serial.print("Type ");
          Serial.print(idCounters[i].id);
          Serial.print(" : ");
          Serial.println(idCounters[i].count);
        }
      }
    }
    //////Actualisation des donnes web tier/////
    int signal = modem.getSignalQuality();
    signalRSSI = String(signal);
    if (debuggprint) {
      Serial.print("Signal RSSI: ");
      Serial.println(signal);
    }

    ///// donnees telechargement////
    durationData = millis() - lastDisplay;
    secondsData = durationData / 1000.0;
    kbData = totalData / 1024.0;
    kbpsData = kbData / secondsData;
    if (debuggprint) {
      Serial.print("Débit estimé : ");
      Serial.print(kbpsData, 2);
      Serial.println(" KB/s");
      Serial.print("Nombre perte connexion CASTER / GSM =  "); Serial.println(countRtcm);
    }
    lastDisplay = millis();
    startData = millis();
    totalData = 0;
  }
}


// Table pour CRC-24Q (valeur initiale : 0)
uint32_t crc24q_table[256];

// Génère la table CRC-24Q une fois au démarrage
void init_crc24q_table() {
  const uint32_t POLY = 0x1864CFB;
  for (int i = 0; i < 256; i++) {
    uint32_t crc = i << 16;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x800000)
        crc = (crc << 1) ^ POLY;
      else
        crc <<= 1;
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



void pushGPGGA()
{
  if (ggaDefaut.length() < 10 || !ggaDefaut.startsWith("$")) {
        Serial.println("[NTRIP] Aucun GGA reçu du GNSS, envoi annulé.");
        return;
    }
    if (debuggprint) {
      Serial.print(F("Pushing GGA to server: "));
   
      Serial.print(ggaDefaut); // .nmea is printable (NULL-terminated) and already has \r\n on the end
    }
    String ggaToSend = ggaDefaut;
    if (!ggaToSend.endsWith("\r\n")) ggaToSend += "\r\n";
    ntripClient.print(String("GET / HTTP/1.0\r\n" + ggaToSend + "\r\n"));
    lastGGA = millis();
}

void setupOutputMode() {
  pinMode(PIN_MODE_0, INPUT_PULLUP);
  pinMode(PIN_MODE_1, INPUT_PULLUP);
  pinMode(PIN_MODE_2, INPUT_PULLUP);
  int m0 = digitalRead(PIN_MODE_0);
  int m1 = digitalRead(PIN_MODE_1);
  int m2 = digitalRead(PIN_MODE_2);
    if (m0 && m1 && m2) // 111
      {
        outputMode = MODE_UDP;
      }
    else if (!m0 && m1 && m2) // 011
      {
        outputMode = MODE_BT;
      }
    else if (m0 && !m1 && m2) // 101
      {
        outputMode = MODE_BLE;
      }
    else if (!m0 && !m1 && m2) // 001
      {
        outputMode = MODE_TCP;
      }
    else // Défaut
      {
        outputMode = MODE_UDP;
      }
}

void setupBTSerial() {
WiFi.disconnect(true);  // Déconnecte et désactive le wifi au cas ou
WiFi.mode(WIFI_OFF);    // Éteint la pile WiFi
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

void sendOutput(const uint8_t* buf, size_t len) {
  if (outputMode == MODE_UDP) {
    udp.beginPacket(udpRemoteIp, udpRemotePort);
    udp.write(buf, len);
    udp.endPacket();
  }
  else if (outputMode == MODE_BT)
    {
      SerialBT.write(buf, len);
    }
  else if (outputMode == MODE_BLE && deviceConnected && pTxCharacteristic)
    {
      pTxCharacteristic->setValue((uint8_t*)buf, len);
      pTxCharacteristic->notify();
    }
  else if (outputMode == MODE_TCP)
    {
      if (tcpClient && tcpClient.connected())
        {
          tcpClient.write(buf, len);
        }
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

// Sequence pour A7670G
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
}


//=-=-=-=-=-=-=-=-=-=-=-=-=
void setup()
{
  prefs.begin("ntripcfg", false);
  prefs.end();
  loadPreferences();
  init_crc24q_table();
  Serial.begin(115200);  //baudrate maxi pour AOG
  delay(200);

// GNSS Serial 
  delay(10);
    GNSSSerial.begin(GNSSBAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);
  delay(1000);

//GSM-----------------------------
  delay(10);
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
   delay(1500);
  poweronmodem();

  Serial.println("Initializing modem...");
  if (!modem.init()) {
    Serial.println("Failed to restart modem, attempting to continue without restarting");
  }
  delay(1000);

  Serial.printf("restart modem");
  if (!modem.restart()) {
    Serial.println("modem.restart() echoue");
    while (true);
  }


  delay(1000);
  if (!modem.gprsConnect(webAPN.c_str(), webSIMUSER.c_str(), webSIMPASS.c_str())) {
    Serial.println("fail");
    delay(10000);
    return;
}

  Serial.print("Waiting for network...");
  int lastNetworkAttemps = millis();
  int now = millis(); 

   while(!modem.waitForNetwork() && ( now - lastNetworkAttemps < ACQUISION_PERIOD_4G ) ) {
  //if (!modem.waitForNetwork()) {
    Serial.println("fail to find network, waiting 10sec before retry");
    delay(10000);
    now = millis();
    //return;
  }

  if (modem.isNetworkConnected()) {
      Serial.println("Network connected");
  }

    Serial.print(F("Connecting to "));
    Serial.print(webAPN);
    if (!modem.gprsConnect(webAPN.c_str(), webSIMUSER.c_str(), webSIMPASS.c_str())) {
        Serial.println(" fail");
        delay(10000);
        return;
    }
    Serial.println(" success");

    if (modem.isGprsConnected()) {
        Serial.println("GPRS connected");
    }

  Serial.println(F("NTRIP testing"));

  now = millis();
  lastNetworkAttemps = millis();

  setupOutputMode();

  if (outputMode == MODE_UDP)
    {
      initWifiUdpAuto();
      Serial.println("Mode selectionne: UDP (PIN1=HIGH, PIN2=HIGH, PIN3=HIGH)");
    }
  else if (outputMode == MODE_BT)
    {
      setupBTSerial();
      Serial.println("Mode selectionne: BT (PIN1=LOW, PIN2=HIGH, PIN3=HIGH)");
    }
  else if (outputMode == MODE_BLE)
    {
      setupBLE();
      Serial.println("Mode selectionne: BLE (PIN1=HIGH, PIN2=LOW, PIN3=HIGH)");
    }
  else if (outputMode == MODE_TCP)
    {
      initWifiUdpAuto();
      delay(2000);
      Serial.println("Mode selectionne: TCP (PIN1=LOW, PIN2=LOW, PIN3=HIGH)");
      if (WiFi.status() != WL_CONNECTED)
        {
          Serial.println("WiFi non connecté, impossible de démarrer le serveur TCP !");
        } 
      else
        {
          tcpServer = new WiFiServer(webTCPPort);
          tcpServer->begin();
          tcpServer->setNoDelay(true);
          Serial.println("Serveur TCP démarré sur le port 2102");
        }
    }


if (outputMode != MODE_BT && outputMode != MODE_BLE)
  {
    // Web server routes
    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.on("/config", handleConfig);
    server.on("/setconfig", handleSetConfig);
    server.on("/reboot", HTTP_POST, handleReboot);
    server.begin();
    delay(1000);
  }

while (Serial.available()) // Empty the serial buffer
    Serial.read();
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=- LOOP  =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void loop()
{
  server.handleClient();

  static unsigned long lastBatRead = 0;
  if (millis() - lastBatRead > 5000) {
    readBatteryPercent();
    lastBatRead = millis();
  }
  static bool ggaOk = false;

  // =-=-=-=-= GNSS READ STATE =-=-=-=-=
  if (GNSSSerial.available()) // Check for a new key press
  {
    String s = GNSSSerial.readStringUntil('\n');
    //Serial.println(s); // Empty the serial buffer
    if (s.startsWith("$GNGGA") || s.startsWith("$GPGGA"))
      {
          ggaDefaut = s;
          ggaOk = true;
      }
    if (s.startsWith("$GNGGA") || s.startsWith("$GPGGA") || s.startsWith("$GPRMC") || s.startsWith("$GNRMC") || s.startsWith("$GPVTG") || s.startsWith("$NVTG") || s.startsWith("$GPZDA") || s.startsWith("$NZDA"))
      {
          if (outputMode == MODE_TCP)
          {
            // Si pas de client connecté, en accepter un nouveau
            if (!tcpClient || !tcpClient.connected())
            {
              tcpClient = tcpServer->available();
              if (tcpClient)
                {
                  tcpConnected = true;
                  Serial.println("Client TCP connecté !");
                  // Optionnel : send hello ou info
                }
              else
                {
                  tcpConnected = false;
                }
            }
            // Si déconnexion :
            if (tcpConnected && !tcpClient.connected())
              {
                Serial.println("Client TCP déconnecté !");
                tcpClient.stop();
                tcpConnected = false;
              }
          }
      }
    sendOutput((const uint8_t*)s.c_str(), s.length());
  }


  // =-=-=-=-= NTRIP STATE =-=-=-=-=
  long now = millis();
  enum states // Use a 'state machine' to open and close the connection
  {
    open_connection,
    push_data_and_wait_for_keypress,
    close_connection,
    waiting_for_keypress
  };
  static states state = open_connection;

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

  switch (state)
  {
    case open_connection:
      if (!ggaOk)
        {
          if (debuggprint) Serial.println("Attente d'une trame GGA du GNSS pour ouvrir la connexion NTRIP...");
          break; // On n'avance pas dans la state machine !
        }
      Serial.println(F("Connecting to the NTRIP caster..."));
      if (beginClient()) // Try to open the connection to the caster
      {
        if (debuggprint) {Serial.println(F("Connected to the NTRIP caster! Press any key to disconnect..."));}
        state = push_data_and_wait_for_keypress; // Move on
      }
      else
      {
        if (debuggprint) {Serial.print(F("Could not connect to the caster. Trying again in 5 seconds."));}
        for (int i = 0; i < 5; i++)
        {
          delay(1000);
          if (debuggprint) {Serial.print(F("."));}
        }
        Serial.println();
      }
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    case push_data_and_wait_for_keypress:
      if ((processConnection() == false))
      {
        state = close_connection; // Move on
      }
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    case close_connection:
      if (debuggprint) {Serial.println(F("Closing the connection to the NTRIP caster..."));}
      closeConnection();
      if (debuggprint) {Serial.println(F("Press any key to reconnect..."));}
      state = waiting_for_keypress; // Move on
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    case waiting_for_keypress:
      state = open_connection; // Move on
      break;
  }
  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  
  //GSM-------------------------------
   now = millis();
    // Make sure we're still registered on the network
    if (!modem.isNetworkConnected()) {
      lastNetworkAttemps = millis();
      if (debuggprint) {Serial.println("LOOP - Network disconnected");}

      // Testing 4G connection during ACQUISION_PERIOD_4G ( second ), if not connected after that, DeepSleep is launched
      while(!modem.waitForNetwork() && ( now - lastNetworkAttemps < ACQUISION_PERIOD_4G ) ) {
        if (debuggprint) {Serial.println("LOOP - fail to find network, waiting 10sec before retry");}
        delay(10000);
        now = millis();
      }

      if (modem.isNetworkConnected()) {
          if (debuggprint) {Serial.println("LOOP - Network re-connected");}
      }


      if (!modem.isGprsConnected()) {
            if (debuggprint) {Serial.println("GPRS disconnected!");
            Serial.print(F("Connecting to "));
            Serial.print(webAPN);}
            if (!modem.gprsConnect(webAPN.c_str(), webSIMUSER.c_str(), webSIMPASS.c_str())) {
                if (debuggprint) {Serial.println(" fail");}
                delay(10000);
                return;
            }
            if (modem.isGprsConnected()) {
                if (debuggprint) {Serial.println("GPRS reconnected");}
            }
        }
    }
  //DeepSleep configuration
    if ( lastState == 0 ) {
        if (debuggprint) {Serial.println("lastState == 0 Valued to " + String(now) );}
        lastState = now;
    }
    

    if ((ntripClient.connected() == true) && (transmitLocation == true) && (millis() - lastGGA > timeSendGGA))
  { if (debuggprint) {Serial.println("Push data gga to caster");}
    lastGGA = millis();
    pushGPGGA();}
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= FUNCTION -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


//Connect to NTRIP Caster. Return true is connection is successful.
bool beginClient()
{
  if (debuggprint) {
    Serial.print(F("Opening socket to "));
    Serial.println(webCaster);
  }

  if (ntripClient.connect(webCaster.c_str(), webPort) == false) //Attempt connection
  {
    if (debuggprint) {Serial.println(F("Connection to caster failed"));}
    return (false);
  }
  else
  { countRtcm++;
    if (debuggprint) {
      Serial.print(F("Connected to "));
      Serial.print(webCaster);
      Serial.print(F(" : "));
      Serial.println(webPort);

      Serial.print(F("Requesting NTRIP Data from mount point "));
      Serial.println(webMount.c_str());
    }

    // Set up the server request (GET)
    const int SERVER_BUFFER_SIZE = 512;
    char serverRequest[SERVER_BUFFER_SIZE];
    snprintf(serverRequest,
             SERVER_BUFFER_SIZE,
             "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP SparkFun u-blox Client v1.0\r\n",
             webMount.c_str());

    // Set up the credentials
    char credentials[512];
    if (strlen(webUser.c_str()) == 0)
    {
      strncpy(credentials, "Accept: */*\r\nConnection: close\r\n", sizeof(credentials));
    }
    else
    {
      //Pass base64 encoded user:pw
      char userCredentials[sizeof(webUser.c_str()) + sizeof(webPW.c_str()) + 2]; //The ':' takes up a spot
      snprintf(userCredentials, sizeof(userCredentials), "%s:%s", webUser.c_str(), webPW.c_str());

      if (debuggprint) {
        Serial.print(F("Sending credentials: "));
        Serial.println(userCredentials);
      }

  #if defined(ARDUINO_ARCH_ESP32)
      //Encode with ESP32 built-in library
      base64 b;
      String strEncodedCredentials = b.encode(userCredentials);
      char encodedCredentials[strEncodedCredentials.length() + 1];
      strEncodedCredentials.toCharArray(encodedCredentials, sizeof(encodedCredentials)); //Convert String to char array
  #else
      //Encode with nfriendly library
      int encodedLen = base64_enc_len(strlen(userCredentials));
      char encodedCredentials[encodedLen];                                         //Create array large enough to house encoded data
      base64_encode(encodedCredentials, userCredentials, strlen(userCredentials)); //Note: Input array is consumed
  #endif

      snprintf(credentials, sizeof(credentials), "Authorization: Basic %s\r\n", encodedCredentials);
    }

    // Add the encoded credentials to the server request
    strncat(serverRequest, credentials, SERVER_BUFFER_SIZE);
    strncat(serverRequest, "\r\n", SERVER_BUFFER_SIZE);

    if (debuggprint) {
      Serial.print(F("serverRequest size: "));
      Serial.print(strlen(serverRequest));
      Serial.print(F(" of "));
      Serial.print(sizeof(serverRequest));
      Serial.println(F(" bytes available"));

    // Send the server request
      Serial.println(F("Sending server request: "));
      Serial.println(serverRequest);
    }
    ntripClient.write((const uint8_t*)serverRequest, strlen(serverRequest));

    //Wait up to 5 seconds for response
    unsigned long startTime = millis();
    while (ntripClient.available() == 0)
    {
      if (millis() > (startTime + 5000))
      {
        if (debuggprint) {Serial.println(F("Caster timed out!"));}
        ntripClient.stop();
        return (false);
      }
      delay(10);
    }

    //Check reply
    int connectionResult = 0;
    char response[512];
    size_t responseSpot = 0;
    while (ntripClient.available()) // Read bytes from the caster and store them
    {
      if (responseSpot == sizeof(response) - 1) // Exit the loop if we get too much data
        break;

      response[responseSpot++] = ntripClient.read();

      if (connectionResult == 0) // Only print success/fail once
      {
        if (strstr(response, "200") != NULL) //Look for '200 OK'
        {
          connectionResult = 200;
        }
        if (strstr(response, "401") != NULL) //Look for '401 Unauthorized'
        {
          if (debuggprint) {Serial.println(F("Hey - your credentials look bad! Check your caster username and password."));}
          connectionResult = 401;
        }
      }
    }
    response[responseSpot] = '\0'; // NULL-terminate the response

    //Serial.print(F("Caster responded with: ")); Serial.println(response); // Uncomment this line to see the full response

    if (connectionResult != 200)
    {
      if (debuggprint) {
        Serial.print(F("Failed to connect to "));
        Serial.println(webCaster);
      }
      return (false);
    }
    else
    {
      if (debuggprint) {
        Serial.print(F("Connected to: "));
        Serial.println(webCaster);
      }
      lastReceivedRTCM_ms = millis(); //Reset timeout
    }
  }

  return (true);
} // /beginClient

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bool processConnection()
{
  if (ntripClient.connected() == true) // Check that the connection is still open
  {
    // uint8_t rtcmData[512 * 4]; //Most incoming data is around 500 bytes but may be larger
    uint8_t rtcmData[512 * 4]; //Most incoming data is around 500 bytes but may be larger
    size_t rtcmCount = 0;

    
    // Lire et transmettre les trames RTCM

    static enum {WAIT_SYNC, READ_LENGTH_1, READ_LENGTH_2, READ_PAYLOAD} stateRtcmTrame = WAIT_SYNC;
    static uint16_t length = 0;
    static uint16_t indexRtcmTrame = 0;
    static uint8_t bufferRtcmTrame[2056];  // > buffer des ports serial

    //Collect any available RTCM data
    while (ntripClient.available())
    {
      //=-=-=-=-=-=-=- analyse rtcm trame =-=-=-=-=
      uint8_t b = ntripClient.read();  //client.read();
      rtcmData[rtcmCount++] = b;
      totalData = totalData+1;

      switch (stateRtcmTrame) {
          case WAIT_SYNC:
            if (b == 0xD3) {          //  Format de base d'une trame RTCM 3.x
              bufferRtcmTrame[0] = b;
              indexRtcmTrame = 1;
              stateRtcmTrame = READ_LENGTH_1;
            }
            break;

          case READ_LENGTH_1:
            bufferRtcmTrame[indexRtcmTrame++] = b;
            length = (b & 0x03) << 8;  // only 2 LSBs used
            stateRtcmTrame = READ_LENGTH_2;
            break;

          case READ_LENGTH_2:
            bufferRtcmTrame[indexRtcmTrame++] = b;
            length |= b;
            length += 3; // Include the 3-byte CRC at the end
            stateRtcmTrame = READ_PAYLOAD;
            if (debuggprint && length > 1023) {Serial.println("Trame RTCM TROP TROP LONGUEEEEEEEE");}
            break;

          case READ_PAYLOAD:
            bufferRtcmTrame[indexRtcmTrame++] = b;
            if (indexRtcmTrame >= length + 3) {
              // Trame complète reçue !

              // Vérification du CRC
              uint32_t crc_calc = compute_crc24q(bufferRtcmTrame, indexRtcmTrame - 3);
              uint32_t crc_recv = ((uint32_t)bufferRtcmTrame[indexRtcmTrame - 3] << 16) |
                                  ((uint32_t)bufferRtcmTrame[indexRtcmTrame - 2] << 8) |
                                  ((uint32_t)bufferRtcmTrame[indexRtcmTrame - 1]);

              if (crc_calc == crc_recv) {
                timeStateCaster = millis();
                GNSSSerial.write(bufferRtcmTrame, indexRtcmTrame); // envoi RTCM sur pour serie GNSS
                parseRTCMMessage(bufferRtcmTrame, indexRtcmTrame);    // ajout pour compteur de trame RTCM
              } else {
                if (debuggprint) {Serial.println("Trame RTCM ignorée : CRC invalide");}
              }
              // Réinitialisation pour la prochaine trame
              indexRtcmTrame = 0;
              stateRtcmTrame = WAIT_SYNC;
            }
            break;
      }
    }

    if (rtcmCount > 0)
    {
      lastReceivedRTCM_ms = millis();
      if (debuggprint) {
        Serial.println();
        Serial.print(F("Pushed "));
        Serial.print(rtcmCount);
        Serial.println(F(" RTCM bytes to GNSS "));
      }
    }
    displayCounters();
  }
  else
  {
    if (debuggprint) {Serial.println(F("Connection dropped!"));}
    return (false); // Connection has dropped - return false
  }

  //Timeout if we don't have new data for maxTimeBeforeHangup_ms
  if ((millis() - lastReceivedRTCM_ms) > maxTimeBeforeHangup_ms)
  {
    if (debuggprint) {Serial.println(F("RTCM timeout!"));}
    return (false); // Connection has timed out - return false
  }

  return (true);
} // /processConnection

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void closeConnection()
{
  if (ntripClient.connected() == true)
  {
    ntripClient.stop();
  }
  if (debuggprint) {Serial.println(F("Disconnected!"));}
  ESP.restart(); //TODO:resolve, delay time, bug infinity reconnect ntrip if base RTK down
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Return true if a key has been pressed
bool keyPressed()
{
  if (Serial.available()) // Check for a new key press
  {
    delay(100); // Wait for any more keystrokes to arrive
    while (Serial.available()) // Empty the serial buffer
      Serial.read();
    return (true);
  }

  return (false);
}
