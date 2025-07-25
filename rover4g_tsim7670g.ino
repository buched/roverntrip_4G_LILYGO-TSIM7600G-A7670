// =============================================================================
// CONFIGURATION DU MODELE - Decommenter UNE SEULE ligne
// =============================================================================
#define MODEL_A7670G    // Pour LilyGO T-A7670G avec GPS externe. https://lilygo.cc/products/t-sim-a7670e?variant=42737494458549

// =============================================================================
// CONFIGURATION AUTOMATIQUE SELON LE MODELE
// =============================================================================
#ifdef MODEL_A7670G
  #define TINY_GSM_MODEM_A7670
  #define USE_EXTERNAL_GPS 1
  #define BOARD_NAME "TSIM_A7670G"
#elif defined(MODEL_A7670E)
  #define TINY_GSM_MODEM_A7670
  #define USE_EXTERNAL_GPS 1
  #define BOARD_NAME "TSIM_A7670E"
#else
  #error "Aucun modele selectionne! Decommenter un #define MODEL_xxx"
#endif

#ifndef NTRIP_CLIENT_H
#define NTRIP_CLIENT_H

#include "TinyGsmClient.h"

class NTRIPClient {
  public:
    NTRIPClient(TinyGsmClient &client);

    bool reqSrcTbl(char *host, int &port);
    bool reqRaw(char *host, int &port, char *mntpnt, char *user, char *psw);
    void stop();
    int available();
    int read();  // ⚠️ Ici on met bien int
    int readLine(char *buffer, int maxlen);
    void sendGGA(const String &gga);
    
  private:
    TinyGsmClient *_client;
};

#endif

#include <HardwareSerial.h>
#include <BluetoothSerial.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_task_wdt.h"

// =============================================================================
// PINOUT SPECIFIQUE PAR MODELE
// =============================================================================
#ifdef MODEL_SIM7600
  // Configuration des broches pour modele original SIM7600
  #define MODEM_RX 26
  #define MODEM_TX 27
  #define MODEM_PWRKEY 4
  #define MODEM_POWER 25
  #define bauds 460800
  #define GNSS_RX 33
  #define GNSS_TX 32
  #define MODE_PIN1 13
  #define MODE_PIN2 15
  #define CONFIG_PIN 14  // Pin pour forcer le mode configuration
#elif defined(MODEL_A7670G)
  // Configuration des broches pour LilyGO T-A7670G
  #define MODEM_RX 27
  #define MODEM_TX 26
  #define MODEM_PWRKEY 4
  #define MODEM_DTR 25
  #define MODEM_RI 33
  #define BOARD_POWERON 12
  #define bauds 460800
  #define GNSS_RX 34
  #define GNSS_TX 35
  #define MODE_PIN1 13
  #define MODE_PIN2 15
  #define CONFIG_PIN 14  // Pin pour forcer le mode configuration
#elif defined(MODEL_A7670E)
  // Configuration des broches pour LilyGO T-A7670E
  #define MODEM_RX 27
  #define MODEM_TX 26
  #define MODEM_PWRKEY 4
  #define MODEM_DTR 25
  #define MODEM_RI 33
  #define BOARD_POWERON 12
  #define bauds 460800
  #define GNSS_RX 34
  #define GNSS_TX 35
  #define MODE_PIN1 14
  #define MODE_PIN2 15
  #define CONFIG_PIN 13  // Pin pour forcer le mode configuration
#endif

// Configuration WiFi
String wifi_ssid = "";      // Sera charge depuis les preferences
String wifi_password = "";  // Sera charge depuis les preferences
const char* udpAddress = "192.168.1.255";
const int udpPort = 9999;

// Point d'acces de configuration
const char* ap_ssid = "rover-gnss-config";
const char* ap_password = "12345678";
WebServer server(80);
Preferences preferences;
bool configMode = false;

// Configuration des buffers
#define GNSS_BUFFER_SIZE 2048
#define BT_BUFFER_SIZE 1024
#define QUEUE_SIZE 50

// Modes de communication
enum CommMode {
  MODE_BLUETOOTH = 0,
  MODE_BLE = 1,
  MODE_UDP = 2
};

volatile CommMode currentMode = MODE_UDP; // Par defaut, sera redefini au boot

// Configuration du modem
const char apn[] = "sl2sfr";
const char user[] = "";
const char pass[] = "";

// Variables pour la configuration NTRIP configurables
String ntrip_host = "crtk.net";       // Adresse du caster par defaut
int ntrip_port = 2101;                // Port par defaut
String ntrip_mountpoint = "LRSEC";    // Valeur par defaut
String ntrip_user = "centipede";      // Valeur par defaut
String ntrip_pass = "centipede";      // Valeur par defaut

// Buffers char pour la compatibilite avec NTRIPClient
char ntrip_host_buf[128];
char ntrip_mountpoint_buf[64];
char ntrip_user_buf[64];
char ntrip_pass_buf[64];

// Initialisation des objets
HardwareSerial SerialAT(1);
#if USE_EXTERNAL_GPS
HardwareSerial GNSS(2);  // GPS externe
#endif
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
NTRIPClient ntrip_c(gsmClient);

// Objets de communication
BluetoothSerial SerialBT;
BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
WiFiUDP udp;
bool deviceConnected = false;

// Buffer circulaire pour les donnees GNSS
struct CircularBuffer {
  char data[GNSS_BUFFER_SIZE];
  volatile size_t head;
  volatile size_t tail;
  volatile size_t count;
};

CircularBuffer gnssBuffer = {0};

// Queue pour les messages NMEA complets
QueueHandle_t nmeaQueue;

// Structure pour les messages NMEA
struct NMEAMessage {
  char data[256];
  size_t length;
};

// Handles des taches
TaskHandle_t xRTCMTaskHandle = NULL;
TaskHandle_t xGNSSTaskHandle = NULL;
TaskHandle_t xCommunicationTaskHandle = NULL;

// BLE callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Client BLE connecte");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Client BLE deconnecte");
      pServer->startAdvertising(); // Redemarrer l'advertising
    }
};

// Fonctions pour le buffer circulaire
bool bufferPut(CircularBuffer* buf, char c) {
  if (buf->count >= GNSS_BUFFER_SIZE) {
    return false; // Buffer plein
  }
  
  buf->data[buf->head] = c;
  buf->head = (buf->head + 1) % GNSS_BUFFER_SIZE;
  buf->count++;
  return true;
}

bool bufferGet(CircularBuffer* buf, char* c) {
  if (buf->count == 0) {
    return false; // Buffer vide
  }
  
  *c = buf->data[buf->tail];
  buf->tail = (buf->tail + 1) % GNSS_BUFFER_SIZE;
  buf->count--;
  return true;
}

size_t bufferAvailable(CircularBuffer* buf) {
  return buf->count;
}

// Fonction pour lire le mode au boot
CommMode readBootMode() {
  pinMode(MODE_PIN1, INPUT_PULLUP);
  pinMode(MODE_PIN2, INPUT_PULLUP);
  
  // Petit delai pour stabiliser les lectures
  delay(10);
  
  bool pin1 = digitalRead(MODE_PIN1);
  bool pin2 = digitalRead(MODE_PIN2);
  
  // Configuration des modes :
  // PIN1=HIGH, PIN2=HIGH -> MODE_UDP (00)
  // PIN1=LOW,  PIN2=HIGH -> MODE_BLUETOOTH (01) 
  // PIN1=HIGH, PIN2=LOW  -> MODE_BLE (10)
  // PIN1=LOW,  PIN2=LOW  -> MODE_UDP (11) - reserve pour extension
  
  if (!pin1 && pin2) {
    Serial.println("Mode selectionne: BLUETOOTH (PIN1=LOW, PIN2=HIGH)");
    return MODE_BLUETOOTH;
  }
  else if (pin1 && !pin2) {
    Serial.println("Mode selectionne: BLE (PIN1=HIGH, PIN2=LOW)");
    return MODE_BLE;
  }
  else {
    Serial.println("Mode selectionne: UDP (defaut)");
    return MODE_UDP;
  }
}

// Fonction pour verifier si le mode configuration est force
bool isConfigModeForced() {
  pinMode(CONFIG_PIN, INPUT_PULLUP);
  delay(10); // Stabiliser la lecture
  
  bool configForced = (digitalRead(CONFIG_PIN) == LOW);
  
  if (configForced) {
    Serial.println("MODE CONFIGURATION FORCE via CONFIG_PIN (LOW)");
    Serial.println("Demarrage du portail de configuration meme si WiFi configure");
  }
  
  return configForced;
}

// =============================================================================
// FONCTION DE DEMARRAGE MODEM SPECIFIQUE PAR MODELE
// =============================================================================
void powerOnModem() {
  Serial.printf("Demarrage modem %s...\n", BOARD_NAME);
  
#ifdef MODEL_SIM7600
  // Sequence pour SIM7600
  pinMode(MODEM_PWRKEY, OUTPUT);
  pinMode(MODEM_POWER, OUTPUT);
  digitalWrite(MODEM_POWER, HIGH);
  delay(100);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(500);
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(5000);
  
#elif defined(MODEL_A7670G)
  // Sequence pour A7670G avec GPIO externe
  pinMode(BOARD_POWERON, OUTPUT);
  digitalWrite(BOARD_POWERON, HIGH); // TRES IMPORTANT pour A7670G
  
  pinMode(MODEM_PWRKEY, OUTPUT);
  pinMode(MODEM_DTR, OUTPUT);
  
  digitalWrite(MODEM_DTR, LOW);
  delay(100);
  
  // Power key sequence A7670G
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(100);
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(1000);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(5000);
  
#elif defined(MODEL_A7670E)
  // Sequence pour A7670E avec GPS integre
  pinMode(BOARD_POWERON, OUTPUT);
  digitalWrite(BOARD_POWERON, HIGH); // TRES IMPORTANT pour A7670E
  
  pinMode(MODEM_PWRKEY, OUTPUT);
  pinMode(MODEM_DTR, OUTPUT);
  
  digitalWrite(MODEM_DTR, LOW);
  delay(100);
  
  // Power key sequence A7670E
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(100);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(1000);
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(5000);
#endif

  Serial.printf("Modem %s demarre\n", BOARD_NAME);
}

void initBluetooth() {
  if (!SerialBT.begin("rover-gnss")) {
    Serial.println("Erreur initialisation Bluetooth");
  } else {
    Serial.println("Bluetooth initialise: 'rover-gnss'");
  }
}

void initBLE() {
  BLEDevice::init("rover-gnss-ble");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService("12345678-1234-1234-1234-123456789abc");
  
  pCharacteristic = pService->createCharacteristic(
                      "87654321-4321-4321-4321-cba987654321",
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID("12345678-1234-1234-1234-123456789abc");
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  
  Serial.println("BLE initialise: 'rover-gnss-ble'");
}

void loadWiFiCredentials() {
  preferences.begin("wifi", false);
  wifi_ssid = preferences.getString("ssid", "");
  wifi_password = preferences.getString("password", "");
  preferences.end();
  
  if (wifi_ssid.length() > 0) {
    Serial.printf("WiFi sauvegarde trouve: %s\n", wifi_ssid.c_str());
  } else {
    Serial.println("Aucun WiFi sauvegarde");
  }
}

void saveWiFiCredentials(String ssid, String password) {
  preferences.begin("wifi", false);
  preferences.putString("ssid", ssid);
  preferences.putString("password", password);
  preferences.end();
  
  wifi_ssid = ssid;
  wifi_password = password;
  Serial.printf("WiFi sauvegarde: %s\n", ssid.c_str());
}

// Nouvelles fonctions pour la configuration NTRIP
void loadNTRIPConfig() {
  preferences.begin("ntrip", false);
  ntrip_host = preferences.getString("host", "crtk.net");
  ntrip_port = preferences.getInt("port", 2101);
  ntrip_mountpoint = preferences.getString("mountpoint", "LRSEC");
  ntrip_user = preferences.getString("user", "centipede");
  ntrip_pass = preferences.getString("pass", "centipede");
  preferences.end();
  
  // Copier vers les buffers char
  strncpy(ntrip_host_buf, ntrip_host.c_str(), sizeof(ntrip_host_buf) - 1);
  strncpy(ntrip_mountpoint_buf, ntrip_mountpoint.c_str(), sizeof(ntrip_mountpoint_buf) - 1);
  strncpy(ntrip_user_buf, ntrip_user.c_str(), sizeof(ntrip_user_buf) - 1);
  strncpy(ntrip_pass_buf, ntrip_pass.c_str(), sizeof(ntrip_pass_buf) - 1);
  ntrip_host_buf[sizeof(ntrip_host_buf) - 1] = '\0';
  ntrip_mountpoint_buf[sizeof(ntrip_mountpoint_buf) - 1] = '\0';
  ntrip_user_buf[sizeof(ntrip_user_buf) - 1] = '\0';
  ntrip_pass_buf[sizeof(ntrip_pass_buf) - 1] = '\0';
  
  Serial.printf("Configuration NTRIP chargee: %s@%s:%d\n", ntrip_mountpoint.c_str(), ntrip_host.c_str(), ntrip_port);
}

void saveNTRIPConfig(String host, int port, String mountpoint, String user, String pass) {
  preferences.begin("ntrip", false);
  preferences.putString("host", host);
  preferences.putInt("port", port);
  preferences.putString("mountpoint", mountpoint);
  preferences.putString("user", user);
  preferences.putString("pass", pass);
  preferences.end();
  
  ntrip_host = host;
  ntrip_port = port;
  ntrip_mountpoint = mountpoint;
  ntrip_user = user;
  ntrip_pass = pass;
  
  // Copier vers les buffers char
  strncpy(ntrip_host_buf, host.c_str(), sizeof(ntrip_host_buf) - 1);
  strncpy(ntrip_mountpoint_buf, mountpoint.c_str(), sizeof(ntrip_mountpoint_buf) - 1);
  strncpy(ntrip_user_buf, user.c_str(), sizeof(ntrip_user_buf) - 1);
  strncpy(ntrip_pass_buf, pass.c_str(), sizeof(ntrip_pass_buf) - 1);
  ntrip_host_buf[sizeof(ntrip_host_buf) - 1] = '\0';
  ntrip_mountpoint_buf[sizeof(ntrip_mountpoint_buf) - 1] = '\0';
  ntrip_user_buf[sizeof(ntrip_user_buf) - 1] = '\0';
  ntrip_pass_buf[sizeof(ntrip_pass_buf) - 1] = '\0';
  
  Serial.printf("Configuration NTRIP sauvegardee: %s@%s:%d\n", mountpoint.c_str(), host.c_str(), port);
}

void handleRoot() {
  String html = R"html(
<!DOCTYPE html>
<html>
<head>
    <title>Configuration - Rover GNSS</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <meta http-equiv="Content-Type" content="text/html; charset=utf-8">
    <style>
        body { font-family: Arial; margin: 20px; background: #f0f0f0; }
        .container { background: white; padding: 30px; border-radius: 10px; max-width: 500px; margin: 0 auto; }
        h1 { color: #333; text-align: center; margin-bottom: 30px; }
        .section { margin: 30px 0; padding: 20px; background: #f8f9fa; border-radius: 8px; }
        .section h2 { color: #007cba; margin-top: 0; margin-bottom: 15px; font-size: 18px; }
        input[type="text"], input[type="password"] { 
            width: 100%; padding: 10px; margin: 10px 0; border: 1px solid #ddd; 
            border-radius: 5px; box-sizing: border-box; 
        }
        label { display: block; margin-top: 15px; margin-bottom: 5px; font-weight: bold; color: #333; }
        button { 
            background: #007cba; color: white; padding: 15px 30px; border: none; 
            border-radius: 5px; cursor: pointer; width: 100%; font-size: 16px; margin-top: 15px;
        }
        button:hover { background: #005a87; }
        button.secondary { background: #6c757d; margin-top: 10px; }
        button.secondary:hover { background: #545b62; }
        .info { background: #e7f3ff; padding: 15px; border-radius: 5px; margin: 20px 0; }
        .networks { margin: 15px 0; }
        .network { 
            background: #ffffff; padding: 12px; margin: 5px 0; border-radius: 5px; 
            cursor: pointer; border: 1px solid #ddd;
        }
        .network:hover { background: #e9ecef; }
        .current-config { background: #d4edda; padding: 15px; border-radius: 5px; margin: 15px 0; }
        .current-config h3 { margin-top: 0; color: #155724; }
    </style>
</head>
<body>
    <div class="container">
        <h1>ROVER GNSS )html" + String(BOARD_NAME) + R"html(</h1>
        
        <div class="current-config">
            <h3>Configuration actuelle</h3>
            <strong>WiFi:</strong> )html" + (wifi_ssid.length() > 0 ? wifi_ssid : "Non configuré") + R"html(<br>
            <strong>NTRIP:</strong> )html" + ntrip_mountpoint + R"html(@)html" + ntrip_host + R"html(:)html" + String(ntrip_port) + R"html( (utilisateur: )html" + ntrip_user + R"html()<br>
            <strong>Mode forcé:</strong> )html" + (isConfigModeForced() ? "OUI (CONFIG_PIN actif)" : "NON") + R"html(
        </div>
        
        <div class="section">
            <h2>📶 Configuration WiFi</h2>
            <div class="info">
                Connectez-vous à votre réseau WiFi pour diffuser les données NMEA en UDP.<br>
                <strong>Astuce:</strong> Pour accéder à cette configuration à tout moment, mettez le pin CONFIG_PIN (pin 13) à la masse au démarrage.
            </div>
            
            <div class="networks" id="networks">
                <button onclick="scanNetworks()">Scanner les réseaux</button>
            </div>
            
            <form action="/save" method="POST">
                <label>Nom du réseau (SSID):</label>
                <input type="text" name="ssid" id="ssid" placeholder="Nom du WiFi" value=")html" + wifi_ssid + R"html(" required>
                
                <label>Mot de passe:</label>
                <input type="password" name="password" id="password" placeholder="Mot de passe WiFi" required>
                
                <div class="section">
                    <h2>📡 Configuration NTRIP</h2>
                    <div class="info">
                        Configurez votre serveur de corrections NTRIP (caster) pour les corrections RTK.<br>
                        <strong>Exemples:</strong> crtk.net, rtk2go.com, igs-ip.net, etc.
                    </div>
                    
                    <label>Adresse du serveur NTRIP (Caster):</label>
                    <input type="text" name="ntrip_host" id="ntrip_host" placeholder="Ex: crtk.net" value=")html" + ntrip_host + R"html(" required>
                    
                    <label>Port du serveur NTRIP:</label>
                    <input type="number" name="ntrip_port" id="ntrip_port" placeholder="2101" value=")html" + String(ntrip_port) + R"html(" min="1" max="65535" required>
                    
                    <label>Point de montage (Mountpoint):</label>
                    <input type="text" name="mountpoint" id="mountpoint" placeholder="Ex: LRSEC" value=")html" + ntrip_mountpoint + R"html(" required>
                    
                    <label>Nom d'utilisateur NTRIP:</label>
                    <input type="text" name="ntrip_user" id="ntrip_user" placeholder="Ex: centipede" value=")html" + ntrip_user + R"html(" required>
                    
                    <label>Mot de passe NTRIP:</label>
                    <input type="password" name="ntrip_pass" id="ntrip_pass" placeholder="Mot de passe NTRIP" value=")html" + ntrip_pass + R"html(" required>
                </div>
                
                <button type="submit">Sauvegarder et Redémarrer</button>
            </form>
        </div>
    </div>
    
    <script>
        function selectNetwork(ssid) {
            document.getElementById('ssid').value = ssid;
        }
        
        function scanNetworks() {
            document.getElementById('networks').innerHTML = '<div>Scan en cours...</div>';
            fetch('/scan')
                .then(response => response.json())
                .then(data => {
                    let html = '<button onclick="scanNetworks()">Re-scanner</button>';
                    data.forEach(network => {
                        html += `<div class="network" onclick="selectNetwork('${network.ssid}')">${network.ssid} (${network.rssi} dBm)</div>`;
                    });
                    document.getElementById('networks').innerHTML = html;
                })
                .catch(error => {
                    document.getElementById('networks').innerHTML = '<div>Erreur lors du scan</div>';
                });
        }
    </script>
</body>
</html>
)html";
  
  server.send(200, "text/html", html);
}

void handleScan() {
  int n = WiFi.scanNetworks();
  String json = "[";
  
  for (int i = 0; i < n; i++) {
    if (i > 0) json += ",";
    json += "{\"ssid\":\"" + WiFi.SSID(i) + "\",\"rssi\":" + String(WiFi.RSSI(i)) + "}";
  }
  json += "]";
  
  server.send(200, "application/json", json);
}

void handleSave() {
  if (server.hasArg("ssid") && server.hasArg("password") && 
      server.hasArg("ntrip_host") && server.hasArg("ntrip_port") &&
      server.hasArg("mountpoint") && server.hasArg("ntrip_user") && server.hasArg("ntrip_pass")) {
    
    String ssid = server.arg("ssid");
    String password = server.arg("password");
    String ntripHost = server.arg("ntrip_host");
    int ntripPort = server.arg("ntrip_port").toInt();
    String mountpoint = server.arg("mountpoint");
    String ntripUser = server.arg("ntrip_user");
    String ntripPass = server.arg("ntrip_pass");
    
    // Validation du port
    if (ntripPort < 1 || ntripPort > 65535) {
      ntripPort = 2101; // Valeur par défaut si invalide
    }
    
    // Sauvegarder les configurations
    saveWiFiCredentials(ssid, password);
    saveNTRIPConfig(ntripHost, ntripPort, mountpoint, ntripUser, ntripPass);
    
    String html = R"html(
<!DOCTYPE html>
<html>
<head>
    <title>Configuration sauvegardée</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">    
    <meta http-equiv="Content-Type" content="text/html; charset=utf-8">
    <style>
        body { font-family: Arial; margin: 40px; background: #f0f0f0; text-align: center; }
        .container { background: white; padding: 30px; border-radius: 10px; max-width: 400px; margin: 0 auto; }
        .success { color: #28a745; }
        .config-summary { background: #f8f9fa; padding: 15px; border-radius: 5px; margin: 20px 0; text-align: left; }
    </style>
</head>
<body>
    <div class="container">
        <h1 class="success">Configuration sauvegardée!</h1>
        <div class="config-summary">
            <strong>WiFi:</strong> )html" + ssid + R"html(<br>
            <strong>Serveur NTRIP:</strong> )html" + ntripHost + R"html(:)html" + String(ntripPort) + R"html(<br>
            <strong>Mountpoint:</strong> )html" + mountpoint + R"html(<br>
            <strong>Utilisateur NTRIP:</strong> )html" + ntripUser + R"html(
        </div>
        <p>Le rover va redémarrer et se connecter avec la nouvelle configuration.</p>
        <p>Redémarrage dans 5 secondes...</p>
    </div>
    <script>
        setTimeout(() => {
            window.location.href = '/';
        }, 5000);
    </script>
</body>
</html>
)html";
    
    server.send(200, "text/html", html);
    
    delay(3000);
    ESP.restart();
  } else {
    server.send(400, "text/plain", "Paramètres manquants");
  }
}

void startConfigPortal() {
  configMode = true;
  Serial.println("Demarrage du portail de configuration WiFi");
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  
  Serial.printf("Point d'acces cree: %s\n", ap_ssid);
  Serial.printf("Mot de passe: %s\n", ap_password);
  Serial.printf("Adresse IP: %s\n", WiFi.softAPIP().toString().c_str());
  Serial.println("Connectez-vous au WiFi 'rover-gnss-config' et allez sur http://192.168.4.1");
  
  server.on("/", handleRoot);
  server.on("/scan", handleScan);
  server.on("/save", HTTP_POST, handleSave);
  server.begin();
}

void initWiFi() {
  loadWiFiCredentials();
  
  // Verifier si le mode configuration est force par le pin
  bool configForced = isConfigModeForced();
  
  if (wifi_ssid.length() == 0 || configForced) {
    if (configForced) {
      Serial.println("Mode configuration force - Demarrage du portail de configuration");
    } else {
      Serial.println("Aucun WiFi configure - Demarrage du portail de configuration");
    }
    startConfigPortal();
    return;
  }
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());
  Serial.printf("Connexion WiFi: %s", wifi_ssid.c_str());
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("WiFi connecte: ");
    Serial.println(WiFi.localIP());
    udp.begin(udpPort);
    Serial.printf("UDP broadcaster sur port %d\n", udpPort);
  } else {
    Serial.println("\nEchec connexion WiFi - Demarrage du portail de configuration");
    startConfigPortal();
  }
}

void initSelectedMode() {
  switch (currentMode) {
    case MODE_BLUETOOTH:
      Serial.println("Initialisation Bluetooth");
      initBluetooth();
      break;
    case MODE_BLE:
      Serial.println("Initialisation BLE");
      initBLE();
      break;
    case MODE_UDP:
      Serial.println("Initialisation UDP/WiFi");
      initWiFi();
      break;
  }
}

// =============================================================================
// TACHES RTCM ET GNSS SPECIFIQUES PAR MODELE
// =============================================================================
void handleRTCMTask(void *pvParameters) {
  char buffer[512];
  size_t bytesRead;
  
  while (1) {
    if (ntrip_c.available()) {
      // Lire par blocs pour reduire les appels systeme
      bytesRead = 0;
      while (ntrip_c.available() && bytesRead < sizeof(buffer)) {
        buffer[bytesRead++] = ntrip_c.read();
      }
      
      if (bytesRead > 0) {
#if USE_EXTERNAL_GPS
        // Ecrire les corrections RTCM au GPS externe
        GNSS.write((uint8_t*)buffer, bytesRead);
#else
        // Envoyer les corrections RTCM au modem A7670E via AT+CGPSRXD
        String rtcmCmd = "AT+CGPSRXD=";
        for (size_t i = 0; i < bytesRead; i++) {
          if (buffer[i] < 16) rtcmCmd += "0";
          rtcmCmd += String(buffer[i], HEX);
        }
        modem.sendAT(rtcmCmd);
        modem.waitResponse();
#endif
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5)); // 5ms delay
  }
}

void handleGNSSTask(void *pvParameters) {
#if USE_EXTERNAL_GPS
  // Version GPS externe (SIM7600 et A7670G)
  char nmeaBuffer[256];
  size_t nmeaIndex = 0;
  char c;
  
  while (1) {
    // Lire toutes les donnees disponibles du GPS externe
    while (GNSS.available()) {
      c = GNSS.read();
      bufferPut(&gnssBuffer, c);
    }
    
    // Traiter les donnees du buffer circulaire
    while (bufferGet(&gnssBuffer, &c)) {
      if (c == '$' || c == '!') {
        // Debut d'une nouvelle trame NMEA
        nmeaIndex = 0;
        nmeaBuffer[nmeaIndex++] = c;
      }
      else if (c == '\n' && nmeaIndex > 0) {
        // Fin de trame NMEA
        nmeaBuffer[nmeaIndex++] = c;
        nmeaBuffer[nmeaIndex] = '\0';
        
        // Envoyer le message complet a la queue
        NMEAMessage msg;
        msg.length = nmeaIndex;
        memcpy(msg.data, nmeaBuffer, nmeaIndex + 1);
        
        // Envoi non-bloquant vers la queue
        if (xQueueSend(nmeaQueue, &msg, 0) != pdTRUE) {
          Serial.println("Queue NMEA pleine");
        }
        
        nmeaIndex = 0;
      }
      else if (nmeaIndex < sizeof(nmeaBuffer) - 2) {
        nmeaBuffer[nmeaIndex++] = c;
      }
      else {
        // Buffer overflow, recommencer
        nmeaIndex = 0;
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(1)); // 1ms delay
  }
  
#else
  // Version GPS integre A7670E
  String gpsData;
  
  while (1) {
    // Demander les donnees GPS au modem A7670E
    if (modem.sendAT("+CGPSINFO")) {
      if (modem.waitResponse(1000L, gpsData) == 1) {
        if (gpsData.indexOf("+CGPSINFO:") != -1) {
          // Extraire les donnees NMEA
          int start = gpsData.indexOf(":") + 1;
          String nmea = gpsData.substring(start);
          nmea.trim();
          
          if (nmea.length() > 0 && nmea != ",,,,,,,,") {
            // Convertir en format NMEA standard
            String gga = "$GPGGA," + nmea + "*00\r\n";
            
            // Traiter comme une trame NMEA complete
            NMEAMessage msg;
            msg.length = gga.length();
            strcpy(msg.data, gga.c_str());
            
            // Envoi non-bloquant vers la queue
            if (xQueueSend(nmeaQueue, &msg, 0) != pdTRUE) {
              Serial.println("Queue NMEA pleine");
            }
          }
        }
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000)); // 1 seconde entre les requetes GPS
  }
#endif
}

void handleCommunicationTask(void *pvParameters) {
  NMEAMessage msg;
  char commBuffer[BT_BUFFER_SIZE];
  size_t commBufferIndex = 0;
  TickType_t lastFlush = xTaskGetTickCount();
  
  while (1) {
    // Recevoir les messages de la queue
    while (xQueueReceive(nmeaQueue, &msg, pdMS_TO_TICKS(1)) == pdTRUE) {
      switch (currentMode) {
        case MODE_BLUETOOTH:
          // Buffering pour Bluetooth
          if (commBufferIndex + msg.length < BT_BUFFER_SIZE) {
            memcpy(&commBuffer[commBufferIndex], msg.data, msg.length);
            commBufferIndex += msg.length;
          } else {
            if (commBufferIndex > 0) {
              SerialBT.write((uint8_t*)commBuffer, commBufferIndex);
              commBufferIndex = 0;
            }
            if (msg.length < BT_BUFFER_SIZE) {
              memcpy(commBuffer, msg.data, msg.length);
              commBufferIndex = msg.length;
            }
          }
          break;
          
        case MODE_BLE:
          // Envoi direct pour BLE (limite de 20 octets par notification)
          if (deviceConnected && pCharacteristic) {
            // Decouper en chunks de 20 octets max
            size_t offset = 0;
            while (offset < msg.length) {
              size_t chunkSize = min((size_t)20, msg.length - offset);
              pCharacteristic->setValue((uint8_t*)&msg.data[offset], chunkSize);
              pCharacteristic->notify();
              offset += chunkSize;
              vTaskDelay(pdMS_TO_TICKS(1)); // Petit delai entre chunks
            }
          }
          break;
          
        case MODE_UDP:
          // Envoi direct UDP (seulement si connecte et pas en mode config)
          if (!configMode && WiFi.status() == WL_CONNECTED) {
            udp.beginPacket(udpAddress, udpPort);
            udp.write((uint8_t*)msg.data, msg.length);
            udp.endPacket();
          }
          break;
      }
    }
    
    // Flush periodique pour Bluetooth
    if (currentMode == MODE_BLUETOOTH) {
      TickType_t currentTime = xTaskGetTickCount();
      if (commBufferIndex > 0 && 
          (commBufferIndex > BT_BUFFER_SIZE / 2 || 
           (currentTime - lastFlush) > pdMS_TO_TICKS(50))) {
        
        SerialBT.write((uint8_t*)commBuffer, commBufferIndex);
        commBufferIndex = 0;
        lastFlush = currentTime;
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(5)); // 5ms delay - plus reactif
    
    // Gerer le serveur web en mode configuration
    if (configMode) {
      server.handleClient();
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  
  Serial.printf("=== ROVER GNSS %s ===\n", BOARD_NAME);
  Serial.printf("GPS: %s\n", USE_EXTERNAL_GPS ? "Externe" : "Integre");
  Serial.printf("CONFIG_PIN: %d (LOW=force config mode)\n", CONFIG_PIN);
  
  // Verifier en premier si le mode configuration est force
  if (isConfigModeForced()) {
    Serial.println("=== MODE CONFIGURATION FORCE ===");
    Serial.println("Demarrage direct du portail de configuration...");
    startConfigPortal();
    
    // En mode configuration force, on ne demarre pas le modem
    // On reste uniquement en mode portail de configuration
    while (true) {
      server.handleClient();
      delay(10);
    }
  }
  
  // Charger la configuration NTRIP sauvegardee
  loadNTRIPConfig();
  
  // Configuration du modem selon le modele
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(1500);
  
  powerOnModem();
  delay(3000);

  Serial.printf("Initialisation modem %s...\n", BOARD_NAME);
  if (!modem.restart()) {
    Serial.println("modem.restart() echoue");
    while (true);
  }

  Serial.println("Connexion GPRS...");
  if (!modem.gprsConnect(apn, user, pass)) {
    Serial.println("GPRS echoue");
    while (true);
  }

  Serial.println("GPRS connecte");
  Serial.print("IP : ");
  Serial.println(modem.localIP());

  Serial.printf("Connexion au MountPoint %s@%s:%d...\n", ntrip_mountpoint.c_str(), ntrip_host.c_str(), ntrip_port);
  // Utiliser les buffers char au lieu des String.c_str()
  if (!ntrip_c.reqRaw(ntrip_host_buf, ntrip_port, ntrip_mountpoint_buf, ntrip_user_buf, ntrip_pass_buf)) {
    Serial.println("Echec connexion NTRIP - redemarrage dans 15s");
    delay(15000);
    ESP.restart();
  }

  Serial.printf("Connecte au MountPoint %s@%s:%d\n", ntrip_mountpoint.c_str(), ntrip_host.c_str(), ntrip_port);
  delay(500);

  // Configuration GPS selon le modele
#if USE_EXTERNAL_GPS
  Serial.println("Configuration GPS externe...");
  GNSS.begin(bauds, SERIAL_8N1, GNSS_RX, GNSS_TX);
  GNSS.setRxBufferSize(2048); // Augmenter le buffer RX
  delay(500);
#else
  Serial.println("Activation GPS integre A7670E...");
  modem.sendAT("+CGPS=1,1"); // Demarrer GPS avec mode NMEA
  modem.waitResponse();
  delay(1000);
#endif

  // Lire le mode de communication depuis les pins
  currentMode = readBootMode();

  // Initialiser le mode selectionne
  initSelectedMode();
  
  // Creer la queue pour les messages NMEA
  nmeaQueue = xQueueCreate(QUEUE_SIZE, sizeof(NMEAMessage));
  if (nmeaQueue == NULL) {
    Serial.println("Echec creation queue NMEA");
    while (true);
  }

  delay(500);

  // Desactiver le watchdog pour les taches
  esp_task_wdt_deinit();

  // Creer les taches FreeRTOS avec priorites optimisees
  xTaskCreatePinnedToCore(handleRTCMTask, "RTCM Task", 8192, NULL, 3, &xRTCMTaskHandle, 0);
  xTaskCreatePinnedToCore(handleGNSSTask, "GNSS Task", 8192, NULL, 2, &xGNSSTaskHandle, 1);
  xTaskCreatePinnedToCore(handleCommunicationTask, "Communication Task", 8192, NULL, 1, &xCommunicationTaskHandle, 1);
  
  Serial.printf("=== ROVER %s PRET ===\n", BOARD_NAME);
  Serial.printf("Configuration NTRIP: %s@%s:%d (utilisateur: %s)\n", 
                ntrip_mountpoint.c_str(), ntrip_host.c_str(), ntrip_port, ntrip_user.c_str());
}

void loop() {
  // Surveillance optionnelle des performances
  static unsigned long lastStats = 0;
  if (millis() - lastStats > 30000) { // Toutes les 30 secondes
    const char* modeNames[] = {"Bluetooth", "BLE", "UDP"};
    
    if (configMode) {
      Serial.printf("Mode: CONFIG (Portail WiFi actif), Queue: %d/%d\n",
                    uxQueueMessagesWaiting(nmeaQueue), QUEUE_SIZE);
      Serial.printf("Connectez-vous au WiFi '%s' et allez sur http://192.168.4.1\n", ap_ssid);
    } else {
      Serial.printf("Mode: %s (fixe), Model: %s, GPS: %s\n", 
                    modeNames[currentMode], 
                    BOARD_NAME,
                    USE_EXTERNAL_GPS ? "Externe" : "Integre");
      
      Serial.printf("NTRIP: %s@%s:%d (%s)\n", 
                    ntrip_mountpoint.c_str(), ntrip_host.c_str(), ntrip_port, ntrip_user.c_str());
                    
#if USE_EXTERNAL_GPS
      Serial.printf("Buffer GNSS: %d/%d, Queue: %d/%d\n",
                    bufferAvailable(&gnssBuffer), GNSS_BUFFER_SIZE,
                    uxQueueMessagesWaiting(nmeaQueue), QUEUE_SIZE);
#else
      Serial.printf("Queue: %d/%d\n",
                    uxQueueMessagesWaiting(nmeaQueue), QUEUE_SIZE);
#endif
      
      if (currentMode == MODE_UDP && WiFi.status() == WL_CONNECTED) {
        Serial.printf("WiFi IP: %s\n", WiFi.localIP().toString().c_str());
      }
    }
    
    lastStats = millis();
  }
  
  delay(1000);
}