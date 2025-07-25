#include "NTRIPClient.h"
#include <base64.h>

NTRIPClient::NTRIPClient(TinyGsmClient &client) {
  _client = &client;
}

bool NTRIPClient::reqSrcTbl(char *host, int &port) {
  if (!_client->connect(host, port)) return false;

  _client->print("GET / HTTP/1.0\r\nHost: ");
  _client->print(host);
  _client->print("\r\nUser-Agent: NTRIP TinyGSMClient\r\n\r\n");
  return true;
}

bool NTRIPClient::reqRaw(char *host, int &port, char *mntpnt, char *user, char *psw) {
  Serial.println("🔄 Connexion au caster NTRIP...");

  if (!_client->connect(host, port)) {
    Serial.println("❌ Connexion échouée au caster");
    return false;
  }

  Serial.println("✅ Socket TCP ouvert");

  String auth = String(user) + ":" + String(psw);
  String encoded = base64::encode((uint8_t*)auth.c_str(), auth.length());

  String req = "GET /" + String(mntpnt) + " HTTP/1.0\r\n";
  req += "Host: " + String(host) + "\r\n";
  req += "User-Agent: NTRIP TinyGSMClient\r\n";
  req += "Ntrip-Version: Ntrip/2.0\r\n";
  req += "Authorization: Basic " + encoded + "\r\n\r\n";

  Serial.println("📤 Requête envoyée :");
  Serial.println(req);
  _client->print(req);

  long timeout = millis() + 7000;
  Serial.println("⏳ Attente de la réponse du serveur...");
  while (_client->available() == 0) {
    if (millis() > timeout) {
      Serial.println("❌ Timeout en attente de réponse");
      return false;
    }
  }

  Serial.println("📥 Réponse du caster :");
  int lines = 0;
  while (_client->available()) {
    String line = _client->readStringUntil('\n');
    Serial.print(">> ");
    Serial.println(line);

    if (line.startsWith("ICY 200") || line.startsWith("HTTP/1.1 200")) {
      Serial.println("✅ Connexion au mountpoint OK");
      return true;
    } else if (line.startsWith("HTTP/1.1 401")) {
      Serial.println("❌ Mauvais identifiants NTRIP !");
      return false;
    }

    if (lines++ > 20) break;  // limite de sécurité
  }

  Serial.println("❌ Réponse inattendue du serveur");
  return false;
}

void NTRIPClient::stop() {
  if (_client) _client->stop();
}

int NTRIPClient::available() {
  if (_client) return _client->available();
  return 0;
}

int NTRIPClient::read() {
  if (_client) return _client->read();
  return -1;
}

int NTRIPClient::readLine(char *buffer, int maxlen) {
  int i = 0;
  while (_client && _client->connected() && i < maxlen - 1) {
    if (_client->available()) {
      char c = _client->read();
      buffer[i++] = c;
      if (c == '\n') break;
    }
  }
  buffer[i] = 0;
  return i;
}

void NTRIPClient::sendGGA(const String &gga) {
  if (_client && _client->connected()) {
    _client->println(gga);
  }
}