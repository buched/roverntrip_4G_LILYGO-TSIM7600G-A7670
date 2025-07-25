#ifndef NTRIP_CLIENT_H
#define NTRIP_CLIENT_H

//#ifdef MODEL_SIM7600
  //#define TINY_GSM_MODEM_SIM7600
//#elif defined(MODEL_A7670G)
  #define TINY_GSM_MODEM_A7670
//#elif defined(MODEL_A7670E)
  //#define TINY_GSM_MODEM_A7670
//#else
  //#error "Aucun modele selectionne! Decommenter un #define MODEL_xxx"
//#endif
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
