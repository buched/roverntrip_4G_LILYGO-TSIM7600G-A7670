/**
 * @file     TinyGsmClientSIM808.h
 * @author   Volodymyr Shymanskyy
 * @license  LGPL-3.0
 * @copyright  Copyright (c) 2016 Volodymyr Shymanskyy
 * @date     Nov 2016
 */

#ifndef SRC_TINYGSMCLIENTSIM808_H_
#define SRC_TINYGSMCLIENTSIM808_H_
// #pragma message("TinyGSM:  TinyGsmClientSIM808")

#include "TinyGsmClientSIM800.h"
#include "TinyGsmGPS.tpp"
#include "TinyGsmBluetooth.tpp"

class TinyGsmSim808 : public TinyGsmSim800, public TinyGsmGPS<TinyGsmSim808>, public TinyGsmBluetooth<TinyGsmSim808> {
  friend class TinyGsmGPS<TinyGsmSim808>;
  friend class TinyGsmBluetooth<TinyGsmSim808>;

 public:
  explicit TinyGsmSim808(Stream& stream) : TinyGsmSim800(stream) {}


  /*
   * GPS/GNSS/GLONASS location functions
   */
 protected:
  // enable GPS
  bool enableGPSImpl(int8_t power_en_pin ,uint8_t enable_level) {
    sendAT(GF("+CGNSPWR=1"));
    if (waitResponse() != 1) { return false; }
    return true;
  }

  bool disableGPSImpl(int8_t power_en_pin ,uint8_t disable_level) {
    sendAT(GF("+CGNSPWR=0"));
    if (waitResponse() != 1) { return false; }
    return true;
  }

  bool isEnableGPSImpl(){
    sendAT(GF("+CGNSPWR?"));
    if (waitResponse(GF(GSM_NL "+CGNSPWR:")) != 1) { return false; }
    bool running = 1 == streamGetIntBefore('\r');
    waitResponse();
    return running;
  }

  bool setGPSBaudImpl(uint32_t baud){
    DBG("Modem does not support set GPS baudrate.");
    return  false;
  }

  bool setGPSModeImpl(uint8_t mode){
      sendAT("+CGNSMOD=1,1,1,1");
      return waitResponse(1000L) == 1;
  }

  bool setGPSOutputRateImpl(uint8_t rate_hz){
    DBG("Modem does not support set GPS output rate.");
    return  false;
  }

  bool enableNMEAImpl(bool outputAtPort){
    DBG("Modem does not support set GPS NMEA output.");
    return true;
  }

  bool disableNMEAImpl(){
    DBG("Modem does not support set GPS NMEA output.");
    return true;
  }

  bool configNMEASentenceImpl(bool CGA,bool GLL,bool GSA,bool GSV,bool RMC,bool VTG,bool ZDA,bool ANT){
    DBG("Modem does not support set GPS NMEA.");
    return false;
  }
  
  // get the RAW GPS output
  // works only with ans SIM808 V2
  String getGPSrawImpl() {
    sendAT(GF("+CGNSINF"));
    if (waitResponse(10000L, GF(GSM_NL "+CGNSINF:")) != 1) { return ""; }
    String res = stream.readStringUntil('\n');
    waitResponse();
    res.trim();
    return res;
  }

  // get GPS informations
  // works only with ans SIM808 V2
  bool getGPSImpl(uint8_t *status,float* lat, float* lon, float* speed = 0, float* alt = 0,
                  int* vsat = 0, int* usat = 0, float* accuracy = 0,
                  int* year = 0, int* month = 0, int* day = 0, int* hour = 0,
                  int* minute = 0, int* second = 0) {
    sendAT(GF("+CGNSINF"));
    if (waitResponse(10000L, GF(GSM_NL "+CGNSINF:")) != 1) { return false; }

    streamSkipUntil(',');                // GNSS run status
    if (streamGetIntBefore(',') == 1) {  // fix status
      // init variables
      float ilat         = 0;
      float ilon         = 0;
      float ispeed       = 0;
      float ialt         = 0;
      int   ivsat        = 0;
      int   iusat        = 0;
      float iaccuracy    = 0;
      int   iyear        = 0;
      int   imonth       = 0;
      int   iday         = 0;
      int   ihour        = 0;
      int   imin         = 0;
      float secondWithSS = 0;

      // UTC date & Time
      iyear  = streamGetIntLength(4);  // Four digit year
      imonth = streamGetIntLength(2);  // Two digit month
      iday   = streamGetIntLength(2);  // Two digit day
      ihour  = streamGetIntLength(2);  // Two digit hour
      imin   = streamGetIntLength(2);  // Two digit minute
      secondWithSS =
          streamGetFloatBefore(',');  // 6 digit second with subseconds

      ilat   = streamGetFloatBefore(',');  // Latitude
      ilon   = streamGetFloatBefore(',');  // Longitude
      ialt   = streamGetFloatBefore(',');  // MSL Altitude. Unit is meters
      ispeed = streamGetFloatBefore(',');  // Speed Over Ground. Unit is knots.
      streamSkipUntil(',');                // Course Over Ground. Degrees.
      streamSkipUntil(',');                // Fix Mode
      streamSkipUntil(',');                // Reserved1
      iaccuracy =
          streamGetFloatBefore(',');    // Horizontal Dilution Of Precision
      streamSkipUntil(',');             // Position Dilution Of Precision
      streamSkipUntil(',');             // Vertical Dilution Of Precision
      streamSkipUntil(',');             // Reserved2
      ivsat = streamGetIntBefore(',');  // GNSS Satellites in View
      iusat = streamGetIntBefore(',');  // GNSS Satellites Used
      streamSkipUntil(',');             // GLONASS Satellites Used
      streamSkipUntil(',');             // Reserved3
      streamSkipUntil(',');             // C/N0 max
      streamSkipUntil(',');             // HPA
      streamSkipUntil('\n');            // VPA

      // Set pointers
      if (lat != NULL) *lat = ilat;
      if (lon != NULL) *lon = ilon;
      if (speed != NULL) *speed = ispeed;
      if (alt != NULL) *alt = ialt;
      if (vsat != NULL) *vsat = ivsat;
      if (usat != NULL) *usat = iusat;
      if (accuracy != NULL) *accuracy = iaccuracy;
      if (iyear < 2000) iyear += 2000;
      if (year != NULL) *year = iyear;
      if (month != NULL) *month = imonth;
      if (day != NULL) *day = iday;
      if (hour != NULL) *hour = ihour;
      if (minute != NULL) *minute = imin;
      if (second != NULL) *second = static_cast<int>(secondWithSS);

      waitResponse();
      return true;
    }

    streamSkipUntil('\n');  // toss the row of commas
    waitResponse();
    return false;
  }
  
    /*
   * Bluetooth functions
   */
   
  bool enableBluetoothImpl() {
    sendAT(GF("+BTPOWER=1"));
    if (waitResponse() != 1) { return false; }
    return true;
  }

  bool disableBluetoothImpl() {
    sendAT(GF("+BTPOWER=0"));
    if (waitResponse() != 1) { return false; }
    return true;
  }
  
  bool setBluetoothVisibilityImpl(bool visible) {
    sendAT(GF("+BTVIS="), visible);
    if (waitResponse() != 1) {
      return false;
    }
    
    return true;
  }

  bool setBluetoothHostNameImpl(const char* name) {
    sendAT(GF("+BTHOST="), name);
    if (waitResponse() != 1) {
      return false;
    }
    
    return true;
  }
};

#endif  // SRC_TINYGSMCLIENTSIM808_H_
