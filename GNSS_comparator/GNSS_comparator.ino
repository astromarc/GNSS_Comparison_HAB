#include <TinyGPSPlus.h>
TinyGPSPlus gal, gps;
#include <SoftwareSerial.h>
SoftwareSerial ssGAL(11, 10), ssGPS(2,3); // RX, TX
int gnssBoudRate = 9600;
unsigned long previousMillis=0;
unsigned long currentMillis;
String gpsLine, galLine;
TinyGPSCustom gpspdop(gps, "GPGSA", 15); // $GPGSA sentence, 15th element
TinyGPSCustom gpshdop(gps, "GPGSA", 16); // $GPGSA sentence, 16th element
TinyGPSCustom gpsvdop(gps, "GPGSA", 17); // $GPGSA sentence, 17th element
TinyGPSCustom galpdop(gal, "GAGSA", 15); // $GPGSA sentence, 15th element
TinyGPSCustom galhdop(gal, "GAGSA", 16); // $GPGSA sentence, 16th element
TinyGPSCustom galvdop(gal, "GAGSA", 17); // $GPGSA sentence, 17th element

// Set Nav Mode to Airborne 4G
static const uint8_t setNavAir4G[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x20};           

// Set Nav Mode to Airborne <1G
static const uint8_t setNavAir1G[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const int len_setNav = 42;

// Set only GALILEO GNSS
static const uint8_t setGnssGal[] = {
  0xb5, 0x62, 0x06, 0x3e, 0x3c, 0x00,
  0x00, 0x20, 0x20, 0x07,
  0x00, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, // GPS
  0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, // SBAS
  0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01, // GALILEO
  0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, // BeiDou
  0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x03, // IMES
  0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x05, // QZSS
  0x06, 0x08, 0x0e, 0x00, 0x00, 0x00, 0x01, 0x01 }; // GLONASS

// Set GNSS Config to GPS + Galileo + GLONASS + SBAS (Causes the M8 to restart!)
static const uint8_t setGnssAll[] = {
  0xb5, 0x62, 0x06, 0x3e, 0x3c, 0x00,
  0x00, 0x20, 0x20, 0x07,
  0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01,
  0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01,
  0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01,
  0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01,
  0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x03,
  0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x05,
  0x06, 0x08, 0x0e, 0x00, 0x01, 0x00, 0x01, 0x01 };


// Set only GPS GNSS
static const uint8_t setGnssGps[] = {
  0xb5, 0x62, 0x06, 0x3e, 0x3c, 0x00,
  0x00, 0x20, 0x20, 0x07,
  0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, // GPS
  0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, // SBAS
  0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, // GALILEO
  0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, // BeiDou
  0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x03, // IMES
  0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x05, // QZSS
  0x06, 0x08, 0x0e, 0x00, 0x00, 0x00, 0x01, 0x01 }; // GLONASS
static const int len_setGnss = 66;


// Set NMEA 4.1 Protocol
static const uint8_t setNmea4_1[] = {
  0xB5, 0x62, 0x06, 0x17, 0x14, 0x00,
  0x00, 0x41, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const int len_setNmea = 26;


// Set 5Hz
static const uint8_t setFreq_5Hz[] = {
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
  0xC8, 0x00, 0x01, 0x00, 0x01,0x00};
static const int len_setFreq = 12;

// Set 1Hz
static const uint8_t setFreq_1Hz[] = {
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
  0xE8, 0x03, 0x01, 0x00, 0x01,0x00};

void setup() {
  
  Serial.begin(9600);
  ssGAL.begin(gnssBoudRate);
  configureGnss(ssGAL,"GAL",5);
  ssGAL.end();
  delay(500);
  ssGPS.begin(gnssBoudRate);
  configureGnss(ssGPS,"GPS",5);
  ssGPS.end();
  delay(500);
  
}




void loop() { 

 readGnss(ssGAL, 500, true, "GAL");
 readGnss(ssGPS, 500, true,"GPS");

}


void readGnss (SoftwareSerial gnss, int sampleTime, bool rawDebug, String constellation){
  gnss.begin(gnssBoudRate);
  while ((currentMillis-previousMillis) <= sampleTime){
  if (rawDebug == true){
  if (gnss.available()) {
    Serial.write(gnss.read());
  }
  if (Serial.available()) {
    gnss.write(Serial.read());
  }
  }
 else{
    if(constellation == "GPS"){
    while (gnss.available() > 0)
    if (gps.encode(gnss.read()))
    displayInfo(gps, constellation); Serial.println(gpsLine);
    }
    if(constellation == "GAL"){
    while (gnss.available() > 0)
    if (gps.encode(gnss.read()))
    displayInfo(gal, constellation);Serial.println(galLine);}
  }
 currentMillis = millis();
  }
  previousMillis = currentMillis;
  gnss.end();
  }
  
  


void configureGnss(SoftwareSerial gnss, String constellation, int boudRate){
  ssGAL.flush();
  delay(100);
  sendUBX(setNavAir1G, len_setNav, gnss); // Set Airborne <1G Navigation Mode
  delay(100);  
  sendUBX(setNmea4_1, len_setNmea, gnss); // Set NMEA 4.1 (needed 4 Galileo)
  delay(100);
  if (constellation == "GPS"){
  sendUBX(setGnssGps, len_setGnss, gnss);// Set GNSS to only GPS
  delay(100);}
  if (constellation == "ALL"){
  sendUBX(setGnssAll, len_setGnss, gnss);// Set GNSS to only GPS
  delay(100);}
  if (constellation == "GAL"){
  sendUBX(setGnssGal, len_setGnss, gnss);// Set GNSS to only GAL
  delay(100);}
  if (boudRate == 1){
  sendUBX(setFreq_1Hz, len_setFreq, gnss); // Set GNSS to only GPS
  delay(100);}
  if (boudRate == 5){
  sendUBX(setFreq_5Hz, len_setFreq, gnss); // Set GNSS to only GPS
  delay(100);}
   }


void sendUBX(const uint8_t *message, const int len, SoftwareSerial serial) {
  int csum1 = 0; // Checksum bytes
  int csum2 = 0;
  for (int i=0; i<len; i++) { // For each byte in the message
    serial.write(message[i]); // Write the byte
    if (i >= 2) { // Don't include the sync chars in the checksum
      csum1 = csum1 + message[i]; // Update the checksum bytes
      csum2 = csum2 + csum1;
    }
  }
  csum1 = csum1 & 0xff; // Limit checksums to 8-bits
  csum2 = csum2 & 0xff;
  serial.write((uint8_t)csum1); // Send the checksum bytes
  serial.write((uint8_t)csum2);
}

void displayInfo(TinyGPSPlus gnss, String constellation)
{
  if (constellation = "GPS"){
     gpsLine = "GPS,";
if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) gpsLine += "0";
    
    gpsLine += String(gps.time.hour());
    gpsLine += ":";
    
    if (gps.time.minute() < 10) gpsLine += "0";
    gpsLine += String(gps.time.minute());
    gpsLine += ":";
    
    if (gps.time.second() < 10) gpsLine += "0";
    gpsLine += String(gps.time.second());
    gpsLine += ".";
    if (gps.time.centisecond() < 10) gpsLine += "0";
    gpsLine += String(gps.time.centisecond());
    gpsLine += ",";
    
  }
  else
  {
    gpsLine +="invaidtime,";
  }

  if (gps.location.isValid())
  {
    gpsLine +=  String(gps.location.lat(), 6);
    gpsLine += ",";
    gpsLine += String(gps.location.lng(), 6);
    gpsLine += ",";
  }
  else
  {
    gpsLine += "invalidloc,";
  }
     gpsLine +=   String(gps.altitude.meters()); 
     gpsLine += ",";
     gpsLine +=   String(gpspdop.value()); 
     gpsLine += ",";
     gpsLine +=   String(gpshdop.value()); 
     gpsLine += ",";
     gpsLine +=  String(gpsvdop.value());
     gpsLine += ",";
     gpsLine +=  String(gps.satellites.value());

}

  if (constellation = "GAL"){
     galLine = "GAL,";
if (gal.time.isValid())
  {
    if (gal.time.hour() < 10) galLine += "0";
    
    galLine += String(gal.time.hour());
    galLine += ":";
    
    if (gal.time.minute() < 10) galLine += "0";
    galLine += String(gal.time.minute());
    galLine += ":";
    
    if (gal.time.second() < 10) galLine += "0";
    galLine += String(gps.time.second());
    galLine += ".";
    if (gal.time.centisecond() < 10) galLine += "0";
    galLine += String(gal.time.centisecond());
    galLine += ",";
    
  }
  else
  {
    galLine +="invaidtime,";
  }

  if (gal.location.isValid())
  {
    galLine +=  String(gal.location.lat(), 6);
    galLine += ",";
    galLine += String(gal.location.lng(), 6);
    galLine += ",";
  }
  else
  {
    galLine += "invalidloc,";
  }
     galLine +=   String(gal.altitude.meters()); 
     galLine += ",";
     galLine +=   String(galpdop.value()); 
     galLine += ",";
     galLine +=   String(galhdop.value()); 
     galLine += ",";
     galLine +=  String(galvdop.value());
     galLine += ",";
     galLine +=  String(gal.satellites.value());

  

  
}
}
