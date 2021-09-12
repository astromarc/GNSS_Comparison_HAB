#include <TinyGPSPlus.h>
#include <NeoSWSerial.h>

NeoSWSerial ssGPS(10, 11), ssGAL(12, 13);
NeoSWSerial * serialPort[2] = {&ssGPS, &ssGAL};
TinyGPSPlus gal, gps;

String gpsLine, galLine, gpsLineVar, galLineVar;
int gnssBoudRate = 9600;
unsigned long previousMillis = 0;
unsigned long currentMillis;

TinyGPSCustom gpspdop(gps, "GPGSA", 15); // $GPGSA sentence, 15th element
TinyGPSCustom gpshdop(gps, "GPGSA", 16); //
TinyGPSCustom gpsvdop(gps, "GPGSA", 17); //
TinyGPSCustom galpdop(gal, "GAGSA", 15); //
TinyGPSCustom galhdop(gal, "GAGSA", 16); //
TinyGPSCustom galvdop(gal, "GAGSA", 17); //
TinyGPSCustom galtime(gal, "GAGGA", 1); //
TinyGPSCustom gpsaltitude(gps, "GPGGA", 9); //
TinyGPSCustom gpssatellites(gps, "GPGGA", 7);
TinyGPSCustom galsatellites(gal, "GAGGA", 7); //



// Set Nav Mode to Airborne 4G
static const uint8_t setNavAir4G[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x20
};

// Set Nav Mode to Airborne <1G
static const uint8_t setNavAir1G[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
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
  0x06, 0x08, 0x0e, 0x00, 0x00, 0x00, 0x01, 0x01
}; // GLONASS

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
  0x06, 0x08, 0x0e, 0x00, 0x01, 0x00, 0x01, 0x01
};


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
  0x06, 0x08, 0x0e, 0x00, 0x00, 0x00, 0x01, 0x01
}; // GLONASS
static const int len_setGnss = 66;


// Set NMEA 4.1 Protocol
static const uint8_t setNmea4_1[] = {
  0xB5, 0x62, 0x06, 0x17, 0x14, 0x00,
  0x00, 0x41, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static const int len_setNmea = 26;

// Set 5Hz
static const uint8_t setFreq_5Hz[] = {
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
  0xC8, 0x00, 0x01, 0x00, 0x01, 0x00
};
static const int len_setFreq = 12;

// Set 5Hz
static const uint8_t setFreq_1Hz[] = {
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
  0xE8, 0x03, 0x01, 0x00, 0x01, 0x00
};

String gnssDebug = "GAL";

void setup() {
  Serial.begin(19200);

  serialPort[0]->begin(gnssBoudRate); //GPS
  configureGnss(serialPort[0], "GPS", 5);
  serialPort[0]->end(); //GPS
  delay(500);
  serialPort[1]->begin(gnssBoudRate); // GAL
  configureGnss(serialPort[1], "GAL", 5);
  serialPort[1]->end(); // GAL
  delay(500);
}


void loop() {

 // readGnss(serialPort[0], 500, false, "GPS", gps); // SS, Sample Time, RawOutcome, constellationName, constellationObject
  readGnss(serialPort[1], 500, false, "GAL", gal); // SS, Sample Time, RawOutcome, constellationName, constellationObject
  Serial.println(gpsLine);
  Serial.println(galLine);

}


void readGnss (NeoSWSerial * serialPort, int sampleTime, bool rawDebug, String constellation, TinyGPSPlus gnss) {
  serialPort->begin(gnssBoudRate);
  while ((currentMillis - previousMillis) <= sampleTime) {
    if (rawDebug == true) {
      if (serialPort->available()) {
        Serial.write(serialPort->read());
      }
      if (Serial.available()) {
        serialPort->write(Serial.read());
      }
    }
    else {
    while (serialPort->available() > 0)
    if (gnss.encode(serialPort->read()))
    displayInfo(gnss, constellation);
    }
    
    currentMillis = millis();
  }
  previousMillis = currentMillis;
  serialPort->end();
}



void configureGnss(NeoSWSerial * serialPort, String constellation, int freq) {
  serialPort->flush();
  delay(100);
  sendUBX(setNavAir1G, len_setNav, serialPort); // Set Airborne <1G Navigation Mode
  delay(100);
  sendUBX(setNmea4_1, len_setNmea, serialPort); // Set NMEA 4.1 (needed 4 Galileo)
  delay(100);
  if (constellation == "GPS") {
    sendUBX(setGnssGps, len_setGnss, serialPort);// Set GNSS to only GPS
    delay(100);
  }
  if (constellation == "ALL") {
    sendUBX(setGnssAll, len_setGnss, serialPort);// Set GNSS to only GPS
    delay(100);
  }
  if (constellation == "GAL") {
    sendUBX(setGnssGal, len_setGnss, serialPort);// Set GNSS to only GAL
    delay(100);
  }
  if (freq == 1) {
    sendUBX(setFreq_1Hz, len_setFreq, serialPort); // Set GNSS to only GPS
    delay(100);
  }
  if (freq == 5) {
    sendUBX(setFreq_5Hz, len_setFreq, serialPort); // Set GNSS to only GPS
    delay(100);
  }
}


void sendUBX(const uint8_t *message, const int len, NeoSWSerial * serialPort) {
  int csum1 = 0; // Checksum bytes
  int csum2 = 0;
  for (int i = 0; i < len; i++) { // For each byte in the message
    serialPort->write(message[i]); // Write the byte
    if (i >= 2) { // Don't include the sync chars in the checksum
      csum1 = csum1 + message[i]; // Update the checksum bytes
      csum2 = csum2 + csum1;
    }
  }
  csum1 = csum1 & 0xff; // Limit checksums to 8-bits
  csum2 = csum2 & 0xff;
  serialPort->write((uint8_t)csum1); // Send the checksum bytes
  serialPort->write((uint8_t)csum2);
}

void displayInfo(TinyGPSPlus gnss, String constellation)
{
  if (constellation = "GPS") {
    gpsLine = "GPS,";
    if (gnss.time.isValid())
    {
      if (gnss.time.hour() < 10) gpsLine += "0";

      gpsLine += String(gnss.time.hour());
      gpsLine += ":";

      if (gnss.time.minute() < 10) gpsLine += "0";
      gpsLine += String(gnss.time.minute());
      gpsLine += ":";

      if (gnss.time.second() < 10) gpsLine += "0";
      gpsLine += String(gnss.time.second());
      gpsLine += ".";
      if (gnss.time.centisecond() < 10) gpsLine += "0";
      gpsLine += String(gnss.time.centisecond());
      gpsLine += ",";

    }
    else
    {
      gpsLine += "invalidtime,";
    }

    if (gnss.location.isValid())
    {
      gpsLine +=  String(gnss.location.lat(), 6);
      gpsLine += ",";
      gpsLine += String(gnss.location.lng(), 6);
      gpsLine += ",";
    }
    else
    {
      gpsLine += "invalidloc,";
    }
    
    gpsLine += String(gpsaltitude.value());
    gpsLine += ",";
    gpsLine +=   String(gpspdop.value());
    gpsLine += ",";
    gpsLine +=   String(gpshdop.value());
    gpsLine += ",";
    gpsLine +=  String(gpsvdop.value());
    gpsLine += ",";
    gpsLine +=  String(gpssatellites.value());
  }

  if (constellation = "GAL") {
    galLine = "GAL,";
    if (gnss.time.isValid())
    {
      if (gnss.time.hour() < 10) galLine += "0";
      galLine += String(gnss.time.hour());
      galLine += ":";
      if (gnss.time.minute() < 10) galLine += "0";
      galLine += String(gnss.time.minute());
      galLine += ":";

      if (gnss.time.second() < 10) galLine += "0";
      galLine += String(gnss.time.second());
      galLine += ".";
      if (gnss.time.centisecond() < 10) galLine += "0";
      galLine += String(gnss.time.centisecond());
      galLine += ",";
    }
    else
    {
      galLine += "invalidtime,";
    }

    if (gnss.location.isValid())
    {
      galLine +=  String(gnss.location.lat(), 6);
      galLine += ",";
      galLine += String(gnss.location.lng(), 6);
      galLine += ",";
    }
    else
    {
      galLine += "invalidloc,";
    }
    galLine += String(galtime.value());
    galLine += ",";
    galLine += String(galsatellites.value());
    




  }
}
