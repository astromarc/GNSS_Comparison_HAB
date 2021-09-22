#include <NeoSWSerial.h>
#include <TinyGPSPlus.h>
TinyGPSPlus gps, gal;

int outGps;
int outGal;
long baudRate;
NeoSWSerial ssGPS(outGps, 3);
NeoSWSerial ssGAL(outGal, 5);
NeoSWSerial * serialPort[2] = { &ssGPS, &ssGAL };
long rate=10000;
unsigned long previousMillis = 0;
unsigned long currentMillis;
byte bytegps = 0;
int i = 0;
char datagps[100] = "";
char * lineSc = "";
char * line  = "";
long gpsInitialBoudrate, galInitialBoudrate;
TinyGPSCustom gpspdop(gps, "GPGSA", 15); // $GPGSA sentence, 15th element
TinyGPSCustom gpshdop(gps, "GPGSA", 16); //
TinyGPSCustom gpsvdop(gps, "GPGSA", 17); //
TinyGPSCustom galpdop(gal, "GAGSA", 15); //
TinyGPSCustom galhdop(gal, "GAGSA", 16); //
TinyGPSCustom galvdop(gal, "GAGSA", 17); //

static const uint8_t setBoudRate38400[] = {
  0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00,
  0x23, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00
};
static const int len_setBoudRate = 26;
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



void setup() {
  Serial.begin(38400);
  
  pinMode(outGps, INPUT);      // make sure serial in is a input pin
  digitalWrite (outGps, HIGH); // pull up enabled just for noise protection
  pinMode(outGal, INPUT);      // make sure serial in is a input pin
  digitalWrite (outGal, HIGH); // pull up enabled just for noise protection
  gpsInitialBoudrate = detRate(outGps);
  galInitialBoudrate = detRate(outGal);
  Serial.println("suu");
  Serial.println(gpsInitialBoudrate);
  Serial.println(galInitialBoudrate);
  serialPort[0]->begin(gpsInitialBoudrate); //GPS
  configureGnss(serialPort[0], "GPS", 5);
  serialPort[0]->end(); //GPS
  delay(500);
  serialPort[1]->begin(galInitialBoudrate); // GAL
  configureGnss(serialPort[1], "GAL", 5);
  serialPort[1]->end(); // GAL
  delay(500);
}

void loop() {

  Serial.println("BEGIN LOOP");
  readGnss(serialPort[0], 300, 38400, "GPS", true);
  readGnss(serialPort[1], 300, 38400, "GAL", true);
  if (gps.time.hour() < 10) Serial.print(F("0"));
  Serial.print(gps.time.hour());
  Serial.print(F(":"));
  if (gps.time.minute() < 10) Serial.print(F("0"));
  Serial.print(gps.time.minute());
  Serial.print(F(":"));
  if (gps.time.second() < 10) Serial.print(F("0"));
  Serial.print(gps.time.second());
  Serial.print(F("."));
  if (gps.time.centisecond() < 10) Serial.print(F("0"));
  Serial.println(gps.time.centisecond());

  if (gal.time.hour() < 10) Serial.print(F("0"));
  Serial.print(gal.time.hour());
  Serial.print(F(":"));
  if (gal.time.minute() < 10) Serial.print(F("0"));
  Serial.print(gal.time.minute());
  Serial.print(F(":"));
  if (gal.time.second() < 10) Serial.print(F("0"));
  Serial.print(gal.time.second());
  Serial.print(F("."));
  if (gal.time.centisecond() < 10) Serial.print(F("0"));
  Serial.println(gal.time.centisecond());
}

long detRate(int recpin) { // function to return valid received baud rate
                          // Note that the serial monitor has no 600 baud option and 300 baud
                          // doesn't seem to work with version 22 hardware serial library
   
   long baud,  x;
   
   for (int i = 0; i < 5; i++)
     {
      while(digitalRead(recpin) == 1){} // wait for low bit to start
      x = pulseIn(recpin,LOW);   // measure the next zero bit width
      rate = x < rate ? x : rate;
     }
  // long rate = pulseIn(recpin,LOW);   // measure zero bit width from character 'U'
     if (rate < 12)
      baud = 115200;
      else if (rate < 20)
      baud = 57600;
      else if (rate < 29)
      baud = 38400;
      else if (rate < 40)
      baud = 28800;
      else if (rate < 60)
      baud = 19200;
      else if (rate < 80)
      baud = 14400;
      else if (rate < 150)
      baud = 9600;
      else if (rate < 300)
      baud = 4800;
      else if (rate < 600)
      baud = 2400;
      else if (rate < 1200)
      baud = 1200;
      else 
      baud = 0;
      Serial.println("Baud is:")  ;
      Serial.print(baud)  ;
   return baud; 
  }

void readGnss (NeoSWSerial * serialPort, int sampleTime, int BoudRate, String constellation, bool rawDebug) {
  serialPort->begin(BoudRate);
  while ((currentMillis - previousMillis) <= sampleTime) {
    if (serialPort->available()) {
      memset(lineSc, sizeof(lineSc), 0);
      memset(line, sizeof(line), 0);
      lineSc = getLine(serialPort);//
      if (lineSc != "F") nmeaParsing(lineSc, constellation); if (rawDebug) Serial.println(lineSc);
    }
    currentMillis = millis();
  }
  previousMillis = currentMillis;
  serialPort->end();
}

void nmeaParsing (const char * nmeaSentence, String constellation) {
  char nmeaSentence2 [100];
  strcpy(nmeaSentence2, nmeaSentence);
  strcat(nmeaSentence2, "\r\n");
  char *nmeaSentence3 = nmeaSentence2;
  while (*nmeaSentence3) {
    if (constellation == "GPS") {
      if (gps.encode(*nmeaSentence3++));
    }
    if (constellation == "GAL") {
      if (gal.encode(*nmeaSentence3++));
    }
  }
}


char * getLine (NeoSWSerial * serialPort) {
  memset(datagps, 0, sizeof(datagps));    // Remove previous readings
  bytegps = 0;                            // Remove data
  bytegps = serialPort->read();

  while (bytegps != '$') {

    bytegps = serialPort->read();
  }
  datagps[0] = '$';
  i = 1;
  while (bytegps != '*' ) {
    bytegps = serialPort->read();
    if (bytegps != 255 ) {
      datagps[i] = bytegps;
      i++;
    }
  }

  while (bytegps != '\r' ) { // Also 13 is '\r' character (Carriage Return) in ASCII code
    bytegps = serialPort->read();
    if (bytegps != '\r' && bytegps != 255 ) {
      datagps[i] = bytegps;
      i++;
    }
  }
  char hexadecimalnum [2];
  sprintf(hexadecimalnum, "%02X", nmea0183_checksum(datagps));
  if ( ((datagps[strlen(datagps) - 2]) == hexadecimalnum[0]) && ((datagps[strlen(datagps) - 1]) == hexadecimalnum[1]) ) return datagps;
  else {
    return "F";
  }
}

void configureGnss(NeoSWSerial * serialPort, String constellation, int freq) {
  serialPort->flush();
  delay(50);
  sendUBX(setNavAir1G, len_setNav, serialPort); // Set Airborne <1G Navigation Mode
  delay(50);
  sendUBX(setNmea4_1, len_setNmea, serialPort); // Set NMEA 4.1 (needed 4 Galileo)
  delay(50);
  if (freq == 1) {
    sendUBX(setFreq_1Hz, len_setFreq, serialPort); // Set GNSS to only GPS
    delay(50);
  }
  if (freq == 5) {
    sendUBX(setFreq_5Hz, len_setFreq, serialPort); // Set GNSS to only GPS
    delay(50);
  }
  if (constellation == "GPS") {
    sendUBX(setGnssGps, len_setGnss, serialPort);// Set GNSS to only GPS
    delay(50);
  }
  if (constellation == "GAL") {
    sendUBX(setGnssGal, len_setGnss, serialPort);// Set GNSS to only GAL
    delay(50);
  }
   sendUBX(setBoudRate38400, len_setBoudRate, serialPort);
   delay(50);
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

int nmea0183_checksum(char *nmea_data)
{
  int crc = 0;
  int i;


  for (i = 1; i < strlen(nmea_data) - 3; i ++) { // removed the - 3 because no cksum is present
    crc ^= nmea_data[i];
  }

  return crc;
}
