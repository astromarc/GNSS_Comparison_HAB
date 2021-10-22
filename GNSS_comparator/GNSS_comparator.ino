#include <SoftwareSerial.h>
SoftwareSerial ssGPS(5, 4);
SoftwareSerial ssGAL(3, 2);
SoftwareSerial * serialPort[2] = { &ssGPS, &ssGAL };
const int boudRate = 9600;
unsigned long previousMillis = 0;
unsigned long currentMillis;
byte bytegps = 0;
int i = 0;
char datagps[90] = "";
char * lineSc = "";


//
//static const uint8_t set3840038400[] = {
//  0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00,
//  0x23, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00
//};
//static const int len_set38400 = 26;


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

// Set 1Hz
static const uint8_t setFreq_1Hz[] = {
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
  0xE8, 0x03, 0x01, 0x00, 0x01, 0x00
};



void setup() {
  Serial.begin(boudRate);
  Serial.println(F("Start Setup"));
  ssGPS.begin(boudRate);
  ssGAL.begin(boudRate);
  configureGnss(serialPort[0], "GPS", 1);
  configureGnss(serialPort[1], "GAL", 1);
  

}

void loop() {
  Serial.println("BEGIN LOOP");
  readGnss(serialPort[0], 1000, boudRate, "GPS", true);
  readGnss(serialPort[1], 1000, boudRate, "GAL", true);
  
  
 
}


void readGnss (SoftwareSerial * serialPort, int sampleTime, int boudrate, String constellation, bool rawDebug) {

serialPort->listen();
  while ((currentMillis - previousMillis) <= sampleTime) {
    if (serialPort->available()) {
      memset(lineSc, sizeof(lineSc), 0);
      lineSc = getLine(serialPort);//
      if (rawDebug) Serial.println(lineSc);
    }
    currentMillis = millis();
  }
  previousMillis = currentMillis;



}

char * getLine (SoftwareSerial * serialPort) {
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

  while (bytegps != '\n' ) { // Also 13 is '\r' character (Carriage Return) in ASCII code
    bytegps = serialPort->read();
    if (bytegps != '\n' && bytegps != 255 ) {
      datagps[i] = bytegps;
      i++;
    }
  }

  return datagps;

}


void configureGnss(SoftwareSerial * serialPort, String constellation, int freq) {
  serialPort->listen();
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
}

void sendUBX(const uint8_t *message, const int len, SoftwareSerial * serialPort) {
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