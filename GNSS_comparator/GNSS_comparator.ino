// SD
#include <SPI.h>
#include <SD.h>
const int chipSelect = 9;

// BMP180
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;
double baselinePressure = 100900; // In PA

// GNSS
#include <PString.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
TinyGPSPlus gps;
TinyGPSPlus gal;
SoftwareSerial ssGPS(5, 4);
SoftwareSerial ssGAL(3, 2);
SoftwareSerial ssLORA(7, 6);
SoftwareSerial * serialPort[3] = { &ssGPS, &ssGAL, &ssLORA};
const int boudRate = 9600;
unsigned long previousMillis = 0;
unsigned long currentMillis;
byte bytegps = 0;
int i = 0;
int numLora = 20; //  to be multiplied for 3 seconds (e.g., 20 each minute, 10 for 30 sec...
int counter = 0;
char datagps[90] = "";
char * lineSc = "";
String mssgLora = "";
String mssgLora2 = "";
TinyGPSCustom gpspdop(gps, "GPGSA", 15); // $GPGSA sentence, 15th element
TinyGPSCustom gpshdop(gps, "GPGSA", 16); // $GPGSA sentence, 16th element
TinyGPSCustom gpsvdop(gps, "GPGSA", 17); // $GPGSA sentence, 17th element
TinyGPSCustom galpdop(gal, "GAGSA", 15); // $GPGSA sentence, 15th element
TinyGPSCustom galhdop(gal, "GAGSA", 16); // $GPGSA sentence, 16th element
TinyGPSCustom galvdop(gal, "GAGSA", 17); // $GPGSA sentence, 17th element
TinyGPSCustom gpssatellites(gps, "GPGGA", 7);
TinyGPSCustom galsatellites(gal, "GAGGA", 7); //


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
  Serial.begin(19200);
  serialPort[2]->begin(9600); // LoRa
  Serial.println(F("Start Setup"));
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  digitalWrite(A0, HIGH);

  //SD
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present"); digitalWrite(A0, HIGH);
    // don't do anything more:
    while (1);
  }
  File dataFile = SD.open("data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("GALTIME,GALLAT,GALLONG,GALALT,GALSAT,GALPDOP,GALHDOP,GALVDOP,GPSTIME,GPSLAT,GPSLONG,GPSALT,GPSSAT,GPSPDOP,GPSHDOP,GPSVDOP,BMPTEMP,BMPALT,BMPPRESSURE");
    dataFile.close();
  }


  //BMP180
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!"); digitalWrite(A0, HIGH);
    while (1) {}
  }

  // GPS
  ssGPS.begin(boudRate);
  ssGAL.begin(boudRate);
  configureGnss(serialPort[0], "GPS", 5);
  configureGnss(serialPort[1], "GAL", 5);



}

void loop() {
  //Serial.println("BEGIN LOOP");

  mssgLora = "";
  readGnss(serialPort[0], 1750, boudRate, "GPS", false);
  readGnss(serialPort[1], 1000, boudRate, "GAL", false);
  // Prepare message to send via LoRa
  if (gps.location.isValid()) {
    mssgLora = displayInfoSmall(gps);
    //Serial.println("GPS");
  }
  else {
    mssgLora = displayInfoSmall(gal);
    //Serial.println("GAL");
  }
  Serial.println(mssgLora);
  if (counter > numLora) {
    serialPort[2]->flush();
    serialPort[2]->println(mssgLora);
    serialPort[2]->flush();
    Serial.println(mssgLora);
    counter = 0;
  }
  counter++;
  if (counter <= (numLora -5)){
  sdWrite(gps, gal);}

  //Serial.println(displayInfo(gps));
  //Serial.println(displayInfo(gal));


  // INDICATORS
  if (bmp.readAltitude(baselinePressure) <= 5000 && counter <= numLora) {
    if (gal.location.isValid() || gps.location.isValid()) {
      digitalWrite(A2, LOW);
      digitalWrite(A2, HIGH);
      delay(250);
      digitalWrite(A2, LOW);
      digitalWrite(A0, LOW);
    }
    if (gal.location.isValid() && gps.location.isValid()) {
      digitalWrite(A2, LOW);
      digitalWrite(A1, HIGH);
      delay(250);
      digitalWrite(A1, LOW);
      digitalWrite(A0, LOW);
    }
    if (gal.location.isValid() == false && (gps.location.isValid() == false)) {
      digitalWrite(A0, HIGH);
      delay(500);
      digitalWrite(A0, LOW);
    }
  }
  else {
    digitalWrite(A0, LOW);
    digitalWrite(A1, LOW);
    digitalWrite(A2, LOW);
  }


}


void sdWrite (TinyGPSPlus gnssGps, TinyGPSPlus gnssGal) {
  String mssgGps = "";
  String mssgGal = "";
  String mssgSd = "";
  mssgGal = displayInfo(gnssGal);
  mssgGal += ",";
  mssgGal += gal.satellites.value();
  mssgGal += ",";
  mssgGal += galpdop.value();
  mssgGal += ",";
  mssgGal += galhdop.value();
  mssgGal += ",";
  mssgGal += galvdop.value();
  mssgGal += ",";

  mssgGps = displayInfo(gnssGps);
  mssgGps += ",";
  mssgGps += gps.satellites.value();
  mssgGps += ",";
  mssgGps += gpspdop.value();
  mssgGps += ",";
  mssgGps += gpshdop.value();
  mssgGps += ",";
  mssgGps += gpsvdop.value();


  mssgSd += mssgGal += mssgGps;
  mssgSd += ",";
  mssgSd += String(bmp.readTemperature());
  mssgSd += ",";
  mssgSd += String(bmp.readAltitude(baselinePressure));
  mssgSd += ",";
  mssgSd += String(bmp.readPressure());
  SD.begin(chipSelect);
  File dataFile = SD.open("data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println(mssgSd);
    dataFile.close();
  }
 

  Serial.println(mssgSd);

}


void readGnss (SoftwareSerial * serialPort, int sampleTime, int boudrate, String constellation, bool rawDebug) {

  serialPort->listen();
  while ((currentMillis - previousMillis) <= sampleTime) {
    if (serialPort->available()) {
      memset(lineSc, sizeof(lineSc), 0);
      lineSc = getLine(serialPort);
      if (rawDebug) Serial.println(lineSc);
      if (constellation == "GPS")  {
        while (*lineSc) if (gps.encode(*lineSc++));
      }
      if (constellation == "GAL")  {
        while (*lineSc) if (gal.encode(*lineSc++));
      }
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

String displayInfo(TinyGPSPlus gnss) {
  String mssg = "";
  if (gnss.time.isValid())
  {
    if (gnss.time.hour() < 10) mssg += (F("0"));
    mssg = (gnss.time.hour());
    mssg += (F(":"));
    if (gnss.time.minute() < 10) mssg += (F("0"));
    mssg += (gnss.time.minute());
    mssg += (F(":"));
    if (gnss.time.second() < 10) mssg += (F("0"));
    mssg += (gnss.time.second());
    mssg += (F("."));
    if (gnss.time.centisecond() < 10) mssg += (F("0"));
    mssg += (gnss.time.centisecond());
  }
  else
  {
    mssg += (F("X"));
  }
  mssg += (F(","));
  // Location
  if (gnss.location.isValid())
  {
    mssg += String(gnss.location.lat(), 4);
    mssg += (F(","));
    mssg += String(gnss.location.lng(), 4);
    mssg += (F(","));
    mssg += String(gnss.altitude.meters());
  }
  else
  {
    mssg += (F("X,X,X"));
  }

  return mssg;

}

String displayInfoSmall(TinyGPSPlus gnss) {
  String mssg = "";
  // Location
  if (gnss.location.isValid())
  {
    mssg += String(gnss.location.lat(), 4);
    mssg += (F(","));
    mssg += String(gnss.location.lng(), 4);
    mssg += (F(","));
    mssg += String(gnss.altitude.meters(),0);
  }
  else
  {
    mssg += (F("X,X,X"));
  }

  return mssg;

}
