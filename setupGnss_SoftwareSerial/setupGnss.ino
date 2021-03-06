// Basic program to configure GNSS modules.
// It starts the module at NMEA 4.1 at Airborne <1G with a configurable frequency (0.5, 1 and 5Hz)
// and option to only show GxGGA or all the NMEA sentences (see configureGNSS function)
// No warranties at all :)
// if only using a Soft Serial, just somment ssGAL and put just '1' in the serialPort pointer. 
// use of pointer with Software Serial to make it easier to play with when declaring functions;
// as this code is intended to be used when communicating whith serial with more than one GNSS modules
// GNSS Modules ~ GPS Modules ;)


#include <SoftwareSerial.h>
SoftwareSerial ssGPS(5, 4);
SoftwareSerial ssGAL(3, 2);
SoftwareSerial * serialPort[2] = { &ssGPS, &ssGAL };



// Set Nav Mode to Airborne <1G
static const uint8_t setNavAir1G[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


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



// Set NMEA 4.1 Protocol
static const uint8_t setNmea4_1[] = {
  0xB5, 0x62, 0x06, 0x17, 0x14, 0x00,
  0x00, 0x41, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


// Set 5Hz
static const uint8_t setFreq_5Hz[] = {
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
  0xC8, 0x00, 0x01, 0x00, 0x01, 0x00
};


// Set 1Hz
static const uint8_t setFreq_1Hz[] = {
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
  0xE8, 0x03, 0x01, 0x00, 0x01, 0x00
};

// Set 0.5Hz
static const uint8_t setFreq_05Hz[] = {
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
  0xD0, 0x07, 0x01, 0x00, 0x01, 0x00
};




// DisableGRMC
static const uint8_t setNmea_disableGRMC[] = {
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0,
  0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

// EnableGRMC
static const uint8_t setNmea_enableGRMC[] = {
0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00
};


// disableGGSA
static const uint8_t setNmea_disableGGSA[] = {
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0,
  0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

// disableGGSA
static const uint8_t setNmea_enableGGSA[] = {
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0,
  0x02, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 
};

// disableGSV
static const uint8_t setNmea_disableGSV[] = {
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0,
  0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

// disableGSV
static const uint8_t setNmea_enableGSV[] = {
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0,
  0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 
};

// disableGSV
static const uint8_t setNmea_disableGLL[] = {
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0,
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

// enableGSV
static const uint8_t setNmea_enableGLL[] = {
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 
};

// disableVTG
static const uint8_t setNmea_disableVTG[] = {
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0,
  0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

// enableVTG
static const uint8_t setNmea_enableVTG[] = {
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0,
  0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 
};





void setup() {
  Serial.begin(9600);
  Serial.println(F("Start Setup"));
  serialPort[1]->begin(9600);
  configureGnss(serialPort[1], "5", false); //Serial port, Frequency, onlyGGA

  

}

void loop() {

    if (serialPort[1]->available()) {
    Serial.write(serialPort[1]->read());
  }
    if (Serial.available()) {
    serialPort[1]->write(Serial.read());
  }
 
}







void configureGnss(SoftwareSerial * serialPort, String freq, bool onlyGGA) {
  serialPort->listen();
  serialPort->flush();
  delay(50);
  sendUBX(setNavAir1G, sizeof(setNavAir1G)/sizeof(uint8_t), serialPort); // Set Airborne <1G Navigation Mode
  delay(50);
  sendUBX(setNmea4_1, sizeof(setNmea4_1)/sizeof(uint8_t), serialPort); // Set NMEA 4.1 (needed 4 Galileo)
  delay(50);
    if (freq == "1") {
      sendUBX(setFreq_1Hz, sizeof(setFreq_1Hz)/sizeof(uint8_t), serialPort); // Set GNSS to only GPS
      delay(50);
    }
  if (freq == "5") {
    sendUBX(setFreq_5Hz, sizeof(setFreq_5Hz)/sizeof(uint8_t), serialPort); // Set GNSS to only GPS
    delay(50);
  }
    if (freq == "0.5") {
    sendUBX(setFreq_05Hz, sizeof(setFreq_05Hz)/sizeof(uint8_t), serialPort); // Set GNSS to only GPS
    delay(50);
  }
  if (onlyGGA == true){
    sendUBX(setNmea_disableGRMC, sizeof(setNmea_enableGRMC)/sizeof(uint8_t), serialPort);
    delay(50);
    sendUBX(setNmea_disableGSV, sizeof(setNmea_enableGSV)/sizeof(uint8_t), serialPort);
    delay(50);
    sendUBX(setNmea_disableGLL, sizeof(setNmea_enableGLL)/sizeof(uint8_t), serialPort);
    delay(50);
    sendUBX(setNmea_disableVTG, sizeof(setNmea_enableGSV)/sizeof(uint8_t), serialPort);
    delay(50);
    sendUBX(setNmea_disableGGSA, sizeof(setNmea_enableGSV)/sizeof(uint8_t), serialPort);
    //missing ublox messages to add the rest of NMEA sentences
    }

  if (onlyGGA == false){
    sendUBX(setNmea_enableGRMC, sizeof(setNmea_enableGRMC)/sizeof(uint8_t), serialPort);
    delay(50);
    sendUBX(setNmea_enableGSV, sizeof(setNmea_enableGSV)/sizeof(uint8_t), serialPort);
    delay(50);
    sendUBX(setNmea_enableGLL, sizeof(setNmea_enableGLL)/sizeof(uint8_t), serialPort);
    delay(50);
    sendUBX(setNmea_enableVTG, sizeof(setNmea_enableGSV)/sizeof(uint8_t), serialPort);
    delay(50);
    sendUBX(setNmea_enableGGSA, sizeof(setNmea_enableGSV)/sizeof(uint8_t), serialPort);
    //missing ublox messages to add the rest of NMEA sentences
    }

}

void sendUBX(const uint8_t *message, const int len, SoftwareSerial * serialPort) {
  serialPort->listen();
  serialPort->flush();
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
