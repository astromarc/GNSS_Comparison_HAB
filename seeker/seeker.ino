#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#define OLED_WIDTH 128
#define OLED_HEIGHT 32

#define OLED_ADDR   0x3C

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT);


#include <SoftwareSerial.h>

SoftwareSerial mySerial(7, 6); // RX, TX
String mssgrec = "Waiting to receive first Message";
String mssg = "Waiting to receive first Message";
bool received = false;
int counter = 0;

void setup() {

  Serial.begin(19200);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.println("Goodnight moon!");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  mySerial.println("Init Relayer");

  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Starting...");
  display.display();

  delay(500);
}

void loop() { // run over and over





  received = false;

  if (mySerial.available()) {

    while (mySerial.available()) {

      mssgrec = mySerial.readStringUntil('\r');

      if (mssgrec.length() > 3) {
        mssg = mssgrec;
        counter = 0;
        mySerial.println(mssg);
      }



    }


  }


  Serial.println(mssg);
  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(counter);
  display.setTextColor(WHITE);
  display.setCursor(0, 17);
  display.println(mssg);
  counter++;
  display.display();
  delay(1000);
}
