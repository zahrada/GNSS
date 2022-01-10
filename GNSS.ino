// libraries
#include <MKRGSM.h>
#include <Arduino.h>
#include "wiring_private.h"
#include <Wire.h> //Needed for I2C to GPS

#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GPS myGPS;

Uart mySerial (&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);


#include "arduino_secrets.h"
// Please enter your sensitive data in the Secret tab or arduino_secrets.h
// PIN Number
const char PINNUMBER[]     = "";
// APN data
const char GPRS_APN[]      = "internet.t-mobile.cz";
const char GPRS_LOGIN[]    = "";
const char GPRS_PASSWORD[] = "";

// initialize the library instance
GSMClient client;
GPRS gprs;
GSM gsmAccess;


// URL, path and port (for example: example.org)
char server[] = "czeposr.cuzk.cz"; //www.euref-ip.net
char path[] = "/CSUM3-MSM";//RTK3-MSM /CSUM3-MSM
int port = 2101; // port 80 is the default for HTTP 2101

byte gps_set_sucess = 0 ;

char nmea[82];
int size_nmea;

int red_light_pin = 2;
int green_light_pin = 3;
int blue_light_pin = 4;
int GNSS_status;


unsigned long timer = 0;


void setup() {
  //RGB LED
  pinMode(red_light_pin, OUTPUT);
  pinMode(green_light_pin, OUTPUT);
  pinMode(blue_light_pin, OUTPUT);

  //Serials
  Serial.begin(115200);
  Serial1.begin(115200);
  mySerial.begin(115200);
  pinPeripheral(1, PIO_SERCOM); //Assign RX function to pin 1
  pinPeripheral(0, PIO_SERCOM); //Assign TX function to pin 0

  delay(100);
/*
  //Read NMEA GGA message from ublox for NTRIP client
  for (int i = 0; i <= 5; i++) {
    if (Serial1.find('$GNGGA')) {
      size_nmea = Serial1.readBytesUntil('\n', nmea, 82);

      //Wait for GPS fix
      if (size_nmea < 80) {
        i = i - 1;
      }

      Serial.print("$GNGGA");
      for (int i = 0 ; i < size_nmea ; i++) {
        Serial.print(nmea[i]);
      }
      Serial.println();
      Serial.println(size_nmea);
    }

    delay(100);
  }
*/
  //Serial.end();


  //Ublox NMEA GGA msg connection
  Wire.begin();

  if (myGPS.begin() == false)
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  //This will pipe all NMEA sentences to the serial port so we can see them
  myGPS.setNMEAOutputPort(client);

  //GSM connection
  bool connected = false;
  // After starting the modem with GSM.begin()
  // attach the shield to the GPRS network with the APN, login and password
  while (!connected) {
    if ((gsmAccess.begin(PINNUMBER) == GSM_READY) &&
        (gprs.attachGPRS(GPRS_APN, GPRS_LOGIN, GPRS_PASSWORD) == GPRS_READY)) {
      connected = true;
    } else {
      Serial.println("Not connected");
      delay(1000);
    }
  }

  Serial.println("connecting...");

  // if you get a connection, report back via serial:
  if (client.connect(server, port)) {
    Serial.println("connected");
    // Make a HTTP request:
    client.print("GET ");
    client.print(path);
    client.println(" HTTP/1.0");
    client.print("Host: ");
    client.println(server);
    client.println("Ntrip-Version: Ntrip/1.0");
    client.println("User-Agent: NTRIP test");
    client.println("Authorization: Basic Y3Z1dHZ5dWthOmsxNTVkcmVtZWpha29rb25l");
    //client.println("Connection: close");
    client.println();
    /*
    client.print("$GNGGA");
    for (int i = 0 ; i < size_nmea ; i++) {
      client.print(nmea[i]);
    }
    */
    /*
     * myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.
     */
  }



  // Setting ublox by register
  /*
    byte setGLL[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B, 0x1A
    };
    mySerial.write(setGLL, sizeof(setGLL));

    byte setGLL[] = {0xb5, 0x62, 0x06, 0x8a, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xca, 0x00, 0x91, 0x20, 0x00, 0x15, 0x16};
    mySerial.write(setGLL, sizeof(setGLL));

    byte setGNS[] = {0xb5, 0x62, 0x06, 0x8a, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xb6, 0x00, 0x91, 0x20, 0x00, 0x01, 0xb2, 0x35};
    mySerial.write(setGNS, sizeof(setGNS));

    byte setVTG[] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xB1, 0x00, 0x91, 0x20, 0x00, 0xFC, 0x99, 0x58};
    mySerial.write(setVTG, sizeof(setVTG));


    byte setGSA[] = {0xb5, 0x62, 0x06, 0x8a, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xc0, 0x00, 0x91, 0x20, 0x00, 0x0b, 0xe4}; //checksum EF
    Serial.write(setGSA, sizeof(setGSA));
  */
}

void loop() {

  GNSS_status =  nmea[42];
  led_gnss_status(GNSS_status);

  if(Serial1.available()){
    Serial.write(Serial1.read());
  }
/*
  //Send NMEA GGA message to NTRIP
  if (millis() - timer > 20000) {
    Serial1.begin(115200);
    
    //Read NMEA GGS message from ublox
    if (Serial1.find('$GNGGA')) {
      size_nmea = Serial1.readBytesUntil('\n', nmea, 82);
    }
    
    client.print("$GNGGA");
    for (int i = 0 ; i < size_nmea ; i++) {
      client.print(nmea[i]);
    }
    
    Serial.println();
    timer = millis();
    Serial1.end();
  }
*/
/*
  //Send RTCM data from NTRIP to ublox
  while (client.available()) {
    byte x = client.read();
    //mySerial.write(x);
    //Serial.write(x);
  }
*/
}

// Attach the interrupt handler to the SERCOM
void SERCOM3_Handler()
{
  mySerial.IrqHandler();
}

void RGB_color(int red_light_value, int green_light_value, int blue_light_value)
{
  analogWrite(red_light_pin, red_light_value);
  analogWrite(green_light_pin, green_light_value);
  analogWrite(blue_light_pin, blue_light_value);
}

void led_gnss_status(byte GNSS_status) {
  if (GNSS_status == 48) {
    RGB_color(255, 0, 0); // Red
  }
  else if (GNSS_status == 49) {
    RGB_color(0, 255, 0); // Greenn
  }
  else if (GNSS_status == 50) {
    RGB_color(0, 0, 255); // Blue
  }
  else if (GNSS_status == 51) {
    RGB_color(255, 255, 125); // Raspberry
  }
  else if (GNSS_status == 53) {
    RGB_color(0, 255, 255); // Cyan
  }
  else if (GNSS_status == 54) {
    RGB_color(255, 0, 255); // Magenta
  }
  else if (GNSS_status == 55) {
    RGB_color(255, 255, 0); // Yellow
  }
  else if (GNSS_status == 56) {
    RGB_color(255, 255, 255); // White
  }
}
