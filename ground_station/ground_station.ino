/*
Ground Station
Tx: Sends commands to the test stand, telling it to activate certain valves
Rx: Receives sensor reading from the test stand and stores that data into the SD card
*/

#include <RH_RF95.h>
#include <SPI.h>
#include <SD.h>

#define RFM95_CS   10     // CS pin for radio module
#define RFM95_RST  1      // RST pin for radio module
#define RFM95_INT  0      // G0 (Interupt) for radio module

// RH_RF95 object for radio module
#define RF95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// File object for SD card logging
File dataFile;
#define FILENAME "data-T2-040425.txt"

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);  // Wait for Serial up to 5 seconds
  Serial.println("Ground Station starting...");

  // Initialize the built-in SD card
  /*if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD Card initialization failed!");
  } else {
    Serial.println("SD Card initialized.");
  }*/

  // Set pins for, and reset, radio module
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Initialize radio module
  if (!rf95.init()){
    Serial.println("Radio init failed!");
    while (1);
  }
  if (!rf95.setFrequency(RF95_FREQ)){
    Serial.println("setFrequency failed!");
    while (1);
  }
  rf95.setTxPower(23, false);
  Serial.println("Radio init succeeded.");

  // create SD header
  dataFile = SD.open(FILENAME, FILE_WRITE); // file where data will be stored
  dataFile.println("Hydrostatic Testing Test Result");
  dataFile.println("*************************");
  dataFile.println("Time (ms),PT1,Solenoid 1,Solenoid 2");
  dataFile.close();

  // Serial println default message
  Serial.println("Message form: [Solenoid #] [\"ON\" or \"OFF\"]");
}

void loop() {
  // TX: Check for Serial input to send commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() > 0) {
      Serial.print("Sending command: ");
      Serial.println(command);
      rf95.send((uint8_t *)command.c_str(), command.length());
      rf95.waitPacketSent();
    }
  }
  
  // RX: Check for incoming sensor readings (with 100ms timeout)
  if (rf95.waitAvailableTimeout(100)) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {

      // Process incoming sensor data into string
      String received = "";
      for (uint8_t i = 0; i < len; i++) {
        received += (char)buf[i];
      }
      received.trim();
      Serial.print("Received sensor reading: ");
      Serial.println(received);
      
      // Log the sensor data to the SD card
      dataFile = SD.open(FILENAME, FILE_WRITE); // file where data will be stored
      if (dataFile) {
        dataFile.print(millis());
        dataFile.print(",");
        dataFile.println(received);
        dataFile.close();
        Serial.println("Data written to SD card.");
      } else {
        Serial.println("Error opening data.txt on SD card!");
      }
    }
  }
}
