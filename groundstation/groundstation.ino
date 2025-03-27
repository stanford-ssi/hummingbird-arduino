#include <SPI.h>
#include <SD.h>
#include <RH_RF95.h>

#define RFM95_CS   10    // Chip Select pin for LoRa
#define RFM95_RST  1     // Reset pin for LoRa
#define RFM95_INT  0     // Interrupt pin for LoRa

#define RF95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

File dataFile; // File object for SD card logging

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);  // Wait for Serial up to 5 seconds
  Serial.println("LoRa Ground Station starting...");

  // Initialize the built-in SD card
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD Card initialization failed!");
  } else {
    Serial.println("SD Card initialized.");
  }

  // Reset LoRa module
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Initialize LoRa module
  if (!rf95.init()){
    Serial.println("LoRa init failed!");
    while (1);
  }
  if (!rf95.setFrequency(RF95_FREQ)){
    Serial.println("setFrequency failed!");
    while (1);
  }
  rf95.setTxPower(23, false);
  Serial.println("LoRa Ground Station init succeeded.");
}

void loop() {
  // Check for Serial input to send commands
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
  
  // Check for incoming sensor readings (with 100ms timeout)
  if (rf95.waitAvailableTimeout(100)) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      String received = "";
      for (uint8_t i = 0; i < len; i++) {
        received += (char)buf[i];
      }
      received.trim();
      // Print the complete message from the satellite (should include timestamp)
      Serial.print("Received sensor reading: ");
      Serial.println(received);
      
      // Optionally, prepend a local timestamp before logging
      String logEntry = "LocalTime:" + String(millis()) + " - " + received;
      
      // Log the data to SD card
      dataFile = SD.open("data.txt", FILE_WRITE);
      if (dataFile) {
        dataFile.println(logEntry);
        dataFile.close();
        Serial.println("Data written to SD card.");
      } else {
        Serial.println("Error opening data.txt on SD card!");
      }
    }
  }
}
