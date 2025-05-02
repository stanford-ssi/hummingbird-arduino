/* Ground Station
Tx: Sends commands to the test stand, telling it to activate certain valves
Rx: Receives sensor reading from the test stand and stores that data into the SD card
*/

#include <RH_RF95.h>
#include <SPI.h>
#include <SD.h>

// ========================================================================== //
//                                RADIO MODULE                                //
// ========================================================================== //

#define RFM95_CS   10               // CS pin
#define RFM95_RST  1                // RST pin
#define RFM95_INT  0                // G0 (Interupt)
#define RF95_FREQ 915.0             // Sets frequency
RH_RF95 rf95(RFM95_CS, RFM95_INT);  // RH_RF95 object

void radio_setup() {
  // Set pins for, and reset, radio module
  SPI.begin();
  pinMode(RFM95_RST, OUTPUT);
  pinMode(RFM95_INT, INPUT); 

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
}

// ========================================================================== //
//                                 SD CARD                                    //
// ========================================================================== //

// File object for SD card logging
#define FILENAME "data-T2-040425.txt"
File dataFile;

void sd_setup() {
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD Card initialization failed!");
    while (1);
  }
  Serial.println("SD Card initialized.");

  // Remove any old log, then write a fresh header
  SD.remove(FILENAME);
  dataFile = SD.open(FILENAME, FILE_WRITE);
  dataFile.println(
    "TIME, "
    "PT1I, PT2I, PT3I, PT4I, PT5I, PT1N, PT1O, PT1P, PT1T, "
    "TT1O, TT1P, TT1T, TT2T, TT3T, "
    "LC1, "
    "PV1N, PV1P, PV1T, BV1O, BV1T, BV1P, PV1O_1, PV1O_2, MBV1I_1, MBV1I_2"
  );
  dataFile.close();
}


// ========================================================================== //
//                              MAIN FUNCTIONS                                //
// ========================================================================== //

void setup() {
  // Initalize serial monitor
  Serial.begin(9600);
  while (!Serial && millis() < 5000);
  Serial.println("Ground Station starting...");

  sd_setup();
  radio_setup();

}

void loop() {
  // TX: Check for Serial input to send commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() > 0) {
      Serial.print("Sending command: ");
      Serial.println(command);
      // sends command to test stand
      rf95.send((uint8_t *)command.c_str(), command.length());
      rf95.waitPacketSent();
    }
  }
  
  // RX: Check for incoming sensor readings + actuator states (with 100ms timeout)
  if (rf95.waitAvailableTimeout(100)) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {

      // Process incoming data package into string
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
        dataFile.println(received);
        dataFile.close();
        Serial.println("Data written to SD card.");
      } else {
        Serial.println("Error opening data.txt on SD card!");
      }

    }
  }
}
