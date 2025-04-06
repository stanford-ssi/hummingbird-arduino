/*
Test Stand
RX: Receives commands from ground station and activates the desired valves
TX: Collects data from sensors and transmits them to the ground station
*/

#include <RH_RF95.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MCP9600.h>
#include "Adafruit_HX711.h"

#define RFM95_CS   10     // CS pin for radio module
#define RFM95_RST  1      // RST pin for radio module
#define RFM95_INT  0      // G0 (Interupt) for radio module

// RH_RF95 object for radio module
#define RF95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// MCP9600 object for thermocouple sensor
Adafruit_MCP9600 mcp;

// Sensor and actuator pins
const int analogSensorPin1 = 23;  // Analog sensor on Teensy 4.1 A9 (in place of PT)
const int analogSensorPin2 = 22;
const int ledPin = 33;           // LED pin (in place of actuator)
bool ledState = false;

// Transmission timing variables
unsigned long lastSensorTransmit = 0;
const unsigned long sensorInterval = 1000; // Transmit every 1 second

// Pins for HX711 Communication
const uint8_t DATA_PIN = 2;
const uint8_t CLOCK_PIN = 3;

// HX711 Object for load cell sensor
Adafruit_HX711 hx711(DATA_PIN, CLOCK_PIN);

void init_hx711(){
  hx711.begin();
  Serial.println("Tareing...");

  for (uint8_t t=0; t<3; t++) {
    hx711.tareA(hx711.readChannelRaw(CHAN_A_GAIN_128));
    hx711.tareA(hx711.readChannelRaw(CHAN_A_GAIN_128));
    hx711.tareB(hx711.readChannelRaw(CHAN_B_GAIN_32));
    hx711.tareB(hx711.readChannelRaw(CHAN_B_GAIN_32));
  }
}

void hx711_loop(){
  // Read from Channel A with Gain 128, can also try CHAN_A_GAIN_64 or CHAN_B_GAIN_32
  // since the read is blocking this will not be more than 10 or 80 SPS (L or H switch)
  int32_t weightA128 = hx711.readChannelBlocking(CHAN_A_GAIN_128);
  Serial.print("Channel A (Gain 128): ");
  Serial.println(weightA128);

  // Read from Channel A with Gain 128, can also try CHAN_A_GAIN_64 or CHAN_B_GAIN_32
  int32_t weightB32 = hx711.readChannelBlocking(CHAN_B_GAIN_32);
  Serial.print("Channel B (Gain 32): ");
  Serial.println(weightB32);
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);  // Wait up to 5 seconds for Serial
  Serial.println("Test Stand starting...");

  // Initalize sensor and actuator pins
  pinMode(analogSensorPin1, INPUT);
  pinMode(analogSensorPin2, INPUT);
  pinMode(ledPin, OUTPUT);

  // Initialize the HX711 Sensor
  //init_hx711();

  // Initialize the MCP9600 sensor
  /*Wire.begin(); // Initalize I2C
  if (!mcp.begin()) {
    Serial.println("MCP9600 not found. Check wiring!");
    while (1);
  }
  mcp.setThermocoupleType(MCP9600_TYPE_K);  // Set thermocouple type (K-type)
  */
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
}

void loop() {
  // RX: Check for incoming radio commands (non-blocking)
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {

      // Proccess incoming command into a string
      String received = "";
      for (uint8_t i = 0; i < len; i++) {
        received += (char)buf[i];
      }
      received.trim();
      Serial.print("Received command: ");
      Serial.println(received);
      
      // Evaluate the command
      if (received.equalsIgnoreCase("ON")) {
        ledState = true;
        Serial.println("LED set to ON");
      } else if (received.equalsIgnoreCase("OFF")) {
        ledState = false;
        Serial.println("LED set to OFF");
      } else {
        Serial.println("Unrecognized command.");
      }

    }
  }
  
  // Update actuator state according to command
  digitalWrite(ledPin, ledState ? HIGH : LOW);
  
  // TX: Transmit sensor data every "sensorInterval" milliseconds
  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorTransmit >= sensorInterval) {
    lastSensorTransmit = currentMillis;
    
    // Read sensor values
    int pt1_val = analogRead(analogSensorPin1);      // Pressure trandsucer
    int pt2_val = analogRead(analogSensorPin2);      // Pressure trandsucer
    // float thermocoupleTemp = mcp.readThermocouple();    // Thermocouple
    
    // Build a sensor data string that includes a timestamp (in milliseconds)
    // Format: "TIME:<timestamp>,PT:<value>,TT:<temperature>"
    String sensorStr = String(currentMillis) +
                       ", " + String(pt1_val) +
                       ", " + String(pt2_val); // +
                       // ", " + String(thermocoupleTemp, 2);

    Serial.print("Sending sensor reading: ");
    Serial.println(sensorStr);
    
    // Transmit the sensor data string via radio
    rf95.send((uint8_t *)sensorStr.c_str(), sensorStr.length());
    rf95.waitPacketSent();
  }
}
