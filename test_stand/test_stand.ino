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

//  I2C Addresses for different thermocouple amplifiers
#define I2C_ADDRESS_TT1O (0x67)
#define I2C_ADDRESS_TT1P (0x66)
#define I2C_ADDRESS_TT1T (0x65)
#define I2C_ADDRESS_TT2T (0x64)
#define I2C_ADDRESS_TT3T (0x60)

// RH_RF95 object for radio module
#define RF95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// MCP9600 object for thermocouple sensor
Adafruit_MCP9600 mcp;
Adafruit_MCP9600 TT1O, TT1P, TT1T, TT2T, TT3T;

// Sensor and actuator pins
const int analogSensorPin1 = 23;  // Analog sensor on Teensy 4.1 A9 (in place of PT), used by PT1-I
const int analogSensorPin2 = 22;  // A8, used by PT2-I
const int analogSensorPin3 = 21;  // A7, used by PT3-I
const int analogSensorPin4 = 16;  // A6, used by PT4-I
const int analogSensorPin5 = 15;  // A5, used by PT5-I
const int analogSensorPin6 = 41;  // A17, used by PT1-N
const int analogSensorPin7 = 40;  // A16, used by PT1-O
const int analogSensorPin8 = 39;  // A15, used by PT1-P
const int analogSensorPin9 = 38;  // A14, used by PT1-T

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
  pinMode(analogSensorPin3, INPUT);
  pinMode(analogSensorPin4, INPUT);
  pinMode(analogSensorPin5, INPUT);
  pinMode(analogSensorPin6, INPUT);
  pinMode(analogSensorPin7, INPUT);
  pinMode(analogSensorPin8, INPUT);
  pinMode(analogSensorPin9, INPUT);
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

  // Initialize the five different Thermocouple Amplifier
  if (!TT1O.begin(I2C_ADDRESS_TT1O)) {
    Serial.println("TT1-O not found. Check wiring!");
    while (1);
  }
  if (!TT1P.begin(I2C_ADDRESS_TT1P)) {
    Serial.println("TT1-P not found. Check wiring!");
    while (1);
  }
  if (!TT1T.begin(I2C_ADDRESS_TT1T)) {
    Serial.println("TT1-T not found. Check wiring!");
    while (1);
  }
  if (!TT2T.begin(I2C_ADDRESS_TT2T)) {
    Serial.println("TT2-T not found. Check wiring!");
    while (1);
  }
  if (!TT3T.begin(I2C_ADDRESS_TT3T)) {
    Serial.println("TT3-T not found. Check wiring!");
    while (1);
  }

  TT1O.setThermocoupleType(MCP9600_TYPE_K);  // Set thermocouple type (K-type)
  TT1P.setThermocoupleType(MCP9600_TYPE_K);
  TT1T.setThermocoupleType(MCP9600_TYPE_K);
  TT2T.setThermocoupleType(MCP9600_TYPE_K);
  TT3T.setThermocoupleType(MCP9600_TYPE_K);  
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
    int pt1_val = analogRead(analogSensorPin1);      // PT1-I
    int pt2_val = analogRead(analogSensorPin2);      // PT2-I
    int pt3_val = analogRead(analogSensorPin3);      // PT3-I
    int pt4_val = analogRead(analogSensorPin4);      // PT4-I
    int pt5_val = analogRead(analogSensorPin5);      // PT5-I
    int pt1N_val = analogRead(analogSensorPin6);     // PT1-N
    int pt1O_val = analogRead(analogSensorPin7);     // PT1-O
    int pt1P_val = analogRead(analogSensorPin8);     // PT1-P
    int pt1T_val = analogRead(analogSensorPin9);     // PT1-T
    // float thermocoupleTemp = mcp.readThermocouple();    // Thermocouple
    float TT1O_temp = TT1O.readThermocouple();       // TT1-O
    float TT1P_temp = TT1P.readThermocouple();       // TT1-P
    float TT1T_temp = TT1T.readThermocouple();       // TT1-T
    float TT2T_temp = TT2T.readThermocouple();       // TT2-T
    float TT3T_temp = TT3T.readThermocouple();       // TT3-T
    
    // Build a sensor data string that includes a timestamp (in milliseconds)
    // Format: "TIME:<timestamp>,PT:<value>,TT:<temperature>"
    String sensorStr = String(currentMillis) +
                       ", " + String(pt1_val) +
                       ", " + String(pt2_val) +
                       ", " + String(pt3_val) +
                       ", " + String(pt4_val) +
                       ", " + String(pt5_val) +
                       ", " + String(pt1N_val) +
                       ", " + String(pt1O_val) +
                       ", " + String(pt1P_val) +
                       ", " + String(pt1T_val); // +
                       // ", " + String(thermocoupleTemp, 2);
                       ", " + String(TT1O_temp, 2);
                       ", " + String(TT1P_temp, 2);
                       ", " + String(TT1T_temp, 2);
                       ", " + String(TT2T_temp, 2);
                       ", " + String(TT3T_temp, 2);

    Serial.print("Sending sensor reading: ");
    Serial.println(sensorStr);
    
    // Transmit the sensor data string via radio
    rf95.send((uint8_t *)sensorStr.c_str(), sensorStr.length());
    rf95.waitPacketSent();
  }
}
