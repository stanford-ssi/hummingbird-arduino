#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_MCP9600.h>

#define RFM95_CS   10    // Chip Select pin for LoRa
#define RFM95_RST  1     // Reset pin for LoRa
#define RFM95_INT  0     // Interrupt pin for LoRa

#define RF95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Sensor pins and objects
const int analogSensorPin = 23;  // Analog sensor (Teensy 4.1 A9)
const int ledPin = 33;           // LED pin for status
bool ledState = false;

Adafruit_MCP9600 mcp;  // MCP9600 thermocouple sensor instance

// Transmission timing variables
unsigned long lastSensorTransmit = 0;
const unsigned long sensorInterval = 1000; // Transmit every 1 second

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);  // Wait up to 5 seconds for Serial
  Serial.println("LoRa Satellite starting...");

  // Setup sensor and status pins
  pinMode(analogSensorPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);

  // Initialize I2C for the MCP9600 sensor
  Wire.begin();

  // Initialize the MCP9600 sensor
  if (!mcp.begin()) {
    Serial.println("MCP9600 not found. Check wiring!");
    while (1);
  }
  // Set thermocouple type (K-type)
  mcp.setThermocoupleType(MCP9600_TYPE_K);

  // Reset LoRa module
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
  Serial.println("LoRa Satellite init succeeded.");
}

void loop() {
  // Check for incoming LoRa commands (non-blocking)
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      String received = "";
      for (uint8_t i = 0; i < len; i++) {
        received += (char)buf[i];
      }
      received.trim();
      Serial.print("Received command: ");
      Serial.println(received);
      
      // Process command to control LED
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
  
  // Update LED state
  digitalWrite(ledPin, ledState ? HIGH : LOW);
  
  // Transmit sensor data every sensorInterval milliseconds
  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorTransmit >= sensorInterval) {
    lastSensorTransmit = currentMillis;
    
    // Read analog sensor value and thermocouple temperature
    int analogValue = analogRead(analogSensorPin);
    float thermocoupleTemp = mcp.readThermocouple();
    
    // Build a sensor data string that includes a timestamp (in milliseconds)
    // Format: "TIME:<timestamp>,ANALOG:<value>,TC:<temperature>"
    String sensorStr = "TIME:" + String(currentMillis) +
                       ",ANALOG:" + String(analogValue) +
                       ",TC:" + String(thermocoupleTemp, 2);
    
    // Print the sensor data for debugging/sanity check
    Serial.print("Sending sensor reading: ");
    Serial.println(sensorStr);
    
    // Transmit the sensor data string via LoRa
    rf95.send((uint8_t *)sensorStr.c_str(), sensorStr.length());
    rf95.waitPacketSent();
  }
}
