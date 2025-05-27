// Simple Teensy 4.1 PT + Ball Valve Test with LoRa RX/TX
// Receives 'O' and 'C' commands via LoRa and transmits pressure readings

#include <RH_RF95.h>
#include <SPI.h>

// LoRa definitions
#define RFM95_CS   10
#define RFM95_RST  1
#define RFM95_INT  0
#define RF95_FREQ  915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Pin definitions
const int ptPin = 23;
const int valvePinBlack = 29;
const int valvePinPurple = 30;

// Valve state
bool valveState = false;
bool pulsing = false;
unsigned long pulseStartTime = 0;
const unsigned long pulseDuration = 500;

// Sensor reading interval
unsigned long lastRead = 0;
const unsigned long readInterval = 1000;

void setup() {
  Serial.begin(9600);

  // Valve pin setup
  pinMode(ptPin, INPUT);
  pinMode(valvePinBlack, OUTPUT);
  digitalWrite(valvePinBlack, LOW);
  pinMode(valvePinPurple, OUTPUT);
  digitalWrite(valvePinPurple, LOW);

  // Radio reset and init
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("LoRa init failed");
    while (1);
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("LoRa set frequency failed");
    while (1);
  }
  rf95.setTxPower(23, false);
  Serial.println("Simple PT + Valve LoRa Test Initialized");
}

void pulseValve(int pin) {
  digitalWrite(pin, HIGH);
  pulseStartTime = millis();
  pulsing = true;
}

void loop() {
  // Receive LoRa command
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      String cmd = "";
      for (uint8_t i = 0; i < len; i++) {
        cmd += (char)buf[i];
      }
      cmd.trim();

      Serial.println(cmd);

      if (!pulsing) {
        if (cmd == "c") {
          digitalWrite(valvePinPurple, LOW);
          pulseValve(valvePinBlack);
          valveState = false;
          Serial.println("MAIN closed.");
        } else if (cmd == "o") {
          digitalWrite(valvePinBlack, LOW);
          pulseValve(valvePinPurple);
          valveState = true;
          Serial.println("MAIN opened.");
        } else {
          Serial.println("Unknown command.");
        }
      } else {
        Serial.println("Pulse in progress. Wait.");
      }
    }
  }

  // End pulse after duration
  if (pulsing && (millis() - pulseStartTime >= pulseDuration)) {
    digitalWrite(valvePinBlack, LOW);
    digitalWrite(valvePinPurple, LOW);
    pulsing = false;
  }

  // Send PT reading
  if (millis() - lastRead >= readInterval) {
    lastRead = millis();
    int adc = analogRead(ptPin);
    float voltage = (adc / 1023.0) * 3.3;

    // From calibration: Pressure (psi) = 2.9599 * ADC - 686.0201
    float pressure_psi = 2.9599 * adc - 686.0201;
    if (pressure_psi < 0) pressure_psi = 0;

    // Construct and send CSV-formatted message
    String msg = String(millis()) + "," + String(pressure_psi, 1);
    rf95.send((uint8_t *)msg.c_str(), msg.length());
    rf95.waitPacketSent();

    // Also print locally
    Serial.print("PT Reading: ");
    Serial.print(adc);
    Serial.print(" (");
    Serial.print(voltage, 3);
    Serial.print(" V) → ");
    Serial.print(pressure_psi, 1);
    Serial.println(" psi");
    Serial.print(" | Valve State: ");
    Serial.println(valveState ? "OPEN" : "CLOSED");
  }
}
