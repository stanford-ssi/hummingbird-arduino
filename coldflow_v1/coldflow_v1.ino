// Simple Teensy 4.1 PT + Ball Valve Test
// Type 'O' to open the MBV, 'C' to close the MBV

// Pin definitons
const int ptPin = 23;
const int valvePinBlack = 29;
const int valvePinPurple = 30;

// Variables for controlling the MBV
bool valveState = false;
bool pulsing = false;
unsigned long pulseStartTime = 0;
const unsigned long pulseDuration = 500;

// Variables for collecting data from PT
unsigned long lastRead = 0;
const unsigned long readInterval = 1000;

void setup() {
  Serial.begin(9600);
  pinMode(ptPin, INPUT);
  pinMode(valvePinBlack, OUTPUT);
  digitalWrite(valvePinBlack, LOW);
  pinMode(valvePinPurple, OUTPUT);
  digitalWrite(valvePinPurple, LOW);
  Serial.println("Simple PT + Valve Test Initialized.");
}

void pulseValve(int pin) {
  digitalWrite(pin, HIGH);
  pulseStartTime = millis();
  pulsing = true;
}

void loop() {
  // Handle serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    
    if (!pulsing) {
      if (cmd == "C") {
        digitalWrite(valvePinPurple, LOW);
        pulseValve(valvePinBlack);
        valveState = false;
        Serial.println("MAIN closed.");
      } else if (cmd == "O") {
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

  // Handle turning off pulsed valve after delay
  if (pulsing && (millis() - pulseStartTime >= pulseDuration)) {
    digitalWrite(valvePinBlack, LOW);
    digitalWrite(valvePinPurple, LOW);
    pulsing = false;
  }

  // Periodically read and print pressure
  if (millis() - lastRead >= readInterval) {
    lastRead = millis();
    int adc = analogRead(ptPin);
    float voltage = (adc / 1023.0) * 3.3;

    // float pressure_psi = ((voltage - 1.0) / 4.0) * 2500.0;

    // Using equation from calibration tests
    // Pressure Gauge (psi) = 2.9599 * PT (10-bit reading) + -686.0201
    float pressure_psi = 2.9599 * adc - 686.0201

    if (pressure_psi < 0) {
      pressure_psi = 0;
    }

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