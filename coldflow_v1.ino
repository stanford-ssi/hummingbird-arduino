// Simple Teensy 4.1 PT + Ball Valve Test

const int ptPin = 23;      // Analog pin for pressure transducer
const int valvePin = 33;   // Digital pin for ball valve (using LED pin here)

bool valveState = false;   // false = closed, true = open
unsigned long lastRead = 0;
const unsigned long readInterval = 1000; // 1 second interval

void setup() {
  Serial.begin(9600);
  pinMode(ptPin, INPUT);
  pinMode(valvePin, OUTPUT);
  digitalWrite(valvePin, LOW); // Start with valve closed
  Serial.println("Simple PT + Valve Test Initialized.");
}

void loop() {
  // Handle serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    
    if (cmd == "OPEN") {
      valveState = true;
      Serial.println("Valve opened.");
    } else if (cmd == "CLOSE") {
      valveState = false;
      Serial.println("Valve closed.");
    } else {
      Serial.println("Unknown command. Use OPEN or CLOSE.");
    }
    digitalWrite(valvePin, valveState ? HIGH : LOW);
  }

  // Periodically read and print pressure
  if (millis() - lastRead >= readInterval) {
    lastRead = millis();
    int adc = analogRead(ptPin);
    float voltage = (adc / 1023.0) * 3.3;
    float pressure_psi = ((voltage - 1.0) / 4.0) * 2500.0;

    // Protect against readings below 1V (e.g. if sensor disconnected)
    if (pressure_psi < 0)
        pressure_psi = 0;
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
