// This program receives a duty cycle input and sets the PWM accordingly

const int mbv1 = 28; 
const int mbv2 = 29;
int mbv1_state, mbv2_state = 0; // Between 0 - 100% duty cycle

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 5000);  // Wait up to 5 seconds for Serial
  Serial.println("Test Stand starting...");

  // Initalize pins
  pinMode(mbv1, OUTPUT);
  pinMode(mbv2, OUTPUT);

}

void loop() {
  if (Serial.available() > 0) {
    byte inByte = Serial.read();   // input format ex. "A 80"
    if (inByte == 'A') mbv1_state = Serial.parseInt();
    if (inByte == 'B') mbv2_state = Serial.parseInt();
  }

  // Convert percentage to base 255
  mbv1_state = 255 * mbv1_state / 100;
  mbv2_state = 255 * mbv2_state / 100;

  analogWrite(mbv1, mbv1_state);
  analogWrite(mbv2, mbv2_state);
}
