// Simple Teensy 4.1 PT + Ball Valve Test with LoRa RX/TX
// Receives 'O' and 'C' commands via LoRa and transmits pressure readings

#include <RH_RF95.h>
#include <SPI.h>
#include <Adafruit_MCP9600.h>
#include <Adafruit_HX711.h>

// LoRa definitions
#define RFM95_CS   10
#define RFM95_RST  1
#define RFM95_INT  0
#define RF95_FREQ  915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Sensor & actuator Definitions

/* 
 * PTs
 * 4 PTs used in WK10 cold flow test. Uses Teensy built-in ADC. Connects to analog pins.
 */
const int PT1_N = 23; // original one also used in previous test
const int PT1_O = 22; // PT on oxidizer line
const int PT1_T = 21; // on the top of the tank
const int PT2_I = 20; // at the bottom of the tank

// PT reading interval
unsigned long lastRead = 0;
const unsigned long readInterval = 1000;

// PT setup
void pt_setup() {
  pinMode(PT1_N, INPUT);
  pinMode(PT1_O, INPUT);
  pinMode(PT1_T, INPUT);
  pinMode(PT2_I, INPUT);
}

/*
 * TTs
 * 3 TTs used in Wk10 cold flow test. Uses I2C module and connects to SCL, SDA
 */

// 3 TTs - will have to check actual TT I2C address
#define I2C_ADDRESS_TT1O (0x67)
#define I2C_ADDRESS_TT1T (0x65)
#define I2C_ADDRESS_TT2T (0x64)
// MCP9600 object for thermocouple sensor
Adafruit_MCP9600 TT1O, TT1T, TT2T;

void tt_setup() {
  // Initialize the five different Thermocouple Amplifier
  if (!TT1O.begin(I2C_ADDRESS_TT1O)) {
    Serial.println("TT1-O not found. Check wiring!");
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
  // Set thermocouple type (K-type)
  TT1O.setThermocoupleType(MCP9600_TYPE_K);
  TT1T.setThermocoupleType(MCP9600_TYPE_K);
  TT2T.setThermocoupleType(MCP9600_TYPE_K);
}

/*
 * LC
 * Uses pseudo-I2C (TX/RX). Connected through module.
 */

 // Pins for HX711 Communication
#define DATA_PIN 32
#define CLOCK_PIN 31

// A signle HX711 Object for load cell sensor
Adafruit_HX711 hx711(DATA_PIN, CLOCK_PIN);

void lc_setup(){
  hx711.begin();

  // read and toss 3 values each
  Serial.println("Tareing....");
  for (uint8_t t=0; t<3; t++) {
    hx711.tareA(hx711.readChannelRaw(CHAN_A_GAIN_128));
    hx711.tareA(hx711.readChannelRaw(CHAN_A_GAIN_128));
    hx711.tareB(hx711.readChannelRaw(CHAN_B_GAIN_32));
    hx711.tareB(hx711.readChannelRaw(CHAN_B_GAIN_32));
  }
}

/*
 * MBV
 * only 1 used for Wk10 cold flow test.
 * Uses 2 digital pins. One pin will be driven high or low, the other will pulse.
 */
const int valvePinBlack = 29;
const int valvePinPurple = 30;

// MBV Valve state
bool mbvValveState = false;
bool pulsing = false;
unsigned long pulseStartTime = 0;
const unsigned long pulseDuration = 500;

/*
 * PV 
 * 2 used for WK10 cold flow test. Uses digital pins to be driven high or low.
 */
const int PV1_T = 16; // tank bleed valve
const int PV1_O = 15; // oxidizer valve

// PV valve state
bool PV1_T_valveState = false;
bool PV1_O_valveState = false;

// setup function for MBV and PV
void pv_setup() {
  // Configure all power valve pins as outputs
  pinMode(PV1_T, OUTPUT);
  pinMode(PV1_O, OUTPUT);
  digitalWrite(PV1_T, LOW);
  digitalWrite(PV1_O, LOW);

  // configure MBV pins
  pinMode(valvePinBlack, OUTPUT);
  digitalWrite(valvePinBlack, LOW);
  pinMode(valvePinPurple, OUTPUT);
  digitalWrite(valvePinPurple, LOW);
}

void setup() {
  Serial.begin(9600);

  pt_setup();
  tt_setup();
  lc_setup();
  pv_setup();

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

float convert2psi(int adc_reading) {
  // From calibration: Pressure (psi) = 2.9599 * ADC - 686.0201
    float pressure_psi = 2.9599 * adc_reading - 686.0201;
    if (pressure_psi < 0) pressure_psi = 0;

    return pressure_psi;
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

      /*
       *  06/01 (Sam C.): flipped boolean order and digitalWrites so now should match actual valve behavior.
       *  also modified logic so now number key 1 controls MBV, 2 controls PV1_T, 3 controls PV1_O.
       *  All are toggles.
       */
      if (!pulsing) {
        if (cmd == "1" && mbvValveState == false) {
          digitalWrite(valvePinBlack, LOW);
          pulseValve(valvePinPurple);
          mbvValveState = true;
          Serial.println("MBV closed.");
        } else if (cmd == "1" && mbvValveState == true) {
          digitalWrite(valvePinPurple, LOW);
          pulseValve(valvePinBlack);
          mbvValveState = false;
          Serial.println("MBV opened.");
        } else {
          Serial.println("Unknown command.");
        }
      } else {
        Serial.println("Pulse in progress. Wait.");
      }

      // control logic for other PVs
      if (cmd == "2" && PV1_T_valveState == false) {
        digitalWrite(PV1_T, HIGH);
        PV1_T_valveState = true;
        Serial.println("PV1_T opened");
      } else if (cmd == "2" && PV1_T_valveState == true) {
        digitalWrite(PV1_T, LOW);
        PV1_T_valveState = false;
        Serial.println("PV1_T closed");
      }

      if (cmd == "3" && PV1_O_valveState == false) {
        digitalWrite(PV1_O, HIGH);
        PV1_O_valveState = true;
        Serial.println("PV1_O opened");
      } else if (cmd == "3" && PV1_O_valveState == true) {
        digitalWrite(PV1_O, LOW);
        PV1_O_valveState = false;
        Serial.println("PV1_O closed");
      }
    }
  }

  // End pulse after duration
  if (pulsing && (millis() - pulseStartTime >= pulseDuration)) {
    digitalWrite(valvePinBlack, LOW);
    digitalWrite(valvePinPurple, LOW);
    pulsing = false;
  }

  // Send sensor reading
  if (millis() - lastRead >= readInterval) {
    lastRead = millis();
    // Read from Channel A with Gain 128, can also try CHAN_A_GAIN_64 or CHAN_B_GAIN_32
    int32_t lc1_val = hx711.readChannelBlocking(CHAN_A_GAIN_128);

    // PT readings from analog pins
    int PT1_N_reading = analogRead(PT1_N);
    int PT1_O_reading = analogRead(PT1_O);
    int PT1_T_reading = analogRead(PT1_T);
    int PT2_I_reading = analogRead(PT2_I);

    // TT readings from I2C
    float tt1o_temp = TT1O.readThermocouple();       // TT1-O
    float tt1t_temp = TT1T.readThermocouple();       // TT1-T
    float tt2t_temp = TT2T.readThermocouple();       // TT2-T

    // deal with later: float voltage = (adc / 1023.0) * 3.3;

    // conversion to PSI done in function

    // Construct and send CSV-formatted message
    String msg = "Time: " + String(millis()) + "\n"
                    + "PT1_N: " + String(convert2psi(PT1_N_reading), 1) + " psi | PT1_O: " + String(convert2psi(PT1_O_reading), 1) + " psi | PT1_T: " + String(convert2psi(PT1_T_reading), 1) + " psi | PT2_I: " + String(convert2psi(PT2_I_reading), 1) + "psi\n"
                    + "TT1_O: " + String(tt1o_temp,1) + " C | TT1_T: " + String(tt1t_temp) + " C | TT2_T: " + String(tt2t_temp) + "C \n"
                    + "LC: " + String(lc1_val) + " (unit) \n";
    rf95.send((uint8_t *)msg.c_str(), msg.length());
    rf95.waitPacketSent();

    // Also print locally
    Serial.print(msg);
    Serial.print("MBV State: ");
    Serial.println(mbvValveState ? "OPEN" : "CLOSED");
    Serial.print("PV1_T State: ");
    Serial.println(PV1_T_valveState ? "OPEN" : "CLOSED");
    Serial.print("PV1_O State: ");
    Serial.println(PV1_O_valveState ? "OPEN" : "CLOSED");
  }
}

/*
Readings:
Time: 
PT1_N: XXX psi | PT1_O: XXX psi | PT1_T: XXX psi | PT2_I: XXX psi \n
TT1_O: XXX C  | TT1_T: XXX C  | TT2_T: XXX C \n
LC: XXX
*/
