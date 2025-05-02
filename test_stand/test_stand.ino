/* Test Stand
RX: Receives commands from ground station and activates the desired valves
TX: Collects data from sensors and transmits them to the ground station
*/

#include <RH_RF95.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MCP9600.h>
#include "Adafruit_HX711.h"

// ========================================================================== //
//                                RADIO MODULE                                //
// ========================================================================== //

#define RFM95_CS   10               // CS pin
#define RFM95_RST  1                // RST pin
#define RFM95_INT  0                // G0 (Interupt)
#define RF95_FREQ 915.0             // Sets frequency
RH_RF95 rf95(RFM95_CS, RFM95_INT);  // RH_RF95 object

// Transmission timing variables
unsigned long lastSensorTransmit = 0;
const unsigned long sensorInterval = 1000; // Transmit every 1 second

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
//                             TEMPERATURE TRANSDUCER                         //
// ========================================================================== //

//  I2C Addresses for different thermocouple amplifiers
#define I2C_ADDRESS_TT1O (0x67)
#define I2C_ADDRESS_TT1P (0x66)
#define I2C_ADDRESS_TT1T (0x65)
#define I2C_ADDRESS_TT2T (0x64)
#define I2C_ADDRESS_TT3T (0x60)
// MCP9600 object for thermocouple sensor
Adafruit_MCP9600 TT1O, TT1P, TT1T, TT2T, TT3T;

void tt_setup() {
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
  // Set thermocouple type (K-type)
  TT1O.setThermocoupleType(MCP9600_TYPE_K);
  TT1P.setThermocoupleType(MCP9600_TYPE_K);
  TT1T.setThermocoupleType(MCP9600_TYPE_K);
  TT2T.setThermocoupleType(MCP9600_TYPE_K);
  TT3T.setThermocoupleType(MCP9600_TYPE_K);
}

// ========================================================================== //
//                                 LOAD CELL                                  //
// ========================================================================== //

// Pins for HX711 Communication
#define DATA_PIN 32
#define CLOCK_PIN 31

// A signle HX711 Object for load cell sensor
Adafruit_HX711 hx711(DATA_PIN, CLOCK_PIN);

void lc_setup(){
  hx711.begin();
  Serial.println("Tareing...");
  hx711.setGainA128();
  hx711.tare(10);
}

// ========================================================================== //
//                           PRESSURE TRANSDUCER                              //
// ========================================================================== //

#define PT1I_PIN 23  // A9
#define PT2I_PIN 22  // A8
#define PT3I_PIN 21  // A7
#define PT4I_PIN 16  // A6
#define PT5I_PIN 15  // A5
#define PT1N_PIN 41  // A17
#define PT1O_PIN 40  // A16
#define PT1P_PIN 39  // A15
#define PT1T_PIN 38  // A14

void pt_setup() {
  pinMode(PT1I_PIN, INPUT);
  pinMode(PT2I_PIN, INPUT);
  pinMode(PT3I_PIN, INPUT);
  pinMode(PT4I_PIN, INPUT);
  pinMode(PT5I_PIN, INPUT);
  pinMode(PT1N_PIN, INPUT);
  pinMode(PT1O_PIN, INPUT);
  pinMode(PT1P_PIN, INPUT);
  pinMode(PT1T_PIN, INPUT); 
}

// ========================================================================== //
//                                POWERED VALVES                              //
// ========================================================================== //

#define PV1N_PIN       3
#define PV1P_PIN       4
#define PV1T_PIN       5
#define BV1O_PIN       6
#define BV1T_PIN       7
#define BV1P_PIN       8
#define PV1O_1_PIN    24
#define PV1O_2_PIN    25
#define MBV1I_1_PIN   26
#define MBV1I_2_PIN   27

// State global variables
bool PV1N_State = false;
bool PV1P_State = false;
bool PV1T_State = false;
bool BV1O_State = false;
bool BV1T_State = false;
bool BV1P_State = false;
bool PV1O_1_State = false;
bool PV1O_2_State = false;
bool MBV1I_1_State = false;
bool MBV1I_2_State = false;

void pv_setup() {
  // Configure all valve pins as outputs
  pinMode(PV1N_PIN, OUTPUT);
  pinMode(PV1P_PIN, OUTPUT);
  pinMode(PV1T_PIN, OUTPUT);
  pinMode(BV1O_PIN, OUTPUT);
  pinMode(BV1T_PIN, OUTPUT);
  pinMode(BV1P_PIN, OUTPUT);
  pinMode(PV1O_1_PIN, OUTPUT);
  pinMode(PV1O_2_PIN, OUTPUT);
  pinMode(MBV1I_1_PIN, OUTPUT);
  pinMode(MBV1I_2_PIN, OUTPUT);

  // Ensure all valves start OFF
  digitalWrite(PV1N_PIN, LOW);
  digitalWrite(PV1P_PIN, LOW);
  digitalWrite(PV1T_PIN, LOW);
  digitalWrite(BV1O_PIN, LOW);
  digitalWrite(BV1T_PIN, LOW);
  digitalWrite(BV1P_PIN, LOW);
  digitalWrite(PV1O_1_PIN, LOW);
  digitalWrite(PV1O_2_PIN, LOW);
  digitalWrite(MBV1I_1_PIN, LOW);
  digitalWrite(MBV1I_2_PIN, LOW);
}

// ========================================================================== //
//                              HELPER FUNCTIONS                              //
// ========================================================================== //

/**
 * @brief Check for and return a received RF95 message as a trimmed String.
 * @return The received command string, or an empty string if nothing is received.
 */
String parse_radio_input() {
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

      return received;
    }
  }
  return "";
}

/**
 * @brief Toggle the named actuator ON or OFF based on "<name> ON"/" OFF".
 * @param received Command string (e.g. "PV1N ON").
 */
void eval_cmd(String received) {
  if (received.endsWith(" ON")) {
    String name = received.substring(0, received.length() - 3); // Extract name before " ON"

    if (name.equalsIgnoreCase("PV1N")) {
      PV1N_State = true;
    } else if (name.equalsIgnoreCase("PV1P")) {
      PV1P_State = true;
    } else if (name.equalsIgnoreCase("PV1T")) {
      PV1T_State = true;
    } else if (name.equalsIgnoreCase("BV1O")) {
      BV1O_State = true;
    } else if (name.equalsIgnoreCase("BV1T")) {
      BV1T_State = true;
    } else if (name.equalsIgnoreCase("BV1P")) {
      BV1P_State = true;
    } else if (name.equalsIgnoreCase("PV1O_1")) {
      PV1O_1_State = true;
    } else if (name.equalsIgnoreCase("PV1O_2")) {
      PV1O_2_State = true;
    } else if (name.equalsIgnoreCase("MBV1I_1")) {
      MBV1I_1_State = true;
    } else if (name.equalsIgnoreCase("MBV1I_2")) {
      MBV1I_2_State = true;
    } else {
      Serial.println("Unknown Power Valve name.");
    }
  } else if (received.endsWith(" OFF")) {
    String name = received.substring(0, received.length() - 4); // Extract name before " OFF"

    if (name.equalsIgnoreCase("PV1N")) {
      PV1N_State = false;
    } else if (name.equalsIgnoreCase("PV1P")) {
      PV1P_State = false;
    } else if (name.equalsIgnoreCase("PV1T")) {
      PV1T_State = false;
    } else if (name.equalsIgnoreCase("BV1O")) {
      BV1O_State = false;
    } else if (name.equalsIgnoreCase("BV1T")) {
      BV1T_State = false;
    } else if (name.equalsIgnoreCase("BV1P")) {
      BV1P_State = false;
    } else if (name.equalsIgnoreCase("PV1O_1")) {
      PV1O_1_State = false;
    } else if (name.equalsIgnoreCase("PV1O_2")) {
      PV1O_2_State = false;
    } else if (name.equalsIgnoreCase("MBV1I_1")) {
      MBV1I_1_State = false;
    } else if (name.equalsIgnoreCase("MBV1I_2")) {
      MBV1I_2_State = false;
    } else {
      Serial.println("Unknown Power Valve name.");
    }
  } else {
    Serial.println("Unrecognized command.");
  }
  update_pv_state();
}

/**
 * @brief Apply each actuator’s boolean state to its output pin.
 */
void update_pv_state() {
  digitalWrite(PV1N_PIN, PV1N_State ? HIGH : LOW);
  digitalWrite(PV1P_PIN, PV1P_State ? HIGH : LOW);
  digitalWrite(PV1T_PIN, PV1T_State ? HIGH : LOW);
  digitalWrite(BV1O_PIN, BV1O_State ? HIGH : LOW);
  digitalWrite(BV1T_PIN, BV1T_State ? HIGH : LOW);
  digitalWrite(BV1P_PIN, BV1P_State ? HIGH : LOW);
  digitalWrite(PV1O_1_PIN, PV1O_1_State ? HIGH : LOW);
  digitalWrite(PV1O_2_PIN, PV1O_2_State ? HIGH : LOW);
  digitalWrite(MBV1I_1_PIN, MBV1I_1_State ? HIGH : LOW);
  digitalWrite(MBV1I_2_PIN, MBV1I_2_State ? HIGH : LOW);
}

/**
 * @brief Once per interval, read all sensors and return their values as CSV.
 * @return Comma-separated timestamp and sensor readings.
 */
String read_sensor_data() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorTransmit >= sensorInterval) {
    lastSensorTransmit = currentMillis;
    
    // Read from Channel A with Gain 128, can also try CHAN_A_GAIN_64 or CHAN_B_GAIN_32
    int32_t lc1_val = hx711.readChannelBlocking(CHAN_A_GAIN_128);

    // Read PT values
    int pt1i_val = analogRead(PT1I_PIN);      // PT1-I
    int pt2i_val = analogRead(PT2I_PIN);      // PT2-I
    int pt3i_val = analogRead(PT3I_PIN);      // PT3-I
    int pt4i_val = analogRead(PT4I_PIN);      // PT4-I
    int pt5i_val = analogRead(PT5I_PIN);      // PT5-I
    int pt1n_val = analogRead(PT1N_PIN);     // PT1-N
    int pt1o_val = analogRead(PT1O_PIN);     // PT1-O
    int pt1p_val = analogRead(PT1P_PIN);     // PT1-P
    int pt1t_val = analogRead(PT1T_PIN);     // PT1-T

    // Read TT values
    float tt1o_temp = TT1O.readThermocouple();       // TT1-O
    float tt1p_temp = TT1P.readThermocouple();       // TT1-P
    float tt1t_temp = TT1T.readThermocouple();       // TT1-T
    float tt2t_temp = TT2T.readThermocouple();       // TT2-T
    float tt3t_temp = TT3T.readThermocouple();       // TT3-T
    
    // Build a sensor data string that includes a timestamp (in milliseconds)
    // Format: "TIME:<timestamp>,PT:<value>,TT:<temperature>..."
    String sensor_data = String(currentMillis) +
                       ", " + String(pt1i_val) +
                       ", " + String(pt2i_val) +
                       ", " + String(pt3i_val) +
                       ", " + String(pt4i_val) +
                       ", " + String(pt5i_val) +
                       ", " + String(pt1n_val) +
                       ", " + String(pt1o_val) +
                       ", " + String(pt1p_val) +
                       ", " + String(pt1t_val) +
                       ", " + String(tt1o_temp, 2) +
                       ", " + String(tt1p_temp, 2) +
                       ", " + String(tt1t_temp, 2) +
                       ", " + String(tt2t_temp, 2) +
                       ", " + String(tt3t_temp, 2) +
                       ", " + String(lc1_val);
    Serial.print("Sending sensor reading: ");
    Serial.println(sensor_data);
    return sensor_data;
  }
  return "";
}

// ========================================================================== //
//                              MAIN FUNCTIONS                                //
// ========================================================================== //

void setup() {
  // SERIAL MONITOR
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // wait up to five seconds
  Serial.println("Test Stand starting...");

  // Initalize I2C
  Wire.begin();

  // setup components
  radio_setup();
  tt_setup();
  lc_setup(); 
  pt_setup();
  pv_setup();

}

void loop() {
  // RX: Receive incoming radio commands
  String cmd = parse_radio_input();
  if (cmd.length()) {
    eval_cmd(cmd);
  }
  
  // TX: Transmit sensor data
  String data = read_sensor_data();
  if (data.length()) {
    rf95.send((uint8_t*)data.c_str(), data.length());
    rf95.waitPacketSent();
    Serial.println("Sensor packet sent.");
  }
}
