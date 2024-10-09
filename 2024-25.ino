// Libraries
#include <Wire.h> //Default library (uses pinMode, etc.)
#include <string.h> //Default library (for printing strings)
#include <LiquidCrystal_I2C.h> //LiquidCrystal I2C by Frank de Brabander
#include <Servo.h> //PWMServo by Jim Studt et al.

//  Define pin names
//  Analog pins
#define VOLTAGE_SENSOR A0
//  PWM pins
#define BRAKE_CONTROL 4
#define PITCH_CONTROL 5
//  Other pins
#define POWER_SWITCH_RELAY 28
#define E_BUTTON 44

// LCD variables
LiquidCrystal_I2C lcd(0x3f, 20, 4)
String powerSource = "Ext";

// Variable for timing / LCD Update (ms)
unsigned long previousMillis = 0;
#define UPDATE_INTERVAL 1000

// Linear actuator constants
// For the PQ12-R, 1000 is fully extended and 2000 is fully retracted.
#define INITIAL_PITCH 1300
#define MINIMUM_PITCH 1800
#define MAXIMUM_PITCH 1300
#define BRAKE_DISENGAGED 1450
#define BRAKE_ENGAGED 1225

// Resistor bank constants
#define LOAD_UNSHORTED 0
#define MINIMUM_RESISTANCE 0

//Enum for the substates within the overall state machine
typedef enum {
  restart,
  power_curve,
  steady_power,
  survival,
  emergency_stop,
  test
} operatingStateMachine;

//Variables to keep track of current and previous state
operatingStateMachine operatingState = restart;
operatingStateMachine previousState = restart;

//Enum for the substates within the "test" state of the overall state machine
typedef enum {
  test_select,
  power_select,
  brake,
  pitch,
  pitot_tube_calibration,
  pitot_tube_measurement,
  read_voltage,
  read_power,
  read_rpm,
  set_load,
  tune_load,
  short_load,
  read_load,
  emergency_button,
  load_disconnect,
  survival_test,
  steady_power_test,
  power_curve_test,
  exit_test
} testStateMachine;

//Variable to keep track of current test state
testStateMachine testState = test_select;

void setup() {
  Serial.begin(9600);  // Set baud rate to 9600 bit/s

  //  Setup functions
  SetupPins();
  Serial.println("Pins set up");
  SetupLCD();
  Serial.println("LCD set up");
  lcd.setCursor(0, 0);  // Set cursor to top left corner
  lcd.print("Starting up...");
  SetupServos();
  Serial.println("Servo set up");
  SetupLoad();
  Serial.println("Load set up");

  delay(3000); // Wait 3 seconds for linear actuators to reach correct positions

  // Initiate state machine variables
  operatingState = restart;
  testState = test_select;
}

void loop() {
  // variable to switch into testing state machine
  static bool testing = false;

  // Update the LCD screen
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= UPDATE_INTERVAL) {
    currentRPM = ReadRPM();
    currentPower = CalculatePower();

    WriteToLCD();
    previousMillis = currentMillis;
  }

  // Stopped at line 215 --------------------------------

}

void SetupPins() {
  // Configure output pins
  pinMode(BRAKE_CONTROL, OUTPUT);
  pinMode(PITCH_CONTROL, OUTPUT);
  pinMode(E_BUTTON, INPUT_PULLUP);
  pinMode(VOLTAGE_SENSOR, INPUT);
  pinMode(POWER_SWITCH_RELAY, OUTPUT);
  bool setExternal = true;
  SetPowerMode(setExternal);
}

void SetPowerMode(bool setExternal) {
  if(setExternal) {
    digitalWrite(POWER_SWITCH_RELAY, HIGH);
    powerSource = "Ext";
  }
  else {
    digitalWrite(POWER_SWITCH_RELAY, LOW);
    powerSource = "Int";
  }
}

void SetupLCD() {
  // Initialize and backlight LCD screen
  lcd.init();
  lcd.backlight();
}

void SetupServos() {
  // Initialize pitch control linear actuator
  pitchActuator.attach(PITCH_CONTROL);
  SetPitch(INITIAL_PITCH);

  //Initialize brake control linear actuator
  brakeActuator.attach(BRAKE_CONTROL);
  SetBrake(BRAKE_DISENGAGED);
}

void SetupLoad() {
  //Make sure the load isn't shorted
  resistorBank.shortLoad(LOAD_UNSHORTED);
  SetLoad(MINIMUM_RESISTANCE);
}

// We stopped at ln 191 on 2023-24
