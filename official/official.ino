// Libraries
#include <Wire.h> //Default library (uses pinMode, etc.)
#include <string.h> //Default library (for printing strings)
#include <LiquidCrystal_I2C.h> //LiquidCrystal I2C by Frank de Brabander
#include <Servo.h> //PWMServo by Jim Studt et al.

//  Global constants for pins
//  Analog pins
#define VOLTAGE_SENSOR A1

// Encoder channel index
#define ENCODER_PIN_INDEX 2

//  PWM pins
#define BRAKE_CONTROL 4
#define PITCH_CONTROL 5

//  Power pins
#define POWER_SWITCH_RELAY 28
#define POWER_METER_RELAY 40
#define E_BUTTON 44

// Voltage variables
#define DISCONNECT_VOLTAGE 0.5

// Variable for timing / LCD Update (ms)
unsigned long previousMillis = 0;
#define UPDATE_INTERVAL 1000

// Linear actuator constants
// For the PQ12-R, 1000 is fully extended and 2000 is fully retracted.
#define INITIAL_PITCH 1000
#define MINIMUM_PITCH 1550 //1800
#define MAXIMUM_PITCH 1000 //1300, 1225
#define SURVIVAL_PITCH 1550 //CHANGE THIS VALUE DURING CALIBRATION
#define BRAKE_DISENGAGED 1450
#define BRAKE_ENGAGED 1300

// Voltage divider constants
#define DISCONNECT_VOLTAGE 0.5
#define SMALL_READER_RESISTOR 39000.0 // check these values
#define LARGE_READER_RESISTOR 390000.0 // check these values

// Resistor and rpm lists
// wind speed = {0,1,2,3,4,5,6,7,8,9,10,11};
float resistor_list[] = {5.0,5.0,5.0,5.0,5.0,50.0,60.0,70.0,80.0,90.0,100.0,110.0};
float rpm_list[] = {5.0,5.0,5.0,5.0,5.0,1560.0,1900.0,2170.0,2540.0,2910.0,3248.0,3585.0};

// ToDo: might be able to erase bc of the rpm list
// Transition RPM values
#define CUT_IN_RPM 300 //By my calculations, our cutin rpm should be closer to 350. -Nathan
#define SURVIVAL_RPM 3800 // CHANGE THIS DURING CALIBRATION
#define SURVIVAL_EXIT_RPM 400 // TESTING THIS VARIABLE

//Arduino Mega global constants
#define OPERATING_VOLTAGE 5.0
#define ANALOG_RANGE 1023.0

//Servos (linear actuators)
Servo pitchActuator;
Servo brakeActuator;

// LCD variables
LiquidCrystal_I2C lcd(0x3f, 20, 4);
String powerSource = "Ext";
float currentRPM = 0.0;
float currentPower = 0.0;
int currentPitch = MINIMUM_PITCH;
int previousPitch = MINIMUM_PITCH;
String brakeState = "Dis";
float currentWindSpeed = 0.0;

//Enum for the substates within the overall state machine
typedef enum {
  restart,
  power_curve,
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
  read_voltage,
  read_power,
  read_rpm,
  set_load,
  read_load,
  emergency_button,
  load_disconnect,
  survival_test,
  power_curve_test,
  exit_test
} testStateMachine;

//Variable to keep track of current test state
testStateMachine testState = test_select;

void setup() {
  Serial.begin(9600);  // Set baud rate to 9600 bit/s

  //  Setup functions
  SetUpPins();
  Serial.println("Pins are set up.");
  SetUpLCD();
  Serial.println("LCD is set up.");
  lcd.setCursor(0, 0);  // Set cursor to top left corner
  lcd.print("Starting up...");
  SetUpServos();
  Serial.println("Servos are set up.");
  SetUpLoad();
  Serial.println("Load is set up.");

  pinMode(ENCODER_PIN_INDEX, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_INDEX), indexISR, RISING);

  delay(3000); // Wait 3 seconds for linear actuators to reach correct positions

  // Initiate state machine variables
  operatingState = restart;
  testState = test_select;
}

void loop() {
  static bool testing = false; // variable to switch into testing state machine

  // Update the LCD screen
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= UPDATE_INTERVAL) {
    currentRPM = ReadRPM();
    currentPower = CalculatePower();

    WriteToLCD();
    previousMillis = currentMillis;
  }

  // If there is an input, display it
  if(Serial.available()){
    String input = Serial.readStringUntil('\n');
    Serial.println(input);

    // If the input was "test", enter the test state
    if(input.equals("test")) {
      Serial.println("Entering test state...");
      operatingState = test;
    }
  }

  switch(operatingState) {
    case restart:
    {
      // If the e-stop button is pressed, emergency stop. No load disconnect possible
      if(IsButtonPressed()) {
        previousState = operatingState;
        operatingState = emergency_stop;
        previousPitch = currentPitch;
      }

      // Once the blades start spinning fast enough, move to power curve
      else if(ReadRPM() > CUT_IN_RPM) {
        operatingState = power_curve;
      }

      break;
    }

    case power_curve:
    {
      // If the load is disconnected or the e-stop button is pressed, emergency stop
      if(!IsLoadConnected() || IsButtonPressed()) {
        previousState = operatingState;
        operatingState = emergency_stop;
        previousPitch = currentPitch;
      }

      // If the wind speed is greater than 11 m/s, switch to survival
      else if(ReadRPM() > SURVIVAL_RPM) {
        operatingState = survival;
      }

      break;
    }

    case survival:
    {
      // If the load is disconnected or the e-stop button is pressed, emergency stop
      if(!IsLoadConnected() || IsButtonPressed()) {
        previousState = operatingState;
        operatingState = emergency_stop;
        previousPitch = currentPitch;
      }

      // TODO: TEST WHETHER WE WANT A DIFF WAY TO EXIT W/ LOWER RPM
      if(ReadRPM() < SURVIVAL_EXIT_RPM) {
        operatingState = power_curve;
      }
      
      break;
    }

    case emergency_stop:
    {
      //NOTE: While the load is disconnected, this code will continually
      //run, but because the nacelle has no power, nothing will happen.
      //This is an intentional design decision. Once the load is 
      //reconnected, the actions will execute and actually change the state
      //If the e-stop button is not pressed, restart
      if(!IsButtonPressed()) {
        Serial.println("Restarting...");

        //Power externally and reset the system to the desired state
        bool setExternal = true;
        SetPowerMode(setExternal);
        SetPitch(previousPitch);
        Serial.println("Pitching back to original position...");
        delay(1000);
        SetBrake(BRAKE_DISENGAGED);
        Serial.println("Disengaging brake...");
        delay(5000); //ToDo: I think this delay needs to be increased, but we should test in the wind tunnel first. -Nathan

        //If we're getting an RPM reading, the load must be connected. Transition out
        if(ReadRPM() >= CUT_IN_RPM) {
          Serial.println("Got an RPM reading. Leaving e-stop state.");
          operatingState = previousState;
          //Set power mode back to internal and wait to stop spinning
          setExternal = false;
          SetPowerMode(setExternal);
          delay(2000);
        }      
      }
    
      break;
    }
  
  case test:
    {
      //Set the "testing" boolean to true to enter the second state machine
      testing = true;
      break;
    }
  }

  switch(operatingState) {
    case restart:
    {
      //Change power to internal
      bool setExternal = false;
      SetPowerMode(setExternal);

      //Disengage the brake, return to initial pitch if low wind speed
      SetBrake(BRAKE_DISENGAGED);
      if(ReadRPM() < CUT_IN_RPM) {
        SetPitch(INITIAL_PITCH);
      }
      
      break;
    }

    case power_curve:
    {
      // Set the load depending on the rpm reading
      // The numbers correlate to the wind speed - resistor value
      float rpm = ReadRPM();
      if (rpm < rpm_list[6]) {
        SetLoad(5);
        currentWindSpeed = 5;
      } else if (rpm >= rpm_list[6] && rpm < rpm_list[7]) {
        SetLoad(6);
        currentWindSpeed = 6;
      } else if (rpm >= rpm_list[7] && rpm < rpm_list[8]) {
        SetLoad(7);
        currentWindSpeed = 7;
      } else if (rpm >= rpm_list[8] && rpm < rpm_list[9]) {
        SetLoad(8);
        currentWindSpeed = 8;
      } else if (rpm >= rpm_list[9] && rpm < rpm_list[10]) {
        SetLoad(9);
        currentWindSpeed = 9;
      } else if (rpm >= rpm_list[10] && rpm < rpm_list[11]) {
        SetLoad(10);
        currentWindSpeed = 10;
      } else if (rpm >= rpm_list[11]) {
        SetLoad(11);
        currentWindSpeed = 11;
      } else {
        SetLoad(11); // Set to 11 to have the highest resistance in case something goes wrong
      }

      //Change power to internal
      bool setExternal = false;
      SetPowerMode(setExternal);
      
      //Adjust the load to match best value based on the wind speed
      SetPitch(MAXIMUM_PITCH);
      break;
    }

    case survival:
    {
      //Change power to internal
      bool setExternal = false;
      SetPowerMode(setExternal);
      
      //Pitch the blades out of the wind completely
      SetPitch(SURVIVAL_PITCH);
      break;
    }
      
    case emergency_stop:
    {
      //Brake and pitch out of the wind
      SetBrake(BRAKE_ENGAGED);
      SetPitch(MINIMUM_PITCH); // ToDo: MIGHT HAVE TO COMMENT OUT
      delay(3000);
      break;
    }
    
    case test:
    {
      //State actions occur in testing state machine
      break;
    }
  }

  //This is the testing state machine. It should only execute if we are actively testing
  if(testing) {
    //Write current values to the LCD
    WriteToLCD();
    //Testing state transitions
    switch(testState) {
      
      case test_select:
      {
        //Output formatting for readability
        Serial.println("----------------------------------------------------------------------------");
        //Select a test to run
        SelectTest();
        break;
      }

      case power_select:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }
        
      case brake:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }

      case pitch:
      {     
        //Return to test selection after test is run
        testState = test_select;
        break;
      }
      
      case read_voltage:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }

      case read_power:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }

      case read_rpm:
      {      
        //Return to test selection after test is run
        testState = test_select;
        break;
      }

      case set_load:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }

      case read_load:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }
     
      case emergency_button:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }

      case load_disconnect:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }
      
      case survival_test:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }
      
      case power_curve_test:
      {
        //Return to test selection after test is run
        testState = test_select;
        break;
      }
     
      case exit_test:
      {
        //Return to the operating state machine in the restart state
        Serial.println("Leaving test state");
        operatingState = restart; 
        testing = false;
        testState = test_select;
        break; 
      }
        
    }

    WriteToLCD();
    
    //Testing state actions
    switch(testState) {

      //Don't do anything in the test selection state, just return to state transitions
      case test_select:
      {
        break;
      }

      //Select the power source. Usually best to power nacelle components externally (from the wall) 
      //if the turbine is not running and internally if the turbine is running
      case power_select:
      {
        //Get input from the serial monitor based on the prompt
        Serial.print("Select the desired power source. Type e to power externally (from the wall), ");
        Serial.println("i to power internally (from the turbine).");
        while(!Serial.available()) {
        }
        String powerInput = Serial.readStringUntil('\n');
        bool setExternal;

        //If the input is "i", set the power mode to internal
        if(powerInput.equals("i")) {
          setExternal = false;
          SetPowerMode(setExternal);
        }

        //If the input is "e", set the power mode to external
        else if(powerInput.equals("e")) {
          setExternal = true;
          SetPowerMode(setExternal);
        }

        //If no entry is detected, return to test selection
        else {
          Serial.println("Invalid entry. Returning to test selection...");
        }
        break;
      }

      //Activate and deactivate the brake using the linear actuator
      case brake:
      {
        //Get input from the serial monitor based on the prompt
        Serial.println("Select brake setting. Type e for engaged, d for disengaged");
        while(!Serial.available()) {
        }
        String brakeInput = Serial.readStringUntil('\n');

        //If the input is "e", engage the brake
        if(brakeInput.equals("e")) {
          SetBrake(BRAKE_ENGAGED);
        }

        //If the input "d", disengage the brake
        else if(brakeInput.equals("d")) {
          SetBrake(BRAKE_DISENGAGED);
        }

        //If the input is anything else, return to the testing state
        else {
          Serial.println("Invalid entry. Returning to test selection...");
        }
        break;
      }

      //Change the pitch angle of the blades using the linear actuator
      case pitch:
      {
        //Run this test until the user manually exits by trapping in the while loop
        while(true) {
          //Write out info to the LCD display
          WriteToLCD();
          //Output instructions for entering values for this test
          char message[100];
          sprintf(message, "Select pitch value. Enter an integer value between %d and %d (inclusive). ", 
                  MAXIMUM_PITCH, MINIMUM_PITCH);
          Serial.print(message);
          Serial.println("Enter 0 to exit");

          //Read in the input based on the prompt
          int pitchInput = ReadInputInt();
    
          //This is counterintuitive, but MINIMUM_PITCH is a greater value than MAXIMUM_PITCH
          if(pitchInput <= MINIMUM_PITCH && pitchInput >= MAXIMUM_PITCH) {
            SetPitch(pitchInput);
          }

          //If the input is 0, exit from this test
          else if(pitchInput == 0) {
            break;
          }

          //If the input is invalid, read in a new input
          else {
            Serial.println("Invalid entry.");
          }
        }

        Serial.println("Returning to test selection...");
        break;
      }

      //Read the voltage across the resistor
      case read_voltage:
      {
        float voltage = ReadVoltage();

        //Output the voltage read
        Serial.print("Current voltage: ");
        Serial.print(voltage);
        Serial.println(" V");
        break;
      }

      //Read the current power output from the turbine
      case read_power:
      {
        //Calculate the current power output
        float power = CalculatePower();

        //Output the power calculated
        Serial.print("Current power output: ");
        Serial.print(power);
        Serial.println(" W");
        break;
      }

      //Read the current rpm from the encoder
      case read_rpm:
      {
        //Output the current RPM
        Serial.print("Current RPM: ");
        Serial.print(ReadRPM());
        Serial.println(" RPM");
        break;
      }

      case set_load:
      {
        //Get input from the serial monitor based on the prompt
        Serial.println("Enter the wind speed you want the resistance of (between 5-11m/s)");
        while(!Serial.available()) {
        }
        String input = Serial.readStringUntil('\n');
        int wind_speed = input.toInt();

        //Set the load to the input value
        if(wind_speed >= 5 && wind_speed <= 11) {
          SetLoad(wind_speed);          
          Serial.print("Set load to ");
          Serial.println(resistor_list[wind_speed]);
        }

        //If the output is invalid, return to the test state
        else {
          Serial.println("Invalid entry.");
          break;
        }

        break;
      }

      case read_load:
      {
        Serial.print("Resistance is currently ");
        Serial.println(ReadLoad());
        break;
      }

      //Test emergency stop when the emergency button is pressed
      case emergency_button:
      {
        //Check if the button is pressed
        if(IsButtonPressed()) {
          Serial.println("Emergency stop active. Performing stop procedures.");
          //Engage the brake and pitch the blades out of the wind
          SetBrake(BRAKE_ENGAGED);
          SetPitch(MINIMUM_PITCH);
        }
        else {
          Serial.println("Emergency stop not active.");
        }
        break;
      }

      //Test emergency stop when the load is disconnected
      case load_disconnect:
      {
        //Check if the button is pressed
        if(!IsLoadConnected()) {
          Serial.println("Load disconnected. Performing stop procedures.");
          //Engage the brake and pitch the blades out of the wind
          SetBrake(BRAKE_ENGAGED);
          SetPitch(MINIMUM_PITCH);
        }
        else {
          Serial.println("Load connected.");
        }
        break;
      }

      //Test the survival state
      case survival_test:
      {
         //Pitch blades out of the wind
        SetPitch(SURVIVAL_PITCH);
        
        break;
      }

      //Test the power curve state
      case power_curve_test:
      {
        break;
      }

      //No state actions if the test state is being exited
      case exit_test:
      {
        break;
      }

      //Trigger this message to the serial monitor if the state machine gets
      //changed to an invalid angle for some reason
      default:
      {
        Serial.println("Default state triggered. Something went wrong.");
      }
    }
  }
}

void SetUpPins() {
  // Configure output pins
  pinMode(BRAKE_CONTROL, OUTPUT);
  pinMode(PITCH_CONTROL, OUTPUT);
  pinMode(E_BUTTON, INPUT_PULLUP);
  //digitalWrite(E_BUTTON, HIGH); is e button continues not working, switch to setting up the pin this
  pinMode(VOLTAGE_SENSOR, INPUT);
  pinMode(POWER_SWITCH_RELAY, OUTPUT);
  pinMode(POWER_METER_RELAY, OUTPUT);
  bool setExternal = true;
  SetPowerMode(setExternal);

  // Relay module for load resistors
  // Using odd pins 25-37, each correlate to a sw
  for (int i = 25; i <= 37; i+=2) {
    pinMode(i, OUTPUT);
  }
}

void SetPowerMode(bool setExternal) {
  if(setExternal) {
    digitalWrite(POWER_SWITCH_RELAY, HIGH);
    powerSource = "Ext";
  }
  else {
    digitalWrite(POWER_SWITCH_RELAY, LOW);
    delay(3000);
    digitalWrite(POWER_METER_RELAY, HIGH);
    powerSource = "Int";
  }
}

void SetUpLCD() {
  // Initialize and backlight LCD screen
  lcd.init();
  lcd.backlight();
}

void SetUpServos() {
  // Initialize pitch control linear actuator
  pitchActuator.attach(PITCH_CONTROL);
  SetPitch(INITIAL_PITCH);

  //Initialize brake control linear actuator
  brakeActuator.attach(BRAKE_CONTROL);
  SetBrake(BRAKE_DISENGAGED);
}

void SetUpLoad() {
  SetLoad(5); // 5m/s resistor
}

//Reads whether the emergency stop button is pressed. Returns true if pressed, false if not
bool IsButtonPressed() {
  return digitalRead(E_BUTTON);
}

// Variables for the ReadRPM
volatile unsigned long lastIndexTime = 0;
volatile unsigned long currentIndexTime = 0;
volatile bool newRevolution = false;

// Used for ReadRPM
// If there is an interrupt in the index channel of encoder, it will come here and set newRevolution to true
void indexISR() {
  lastIndexTime = currentIndexTime;
  currentIndexTime = micros();
  newRevolution = true;
}

float ReadRPM() {
  static float rpm = 0;
  static float prevRPM = 0;
  static int noRPMcount = 0;

  if (newRevolution) {
    newRevolution = false;
    unsigned long timeInterval = currentIndexTime - lastIndexTime;
    if (timeInterval > 0) {
      rpm = (60.0 * 1000000.0) / timeInterval;
      noRPMcount = 0;
      //  Serial.println(rpm);
    }
  } 
  // else { // Trying to figure out the 0 rpm
  //   noRPMcount++;
  //   if (noRPMcount >= 50) { // Probably have to change this number:
  //     rpm = 0.0;
  //     noRPMcount = 0;
  //   }
  // }

  // Disregards outliers
  if (rpm > 4000) {
    rpm = prevRPM;
  }

  prevRPM = rpm;
  return(rpm);
}

//Reads whether the load is connected. Returns true if connected and false if not
bool IsLoadConnected() {
  float voltage = ReadVoltage();

  if(voltage >= DISCONNECT_VOLTAGE){
    return 1;
  }
  else if (voltage < DISCONNECT_VOLTAGE){
    return 0;
  }
}

//Sets the brake linear actuator to a given position
void SetBrake(int brakePosition) {  
  //Set the servo to the desired angle
  brakeActuator.writeMicroseconds(brakePosition);
  if(brakePosition == BRAKE_ENGAGED) {
    brakeState = "Eng";
  }
  else {
    brakeState = "Dis";
  }
}

//Sets the pitch linear actuator to a given position
void SetPitch(int pitchAngle){
  //Set the servo to the desired angle
  pitchActuator.writeMicroseconds(pitchAngle);
  currentPitch = pitchAngle;
}

//Calculates an estimate of the current power output from the turbine
float CalculatePower() {
  //Read the current from the current sensor
  float voltage = ReadVoltage();

  //Read the resistance from the resistor bank
  float resistance = ReadLoad();

  //Calculate and return the power
  float power = voltage * voltage / resistance;
  return power;
}

//Reads the voltage measured across the voltage divider
float ReadVoltage() {
  //Read in the voltage
  float voltageReading = (float) analogRead(VOLTAGE_SENSOR);
  
  //Convert to the actual voltage reading
  float dividedVoltage = (voltageReading / ANALOG_RANGE) * OPERATING_VOLTAGE;
  
  //Convert from voltage reading to actual voltage (voltage divider used)
  float voltage = dividedVoltage / (SMALL_READER_RESISTOR / (SMALL_READER_RESISTOR + LARGE_READER_RESISTOR));
  
  return voltage;
}

int ReadInputInt() {
  bool inputDetected = false;
  while(!Serial.available()){
  }
  String input = Serial.readStringUntil('\n');

  //Cast the input to type float and return it
  int inputInt = input.toInt();

  if(inputInt == 0) {
    Serial.println("WARNING: Non-integer input detected. If you put in a zero intentionally, disregard.");
  }
  
  return inputInt;
}

float ReadLoad() {
  // Get the resistance from the load
  // Loop through relays to see which one is "on"
  for (int i = 25; i <= 37; i+=2) { // goes through pins 15-21
    int relayState = digitalRead(i);
    if (relayState == LOW) {
      return resistor_list[(i - 25) / 2 + 5];;
    }
  }

  Serial.println("Error: No relay is LOW");
  return -1; // Return an error value
}

void SetLoad(int wind_speed) {
  // Set the 'on' relay to 'off'
  for (int i = 25; i <= 37; i+=2) {
    if (digitalRead(i) == LOW) {
      digitalWrite(i, HIGH);
    }
  }

  // Set the wanted one 'on' - windspeed can be 5-11,
  digitalWrite(2 * wind_speed + 15, LOW);
}

//Write data to the LCD screen
void WriteToLCD() {
  lcd.clear(); // Clear the LCD screen
  
  //Write the current state
  lcd.setCursor(0, 0);
  lcd.print("Current State: ");

  if(operatingState == restart) {
    lcd.print("restart");
  } else if(operatingState == power_curve) {
    lcd.print("power curve");
  } else if(operatingState == survival) {
    lcd.print("survival");
  } else if(operatingState == emergency_stop) {
    lcd.print("shutdown");
  } else if(operatingState == test) {
  
    if(testState == test_select) {
      lcd.print("test select");
    } else if(testState == power_select) {
      lcd.print("power select");
    } else if(testState == brake) {
      lcd.print("brake test");
    } else if(testState == pitch) {
      lcd.print("pitch test");
    } else if(testState == read_voltage) {
      lcd.print("read voltage");
    } else if(testState == read_power) {
      lcd.print("read power");
    } else if(testState == read_rpm) {
      lcd.print("read rpm");
    } else if(testState == set_load) {
      lcd.print("set load");
    } else if(testState == read_load) {
      lcd.print("read load");
    } else if(testState == emergency_button) {
      lcd.print("e button");
    } else if(testState == load_disconnect) {
      lcd.print("load discon");
    } else if(testState == survival_test) {
      lcd.print("survival test");
    } else if(testState == power_curve_test) {
      lcd.print("pwr crve test");
    } else if(testState == exit_test) {
      lcd.print("exiting test ");
    }
  }

  //Write the wind speed (WS)
  lcd.setCursor(0, 1);
  lcd.print("WS: ");
  lcd.print(currentWindSpeed);

  //Write the turbine RPM
  lcd.setCursor(10, 1);
  lcd.print("RPM: ");
  lcd.print(currentRPM);

  //Write the pitch "angle"
  lcd.setCursor(0, 2);
  lcd.print((char)224);
  lcd.print(": ");
  lcd.print(currentPitch);

  //Write the brake state
  lcd.setCursor(10, 2);
  lcd.print("Brake: ");
  lcd.print(brakeState);

  //Write the power output
  lcd.setCursor(0, 3);
  lcd.print("Pwr: ");
  lcd.print(currentPower);

  //Write the current power source
  lcd.setCursor(10, 3);
  lcd.print("Src: ");
  lcd.print(powerSource);

  // lambda chart variables
  // Serial.print("RPM: ");
  // Serial.println(currentRPM);
  // Serial.print("Volt: ");
  // Serial.println(ReadVoltage());
}

//Selects a test state based on manual user input
void SelectTest() {
  Serial.print("Enter a desired test. Valid options: pwrsrc, brake, pitch, readpwr, readvolt, readrpm, setload,\n");
  Serial.print("readload, ebutt, loaddis, survival, steadypwr, pwrcurve, exit\n");

  while(!Serial.available()){
  }
  String input = Serial.readStringUntil('\n');

  if(input.equals("pwrsrc")) {
    testState = power_select;
  }
  
  if(input.equals("brake")) {
    testState = brake;
  }
  
  if(input.equals("pitch")) {
    testState = pitch;
  }
  
  if(input.equals("ebutt")) {
    testState = emergency_button;
  }
  
  if(input.equals("loaddis")) {
    testState = load_disconnect;
  }

  if(input.equals("readpwr")) {
    testState = read_power;
  }
  
  if(input.equals("readvolt")) {
    testState = read_voltage;
  }

  if(input.equals("readrpm")) {
    testState = read_rpm;
  }

  if(input.equals("setload")) {
    testState = set_load;
  }

  if(input.equals("readload")) {
    testState = read_load;
  }
  
  if(input.equals("survival")) {
    testState = survival_test;
  }

  if(input.equals("pwrcurve")) {
    testState = power_curve_test;
  }
  
  if(input.equals("exit")) {
    testState = exit_test;
  }
}
