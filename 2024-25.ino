//Define pin names
//Analog pins
#define VOLTAGE_SENSOR A0
//PWM pins
#define BRAKE_CONTROL 4
#define PITCH_CONTROL 5
//Other pins
#define POWER_SWITCH_RELAY 28
#define E_BUTTON 44

void setup() {
  Serial.begin(9600);  //Set baud rate to 9600 bit/s
}
