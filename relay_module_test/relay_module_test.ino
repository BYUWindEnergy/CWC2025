// Define the resistor_list array (adjust values as needed)
float resistor_list[] = {10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0};

// Function declarations
float ReadLoad();
void SetLoad(int wind_speed);

void setup() {
  Serial.begin(9600);
  
  // Initialize relay pins as outputs
  for (int i = 15; i <= 21; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH); // Set all relays to OFF initially
  }
  
  Serial.println("Relay Test Program");
  printMenu();
}

void loop() {
  if (Serial.available() > 0) {
    char choice = Serial.read();
    
    switch (choice) {
      case '1':
        testReadLoad();
        break;
      case '2':
        testSetLoad();
        break;
      default:
        Serial.println("Invalid choice. Please enter 1 or 2.");
    }
    
    printMenu();
  }
}

void printMenu() {
  Serial.println("\n1: Test ReadLoad()");
  Serial.println("2: Test SetLoad()");
  Serial.println("Enter your choice:");
}

void testReadLoad() {
  Serial.println("Testing ReadLoad() function:");
  float resistance = ReadLoad();
  if (resistance != -1) {
    Serial.print("Resistance read: ");
    Serial.println(resistance);
  }
}

void testSetLoad() {
  Serial.println("Testing SetLoad() function:");
  Serial.println("Enter wind speed (5-11):");
  
  while (!Serial.available()) {
    // Wait for input
  }
  
  if (Serial.available() > 0) {
    int wind_speed = Serial.parseInt();
    Serial.read(); // Clear the newline character
    
    Serial.print("Setting load for wind speed: ");
    Serial.println(wind_speed);
    
    SetLoad(wind_speed);
  }
}

float ReadLoad() {
  // Get the resistance from the load
  // Loop through relays to see which one is "on"
  int load_i = 5;
  char message[100];
  for (int i = 15; i <= 21; i++) { // goes through pins 15-21
    int relayState = digitalRead(i);
    if (relayState == LOW) { // Changed from HIGH to LOW
      float resistance = resistor_list[load_i - 5];
      sprintf(message, "Current load resistance is %.2f", resistance);
      Serial.println(message);
      return resistance;
    }
    load_i++;
  }

  Serial.println("Error: No relay is LOW");
  return -1; // Return an error value
}

void SetLoad(int wind_speed) {
  // Validate wind_speed
  if (wind_speed < 5 || wind_speed > 11) {
    Serial.println("Error: Invalid wind speed");
    return;
  }

  // Set all the relays off
  for (int i = 15; i <= 21; i++) { // goes through pins 15-21
    digitalWrite(i, HIGH);
  }

  // Set the wanted one 'on' - windspeed can be 5-11, so this can write for pins 15-21
  digitalWrite(wind_speed + 10, LOW);

  char message[100];
  sprintf(message, "Current load resistance is %.2f", resistor_list[wind_speed - 5]);
  Serial.println(message);
}
