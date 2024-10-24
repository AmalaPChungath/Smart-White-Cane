#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <NewPing.h>

// Create TinyGPS object
TinyGPS gps;

// Set up SoftwareSerial for GPS and GSM modules
SoftwareSerial gpsSerial(11, 12); // RX, TX for GPS
SoftwareSerial gsmSerial(9, 8);    // RX, TX for GSM

// Push button pin
#define BUTTON_PIN 10

// Recipient phone number (replace with actual number)
const char phoneNumber[] = "+918138801839"; // Replace with recipient's number

// Variables for storing GPS data
float latitude, longitude;
unsigned long fix_age;
bool buttonPressed = false;

// Define the pins for the ultrasonic sensors
const int triggerPin1 = 19;
const int echoPin1 = 2;
const int triggerPin2 = 18;
const int echoPin2 = 3;
const int triggerPin3 = 17;
const int echoPin3 = 4;
const int triggerPin4 = 16;
const int echoPin4 = 5;

// Define the pin for the buzzer and vibration motor
const int buzzerPin = 6;
const int motorPin = 7;  // Pin for the vibration motor

// Define the maximum distance we want to measure (in cm)
const int maxDistance = 200;

// Define the pin for the water sensor
int sensorPin = A0;  // Analog input pin for the water sensor
int sensorValue = 0; // Variable to store sensor value

// Create NewPing objects for each sensor
NewPing sonar1(triggerPin1, echoPin1, maxDistance);
NewPing sonar2(triggerPin2, echoPin2, maxDistance);
NewPing sonar3(triggerPin3, echoPin3, maxDistance);
NewPing sonar4(triggerPin4, echoPin4, maxDistance);

// Variables for timing
unsigned long lastSensorReadTime = 0; // Last time the sensors were read
const unsigned long sensorReadInterval = 5000; // Interval to read sensors (in milliseconds)

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Initialize GPS and GSM serial communications
  gpsSerial.begin(9600);
  gsmSerial.begin(9600);

  // Initialize the push button pin
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Use internal pull-up resistor

  // Delay to allow GSM module to initialize
  delay(1000);

  // GSM module setup
  sendGSMCommand("AT");            // Check if GSM is responding
  sendGSMCommand("AT+CMGF=1");    // Set GSM module to SMS text mode

  Serial.println("System Ready.");
}

void loop() {
  // Continuously check for button press
  checkButton();

  // If button was pressed, handle the action
  if (buttonPressed) {
    handleButtonPress();
    buttonPressed = false; // Reset the button pressed flag
  }
3
  // Read sensors at the specified interval
  if (millis() - lastSensorReadTime >= sensorReadInterval) {
    lastSensorReadTime = millis(); // Update the last sensor read time
    readSensors(); // Call function to read all sensors
  }

  // Small delay for stability
  delay(100); 
}

// Function to check for button press
void checkButton() {
  int buttonState = digitalRead(BUTTON_PIN);
  
  // Check if the button is pressed
  if (buttonState == LOW) {
    buttonPressed = true;  // Set button pressed flag
    Serial.println("Button pressed");
  }
}

// Function to handle button press
void handleButtonPress() {
  // Create a message with the latitude and longitude
  String latitudeString = String(latitude, 6);
  String longitudeString = String(longitude, 6);
  String message = "Location: https://maps.google.com/?q=" + latitudeString + "," + longitudeString;

  // Send SMS if GPS fix is valid
  if (fix_age != TinyGPS::GPS_INVALID_AGE) {
    sendSMS(phoneNumber, message);
    Serial.println("SMS Sent: " + message);
  } else {
    Serial.println("GPS fix not valid, SMS not sent.");
  }
}

// Function to read all sensors
void readSensors() {
  // Read GPS data
  readGPSData();

  // Read distances from the ultrasonic sensors
  readUltrasonicSensors();

  // Read the water sensor value
  readWaterSensor();
}

// Function to read GPS data
void readGPSData() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (gps.encode(c)) {
      gps.f_get_position(&latitude, &longitude, &fix_age);
    }
  }
}

// Function to read distances from ultrasonic sensors
void readUltrasonicSensors() {
  unsigned int distance1 = sonar1.ping_cm();
  unsigned int distance2 = sonar2.ping_cm();
  unsigned int distance3 = sonar3.ping_cm();
  unsigned int distance4 = sonar4.ping_cm();

  Serial.print("Distance 1: ");
  Serial.print(distance1);
  Serial.print(" cm\t");
  Serial.print("Distance 2: ");
  Serial.print(distance2);
  Serial.print(" cm\t");
  Serial.print("Distance 3: ");
  Serial.print(distance3);
  Serial.print(" cm\t");
  Serial.print("Distance 4: ");
  Serial.println(distance4);
}

// Function to read water sensor value
void readWaterSensor() {
  sensorValue = analogRead(sensorPin);  // Read the analog sensor value
  Serial.print("Water Sensor Value: ");
  Serial.println(sensorValue);  // Print the sensor value to the serial monitor

  // Activate alert for high water level
  if (sensorValue > 50) { // Condition for high water level
    activateAlert(2500, 500); // Buzzer for high water level
  }
}

// Function to send a command to the GSM module
void sendGSMCommand(String command) {
  gsmSerial.println(command);
  delay(1000);
  while (gsmSerial.available()) {
    Serial.write(gsmSerial.read());
  }
}

// Function to send an SMS
void sendSMS(const char* phoneNumber, String message) {
  sendGSMCommand("AT+CMGS=\"" + String(phoneNumber) + "\"");  // Send the phone number
  gsmSerial.print(message);                                   // Send the message content
  gsmSerial.write(26);  // ASCII code for Ctrl+Z to send SMS
  delay(5000);
}

// Function to activate the buzzer and motor simultaneously
void activateAlert(int frequency, int duration) {
  // Activate both buzzer and motor
  digitalWrite(motorPin, HIGH);  // Turn on the motor
  playTone(buzzerPin, frequency, duration);  // Play tone on buzzer
  digitalWrite(motorPin, LOW);   // Turn off the motor after the tone
}

// Function to activate the continuous buzzer with different sound patterns
void playTone(int pin, int frequency, int duration) {
  // Calculate the duration for which the buzzer should be on and off
  int onDuration = 100;  // Buzzer on duration (in milliseconds)
  int offDuration = 100; // Buzzer off duration (in milliseconds)

  // Adjust the on and off durations based on the "frequency" parameter
  if (frequency == 1000) {
    onDuration = 500;  // Longer beep
    offDuration = 100; // Short pause
  } else if (frequency == 1500) {
    onDuration = 300;  // Medium beep
    offDuration = 150; // Medium pause
  } else if (frequency == 2000) {
    onDuration = 100;  // Rapid beep
    offDuration = 50;  // Shorter pause
  } else if (frequency == 2500) {
    onDuration = 1000;  // Long beep
    offDuration = 200;  // Long pause
  }

  // Generate the sound pattern for the specified duration
  long endTime = millis() + duration;
  while (millis() < endTime) {
    digitalWrite(pin, HIGH);    // Turn the buzzer on
    delay(onDuration);          // Keep it on for the specified on duration
    digitalWrite(pin, LOW);     // Turn the buzzer off
    delay(offDuration);         // Keep it off for the specified off duration
  }
}
