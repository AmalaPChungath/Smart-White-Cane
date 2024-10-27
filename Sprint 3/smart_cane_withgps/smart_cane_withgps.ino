#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <NewPing.h>

// Create TinyGPS object
TinyGPS gps;

// Set up SoftwareSerial for GPS and GSM modules
SoftwareSerial gpsSerial(11, 12); // RX, TX for GPS
SoftwareSerial gsmSerial(8, 9);    // RX, TX for GSM

// Push button pin
#define BUTTON_PIN 10

// Recipient phone number
const char phoneNumber[] = "+918138801839";

// Variables for storing GPS data
float latitude = 0.0, longitude = 0.0;
unsigned long fix_age;

// Variables for timing GPS data reading
unsigned long lastGPSReadTime = 0;
const unsigned long gpsReadInterval = 2000; // Example: Read GPS every 2 seconds

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
const int motorPin = 7;

// Define the maximum distance we want to measure (in cm)
const int maxDistance = 200;

// Define the pin for the water sensor
int sensorPin = A0;
int sensorValue = 0;

// Create NewPing objects for each sensor
NewPing sonar1(triggerPin1, echoPin1, maxDistance);
NewPing sonar2(triggerPin2, echoPin2, maxDistance);
NewPing sonar3(triggerPin3, echoPin3, maxDistance);
NewPing sonar4(triggerPin4, echoPin4, maxDistance);

// Variables for timing
unsigned long lastSensorReadTime = 0;
const unsigned long sensorReadInterval = 5000;

void sendGSMCommand(String command);
void sendSMS(const char* phoneNumber, String message);
void readWaterSensor();
void activateAlert(int duration);
void playTone(int pin, int duration);
void readGPSData();

void setup() {
    // Initialize serial communication for debugging
    Serial.begin(9600);

    // Initialize GPS and GSM serial communications
    gpsSerial.begin(9600);
    gsmSerial.begin(9600);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(buzzerPin, OUTPUT);
    pinMode(motorPin, OUTPUT);

    // Delay to allow GSM module to initialize
    delay(1000);

    // GSM module setup
    sendGSMCommand("AT");          // Check if GSM is responding
    sendGSMCommand("AT+CMGF=1");  // Set GSM module to SMS text mode

    Serial.println("System Ready.");
}

void loop() {
    // Continuously check for button press
    checkButton();

    // If button was pressed, handle the action
    if (buttonPressed) {
        handleButtonPress();
        buttonPressed = false;  // Reset the button pressed flag
    }

    // Read sensors at the specified interval
    if (millis() - lastSensorReadTime >= sensorReadInterval) {
        lastSensorReadTime = millis();   // Update the last sensor read time
        readSensors();   // Call function to read all sensors
    }

    // Check if it's time to read GPS data
    if (millis() - lastGPSReadTime >= gpsReadInterval) {
        lastGPSReadTime = millis();  // Update the last GPS read time
        readGPSData();  // Call function to read GPS data
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
    readUltrasonicSensors();
    readWaterSensor();
}

// Function to read GPS data
void readGPSData() {
    gpsSerial.begin(9600);  // Reinitialize GPS serial
    gpsSerial.listen();      // Ensure GPS is listening

    Serial.println("Reading GPS Data...");

    unsigned long start = millis();
    while (millis() - start < 1000) {  // 1 second timeout
        while (gpsSerial.available()) {
            char c = gpsSerial.read();
            if (gps.encode(c)) {
                gps.f_get_position(&latitude, &longitude, &fix_age);
            }
        }
    }

    // Print the GPS data for debugging
    if (fix_age != TinyGPS::GPS_INVALID_AGE) {
        Serial.print("Latitude: ");
        Serial.print(latitude, 6);
        Serial.print(", Longitude: ");
        Serial.println(longitude, 6);
    } else {
        Serial.println("No valid GPS fix.");
    }

    gsmSerial.begin(9600); // Reinitialize GSM serial for next use
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

    // Alerts based on distance readings
    if (distance1 > 0 && distance1 < 30) {
        activateAlert(500);
    }
    if (distance2 > 0 && distance2 < 30) {
        activateAlert(700);
    }
    if (distance3 > 0 && distance3 < 30) {
        activateAlert(900);
    }
    if (distance4 > 0 && distance4 < 30) {
        activateAlert(1200);
    }
}

// Function to read water sensor value
void readWaterSensor() {
    sensorValue = analogRead(sensorPin);
    Serial.print("Water Sensor Value: ");
    Serial.println(sensorValue);

    if (sensorValue > 50) {
        activateAlert(250);
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
    sendGSMCommand("AT+CMGS=\"" + String(phoneNumber) + "\"");
    gsmSerial.print(message);
    gsmSerial.write(26);
    delay(5000);
}

// Function to activate the buzzer and motor simultaneously
void activateAlert(int duration) {
    digitalWrite(motorPin, HIGH);
    playTone(buzzerPin, duration);
    digitalWrite(motorPin, LOW);
}

// Function to activate the continuous buzzer with different sound patterns
void playTone(int pin, int duration) {
    int onDuration = 100;
    int offDuration = 100;
    long endTime = millis() + duration;

    while (millis() < endTime) {
        digitalWrite(pin, HIGH);
        delay(onDuration);
        digitalWrite(pin, LOW);
        delay(offDuration);
    }
}
