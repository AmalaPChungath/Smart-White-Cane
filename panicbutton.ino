#include <TinyGPS.h>
#include <SoftwareSerial.h>

// Create TinyGPS object
TinyGPS gps;

// Set up SoftwareSerial for GPS and GSM modules
SoftwareSerial gpsSerial(11, 12); // RX, TX for GPS
SoftwareSerial gsmSerial(9, 8);  // RX, TX for GSM

// Push button pin
#define BUTTON_PIN 10

// Recipient phone number (replace with actual number)
const char phoneNumber[] = "+918138801839"; // Replace with recipient's number

// Variables for storing GPS data
float latitude, longitude;
unsigned long fix_age;
bool buttonPressed = false;

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
  sendGSMCommand("AT+CMGF=1");      // Set GSM module to SMS text mode
  
  Serial.println("System Ready. Waiting for button press...");
}

void loop() {
  // Check if the push button is pressed
  if (digitalRead(BUTTON_PIN) == LOW) {
    buttonPressed = true;
    delay(200); // Debounce delay
    Serial.println("button pressed");
  }

  // Read data from the GPS module
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (gps.encode(c)) {
      gps.f_get_position(&latitude, &longitude, &fix_age);
    }
  }

  // If the button was pressed and we have a valid GPS fix, send the SMS
  if (buttonPressed && fix_age != TinyGPS::GPS_INVALID_AGE) {
    buttonPressed = false;

    // Create a message with the latitude and longitude
    String latitudeString = String(latitude, 6);
    String longitudeString = String(longitude, 6);
    String message = "Location: https://maps.google.com/?q=" + latitudeString + "," + longitudeString;

    // Send SMS
    sendSMS(phoneNumber, message);
    Serial.println("SMS Sent: " + message);
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
