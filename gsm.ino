#include <SoftwareSerial.h>

SoftwareSerial gsmSerial(9, 8); // RX, TX pins

void setup() {
  // Start serial communication with the GSM module and the Serial Monitor
  gsmSerial.begin(9600);  
  Serial.begin(9600);

  // Wait for GSM module to initialize
  delay(1000);

  // Send initialization commands and monitor responses
  Serial.println("Initializing GSM module...");

  gsmSerial.println("AT"); // Test communication
  delay(100);
  printResponse();

  gsmSerial.println("AT+CSQ");
  delay(100);
  printResponse();

  gsmSerial.println("AT+CMGF=1"); // Set SMS mode to text
  delay(100);
  printResponse();

  gsmSerial.println("AT+CMGS=\"+918078221075\""); // Replace with the recipient's phone number
  delay(100);
  printResponse();

  gsmSerial.println("Hello, this is a test message!"); // Your message
  delay(100);
  gsmSerial.write(26); // Send the SMS (CTRL+Z)
  
  Serial.println("Message sent.");
}

void loop() {
  // Continuously read from GSM module and print to Serial Monitor
  while (gsmSerial.available()) {
    Serial.write(gsmSerial.read());
  }
}

void printResponse() {
  while (gsmSerial.available()) {
    Serial.write(gsmSerial.read());
  }
}
