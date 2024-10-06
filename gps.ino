#include <TinyGPS.h>


#include <SoftwareSerial.h>

// Create a TinyGPS++ object
TinyGPS gps;

// Set up software serial on pins 4 (RX) and 3 (TX)
SoftwareSerial gpsSerial(12, 11);

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  // Initialize the GPS serial communication
  gpsSerial.begin(9600);

  Serial.println("GPS Module Test - Latitude and Longitude");
}

void loop() {
  while (gpsSerial.available()) {
    // Read the data from the GPS module
    char c = gpsSerial.read();
    //Serial.print(c);
    //Serial.println("test");

    // Attempt to encode the GPS data
    if (gps.encode(c)) {
      // If new data is available, display latitude and longitude
      displayGPSData();
      
    }
  }
}

void displayGPSData() {
  float latitude, longitude;
  unsigned long fix_age;

  // Get latitude and longitude from the GPS
  gps.f_get_position(&latitude, &longitude, &fix_age);

  // Print the latitude and longitude
  Serial.print("Latitude: ");
  Serial.println(latitude == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : latitude, 6);
  Serial.print("Longitude: ");
  Serial.println(longitude == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : longitude, 6);
}