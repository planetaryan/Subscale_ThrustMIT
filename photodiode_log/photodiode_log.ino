#include <SD.h>
#include <SPI.h>

const int chipSelect = BUILTIN_SDCARD;
const int photodiodePin = A24; // Define the pin connected to the photodiode receiver

void setup() {
  Serial.begin(9600);

  // Initialize the SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("Card initialization failed!");
    while (1); // Stop execution if initialization fails
  }
  Serial.println("Card initialized.");
}

void loop() {
  int photodiodeValue = analogRead(photodiodePin); // Read the value from the photodiode receiver

  // Open the file "photodiode_log.txt" on the SD card for appending data
  File dataFile = SD.open("photodiode_log.txt", FILE_WRITE);

  // Check if the file is available and append the photodiode value
  if (dataFile) {
    dataFile.println(photodiodeValue);
    dataFile.close();
    Serial.println("Photodiode value logged: " + String(photodiodeValue));
  } else {
    Serial.println("Error opening file!");
  }

  delay(300); 
}
