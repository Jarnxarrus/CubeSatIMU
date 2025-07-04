#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>
#include <SdFat.h>

// === Constants ===
#define SD_CS 10 // CS pin of the Catalex SD module 
#define LED_PIN 5 // Pin D5 for the LED

// === IMU and SD file objects ===
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
SdFat sd;            // Objet SdFat
SdFile logFile;      // Fichier pour écrire

void setup() {
  Serial.begin(115200); // Serial Monitor for calibration feedback

  //IMU initialization
  if (!bno.begin()) {
    Serial.println(F("Error : Can't find BNO055 !"));
    while (1);
  }

  bno.setExtCrystalUse(true); // Use external crystal for better accuracy

  //SD card initialization
  if (!sd.begin(SD_CS, SPI_HALF_SPEED)) { 
    Serial.println(F("Error : SD card initialization failed ! Is there an SD card in the module ?"));
    while (1);
  }
  Serial.println(F("SD card initialized"));

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  uint8_t sys, g, a, m;
  bno.getCalibration(&sys, &g, &a, &m);

  // Display calibration status in Serial Monitor
  Serial.print(F("System: ")); Serial.print(sys);
  Serial.print(F(" Gyro: ")); Serial.print(g);
  Serial.print(F(" Accel: ")); Serial.print(a);
  Serial.print(F(" Magneto: ")); Serial.println(m);

  // Save to SD card only if all calibration levels are at 3
  if (sys == 3 && g == 3 && a == 3 && m == 3) {

    // Read the calibration offset values
    adafruit_bno055_offsets_t offsets;
    bno.getSensorOffsets(offsets);

    // Print offsets to Serial Monitor for operator
    Serial.println(F("Calibration OK, sauvegarde..."));
    Serial.print(F("Accel Offset: ")); Serial.print(offsets.accel_offset_x); Serial.print(", ");
    Serial.print(offsets.accel_offset_y); Serial.print(", ");
    Serial.println(offsets.accel_offset_z);

    Serial.print(F("Gyro Offset: ")); Serial.print(offsets.gyro_offset_x); Serial.print(", ");
    Serial.print(offsets.gyro_offset_y); Serial.print(", ");
    Serial.println(offsets.gyro_offset_z);

    Serial.print(F("Mag Offset: ")); Serial.print(offsets.mag_offset_x); Serial.print(", ");
    Serial.print(offsets.mag_offset_y); Serial.print(", ");
    Serial.println(offsets.mag_offset_z);

    Serial.print(F("Accel Radius: ")); Serial.println(offsets.accel_radius);
    Serial.print(F("Mag Radius: ")); Serial.println(offsets.mag_radius);
    
    digitalWrite(LED_PIN, HIGH);   // Turn on LED
    delay(5000);                   // Keep LED on for 5 seconds
    digitalWrite(LED_PIN, LOW);    // Turn off LED

    // Open file
    if (logFile.open("calibration.csv", O_CREAT | O_WRITE | O_TRUNC)) { 
      // O_TRUNC pour écraser à chaque fois
      
      char buffer[128];
      int len = snprintf(buffer, sizeof(buffer), 
        "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
        offsets.accel_offset_x, offsets.accel_offset_y, offsets.accel_offset_z,
        offsets.gyro_offset_x, offsets.gyro_offset_y, offsets.gyro_offset_z,
        offsets.mag_offset_x, offsets.mag_offset_y, offsets.mag_offset_z,
        offsets.accel_radius, offsets.mag_radius);

      logFile.write(buffer, len);
      logFile.close();

      Serial.println(F("Calibration offsets saved to SD"));
    } else {
      Serial.println(F("Error when opening file"));
    }
    while (1);// stop here after saving
  }
  delay(1000);
}