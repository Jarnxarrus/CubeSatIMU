#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>
#include <SdFat.h>

// === Constants ===
#define SD_CS 10                 // CS pin for the SD card module
#define LED_PIN 5                // Pin D5 for status LED
#define ACCEL_THRESHOLD 40     // Acceleration threshold (in m/s²) to trigger logging
#define LED_BLINK_INTERVAL 2000 // LED blink interval during measurement phase (ms)

// === Sensor and SD objects ===
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
SdFat sd;
SdFile logFile;

// === Launch detection variables ===
bool measuring = false;

// === LED blink timing ===
unsigned long lastBlinkTime = 0;
bool ledState = false;

// construction variables
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 1000;  // En millisecondes (1 seconde)



// === Function to load calibration from SD ===
bool loadCalibrationFromSD() {
  SdFile calibFile;
  if (!calibFile.open("calibration.csv", O_READ)) {
    Serial.println(F("Error: Could not open calibration.csv"));
    return false;
  }

  char line[128];
  int len = calibFile.fgets(line, sizeof(line));
  calibFile.close();

  if (len <= 0) {
    Serial.println(F("Error: Empty calibration file"));
    return false;
  }

  // Parse CSV line
  int values[11];
  char* token = strtok(line, ",");
  for (int i = 0; i < 11; i++) {
    if (!token) {
      Serial.println(F("Error: Invalid calibration format"));
      return false;
    }
    values[i] = atoi(token);
    token = strtok(nullptr, ",");
  }

  // Apply offsets to IMU
  adafruit_bno055_offsets_t offsets;
  offsets.accel_offset_x = values[0];
  offsets.accel_offset_y = values[1];
  offsets.accel_offset_z = values[2];
  offsets.gyro_offset_x  = values[3];
  offsets.gyro_offset_y  = values[4];
  offsets.gyro_offset_z  = values[5];
  offsets.mag_offset_x   = values[6];
  offsets.mag_offset_y   = values[7];
  offsets.mag_offset_z   = values[8];
  offsets.accel_radius   = values[9];
  offsets.mag_radius     = values[10];


  bno.setSensorOffsets(offsets);
  Serial.println(F("Calibration offsets loaded from SD."));
  return true;
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);

  // Initialize IMU
  if (!bno.begin()) {
    Serial.println(F("Error: BNO055 not detected!"));
    while (1);
  }
  bno.setExtCrystalUse(true);  // Use external crystal for better accuracy

  // Initialize SD card
  if (!sd.begin(SD_CS, SPI_HALF_SPEED)) {
    Serial.println(F("Error: SD card initialization failed!"));
    while (1);
  }
  Serial.println(F("SD card initialized"));

// Load calibration data
  if (!loadCalibrationFromSD()) {
    Serial.println(F("Aborting due to missing calibration."));
    while (1);
  }

  // Remove previous log file
  sd.remove("log.csv");

  // File creation with header line ***
  if (logFile.open("log.csv", O_CREAT | O_WRITE | O_TRUNC)) {
    logFile.println(F("time_ms,quat_w,quat_x,quat_y,quat_z,accel_x,accel_y,accel_z,lin_accel_x,lin_accel_y,lin_accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z"));
    logFile.close();
  } else {
    Serial.println(F("Error: could not create log file with header"));
  }

}

void loop() {
  // Get acceleration including gravity
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  float accelMagnitude = sqrt(accel.x() * accel.x() + accel.y() * accel.y() + accel.z() * accel.z());

  // Print acceleration magnitude for tuning/debugging
  if (millis() - lastPrintTime >= PRINT_INTERVAL) {
    Serial.print(F("accelMagnitude = "));
    Serial.println(accelMagnitude, 4);
    lastPrintTime = millis();
  }

  // Blink LED every 3 seconds if measuring
  if (measuring) {
    if (millis() - lastBlinkTime >= LED_BLINK_INTERVAL) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastBlinkTime = millis();
    }
  }

  // Launch detection: only once, set measuring to true when conditions met
  if (!measuring && accelMagnitude > ACCEL_THRESHOLD) {
  measuring = true;
  Serial.println(F("Launch detected, starting data logging."));
  }

  // If measuring is true, write one data line on each loop iteration
  // Opening and closing the file each time to keep data safe
  if (measuring) {
    if (logFile.open("log.csv", O_CREAT | O_WRITE | O_APPEND)) {

      // Write timestamp (milliseconds since start)
      logFile.print(millis());
      logFile.print(',');

      // Orientation (quaternions)
      imu::Quaternion quat = bno.getQuat();
      logFile.print(quat.w(), 4); logFile.print(',');
      logFile.print(quat.x(), 4); logFile.print(',');
      logFile.print(quat.y(), 4); logFile.print(',');
      logFile.print(quat.z(), 4); logFile.print(',');

      // Acceleration including gravity
      logFile.print(accel.x(), 4); logFile.print(',');
      logFile.print(accel.y(), 4); logFile.print(',');
      logFile.print(accel.z(), 4); logFile.print(',');

      // Linear acceleration (without gravity)
      imu::Vector<3> linearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      logFile.print(linearAccel.x(), 4); logFile.print(',');
      logFile.print(linearAccel.y(), 4); logFile.print(',');
      logFile.print(linearAccel.z(), 4); logFile.print(',');

      // Angular velocity (gyroscope)
      imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      logFile.print(gyro.x(), 4); logFile.print(',');
      logFile.print(gyro.y(), 4); logFile.print(',');
      logFile.print(gyro.z(), 4); logFile.print(',');

      // Magnetic field (magnetometer)
      imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
      logFile.print(mag.x(), 4); logFile.print(',');
      logFile.print(mag.y(), 4); logFile.println(mag.z(), 4);

      logFile.close();
    } else {
      Serial.println(F("Error: could not open log file"));
    }
  }

  delay(10);  // Sampling interval
}

