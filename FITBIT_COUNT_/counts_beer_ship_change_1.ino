#include <Wire.h>
#include <MPU6050.h>
#include <TM1637Display.h>

// MPU6050 setup
MPU6050 mpu;

// TM1637 setup
#define CLK_PIN 14 // or GPIO 14
#define DIO_PIN 12 // or GPIO 12
TM1637Display display(CLK_PIN, DIO_PIN);

float tiltThresholdX = 15.0;  // Tilt angle threshold in degrees for X axis
const float tiltThresholdY = 15.0;  // Tilt angle threshold in degrees for Y axis
int counter = 0;
unsigned long lastMotionTime = 0;
const unsigned long motionCooldown = 1000;  // 2 seconds cooldown to avoid multiple detections for a single drink
bool motionDetectedX = false;  // Flag to track if a movement has been detected in X axis
bool motionDetectedY = false;  // Flag to track if a movement has been detected in Y axis

void setup() {
  Serial.begin(115200);
  Wire.begin(4, 5);  // SDA, SCL pins for ESP8266

  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }

  display.setBrightness(0x0f);  // Set brightness level (0x00 to 0x0f)
}

void loop() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Convert raw accelerometer data to Gs
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  // Calculate the tilt angles (in degrees) relative to the vertical direction based on the X and Y axes
  float tiltAngleX = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * (180.0 / PI);
  float tiltAngleY = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * (180.0 / PI);

  unsigned long currentTime = millis();

  // Check if the tilt angle in X-axis exceeds the threshold and if enough time has passed since the last motion detection
  if ((tiltAngleX > tiltThresholdX) &&
      (currentTime - lastMotionTime) > motionCooldown &&
      !motionDetectedY) {  // Check that there was no significant Y-axis movement
    if (!motionDetectedX) {
      counter++;
      tiltThresholdX += 2.0;  // Increase the tilt threshold by 2 degrees
      Serial.print("Drinking motion detected. Counter incremented: ");
      Serial.println(counter);
      Serial.print("New tiltThresholdX: ");
      Serial.println(tiltThresholdX);
      lastMotionTime = currentTime;  // Update the last motion time
      motionDetectedX = true;  // Set the flag indicating a movement has been detected in X axis
    }
  } else if (tiltAngleX <= tiltThresholdX) {
    motionDetectedX = false;  // Reset the flag when the tilt angle goes back below the threshold
  }

  // Check if the tilt angle in Y-axis exceeds the threshold
  if (tiltAngleY > tiltThresholdY) {
    motionDetectedY = true;  // Set the flag indicating a movement has been detected in Y axis
  } else {
    motionDetectedY = false;  // Reset the flag when the tilt angle goes back below the threshold
  }

  // Display the counter value on the TM1637 display
  display.showNumberDec(counter, true);

  delay(500);  // Delay for stability and readability
}
