#include <HMC5883L.h>

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <HMC5883L.h>  // Correct Library

Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
HMC5883L compass;

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);

    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("MPU6050 not detected!");
        while (1);
    }

    // Initialize BMP180
    if (!bmp.begin()) {
        Serial.println("BMP180 not detected!");
        while (1);
    }

    // Initialize HMC5883L Magnetometer
    compass = HMC5883L();
    compass.SetScale(1.3);  // ✅ Correct function
    compass.SetMeasurementMode(HMC5883L_CONTINUOUS);  // ✅ Correct function

    Serial.println("HW-290 (MPU6050 + HMC5883L + BMP180) Initialized!");
}

void loop() {
    MagnetometerRaw mag = compass.ReadRawAxis();  // ✅ Correct function

    float heading = atan2(mag.YAxis, mag.XAxis) * 180.0 / M_PI;
    if (heading < 0) heading += 360;

    Serial.print("Compass Heading: "); Serial.print(heading); Serial.println("°");
    delay(1000);
}
