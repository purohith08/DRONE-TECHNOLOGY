#include <Wire.h>
#include <MPU9250.h>

MPU9250 mpu;  // Create MPU9250 instance

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);  // SDA = GPIO 21, SCL = GPIO 22

    if (!mpu.setup(0x68)) {  // MPU9250 I2C Address (0x68 or 0x69)
        Serial.println("MPU9250 not detected!");
        while (1);
    }
    Serial.println("MPU9250 Initialized!");
}

void loop() {
    if (mpu.update()) {
        Serial.print("Accelerometer (g): ");
        Serial.print(mpu.getAccX()); Serial.print(", ");
        Serial.print(mpu.getAccY()); Serial.print(", ");
        Serial.println(mpu.getAccZ());

        Serial.print("Gyroscope (°/s): ");
        Serial.print(mpu.getGyroX()); Serial.print(", ");
        Serial.print(mpu.getGyroY()); Serial.print(", ");
        Serial.println(mpu.getGyroZ());

        Serial.print("Magnetometer (uT): ");
        Serial.print(mpu.getMagX()); Serial.print(", ");
        Serial.print(mpu.getMagY()); Serial.print(", ");
        Serial.println(mpu.getMagZ());

        Serial.println("--------------------");
        delay(500);
    }
}
