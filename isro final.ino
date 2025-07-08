#include <Wire.h>
#include <MPU6050.h>

#define MPU6050_ADDR 0x68  

MPU6050 mpu;

float ax, ay, az; 
float gx, gy, gz; 
float roll, pitch, yaw = 0.0; 

unsigned long lastTime;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }

    Serial.println("MPU6050 Initialized.");
    lastTime = millis();
}

void readMPU6050() {
    int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
    mpu.getMotion6(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);

    ax = rawAx / 16384.0; 
    ay = rawAy / 16384.0;
    az = rawAz / 16384.0;
    gx = rawGx / 131.0;  
    gy = rawGy / 131.0;
    gz = rawGz / 131.0;
}


void calculateAngles() {
    
    roll = atan2(ay, az) * 180.0 / M_PI;  
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
    
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; 
    lastTime = currentTime;

    yaw += gz * dt;  

    if (yaw < 0) yaw += 360;
    if (yaw > 360) yaw -= 360;
}

String getDirection(float yaw) {
    if (yaw >= 337.5 || yaw < 22.5) return "North";
    if (yaw >= 22.5 && yaw < 67.5) return "North-East";
    if (yaw >= 67.5 && yaw < 112.5) return "East";
    if (yaw >= 112.5 && yaw < 157.5) return "South-East";
    if (yaw >= 157.5 && yaw < 202.5) return "South";
    if (yaw >= 202.5 && yaw < 247.5) return "South-West";
    if (yaw >= 247.5 && yaw < 292.5) return "West";
    if (yaw >= 292.5 && yaw < 337.5) return "North-West";
    return "Unknown";
}

void loop() {
    readMPU6050();
    calculateAngles();
    
    String direction = getDirection(yaw);

    Serial.print("Accel X: "); Serial.print(ax);
    Serial.print(" | Accel Y: "); Serial.print(ay);
    Serial.print(" | Accel Z: "); Serial.print(az);

    Serial.print(" | Gyro X: "); Serial.print(gx);
    Serial.print(" | Gyro Y: "); Serial.print(gy);
    Serial.print(" | Gyro Z: "); Serial.print(gz);

    Serial.print(" | Roll: "); Serial.print(roll);
    Serial.print(" | Pitch: "); Serial.print(pitch);
    Serial.print(" | Yaw: "); Serial.print(yaw);
    Serial.print(" | Direction: "); Serial.println(direction);

    delay(100);
}
