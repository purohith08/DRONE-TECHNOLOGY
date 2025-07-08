#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <MPU6050.h>

#define RX_PIN 16  // ESP32 UART RX (connect to Pixhawk TX)
#define TX_PIN 17  // ESP32 UART TX (connect to Pixhawk RX)

const char* SSID = "iPhone";     // Replace with your WiFi SSID
const char* PASSWORD = "purohith"; // Replace with your WiFi password
const int UDP_PORT = 14550;  // MAVLink default UDP port
const char* GCS_IP = "172.20.10.15";  // Replace with Ground Control Station (GCS) IP

WiFiUDP udp;
HardwareSerial PixhawkSerial(1);  // UART1 on ESP32
MPU6050 mpu;

float ax, ay, az; 
float gx, gy, gz; 
float roll, pitch, yaw = 0.0; 
unsigned long lastTime;

void setup() {
    Serial.begin(115200);  // Debugging output
    WiFi.begin(SSID, PASSWORD);

    // Wait for WiFi connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nConnected to WiFi");
    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.localIP());

    // Initialize Pixhawk Serial communication
    PixhawkSerial.begin(57600, SERIAL_8N1, RX_PIN, TX_PIN);

    // Initialize UDP for communication with GCS
    udp.begin(UDP_PORT);

    // Initialize MPU6050
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

void sendMPU6050Data() {
    String mpuData = "AX:" + String(ax) + ", AY:" + String(ay) + ", AZ:" + String(az) +
                     ", GX:" + String(gx) + ", GY:" + String(gy) + ", GZ:" + String(gz) +
                     ", Roll:" + String(roll) + ", Pitch:" + String(pitch) + ", Yaw:" + String(yaw) +
                     ", Dir:" + getDirection(yaw);

    udp.beginPacket(GCS_IP, UDP_PORT);
    udp.print(mpuData);
    udp.endPacket();
}

void loop() {
    // Read MPU6050 and calculate angles
    readMPU6050();
    calculateAngles();

    // Send MPU6050 data to Ground Control Station
    sendMPU6050Data();

    // Read from Pixhawk and send over UDP
    while (PixhawkSerial.available()) {
        uint8_t data = PixhawkSerial.read();
        udp.beginPacket(GCS_IP, UDP_PORT);
        udp.write(data);
        udp.endPacket();
    }

    // Receive UDP data and send to Pixhawk
    int packetSize = udp.parsePacket();
    if (packetSize) {
        while (udp.available()) {
            char incoming = udp.read();
            PixhawkSerial.write(incoming);
        }
    }

    delay(100);  // Prevent excessive data transmission
}
