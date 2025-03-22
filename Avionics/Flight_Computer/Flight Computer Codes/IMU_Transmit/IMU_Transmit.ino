#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <LoRa.h>

#define SS   PA4   // LoRa NSS
#define RST  PA3   // LoRa Reset
#define DIO0 PB0   // LoRa DIO0

MPU6050 mpu;

void setup() {
    Serial.begin(115200);
    Wire.begin();  // Initialize I2C

    // Initialize MPU6050
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    } else {
        Serial.println("MPU6050 initialized.");
    }

    // Set up LoRa module
    LoRa.setPins(SS, RST, DIO0);
    if (!LoRa.begin(433E6)) {  
        Serial.println("LoRa startup failed!");
        while (1);
    } else {
        Serial.println("LoRa initialized.");
    }
}

void loop() {
    int16_t ax, ay, az;
    
    // Read accelerometer values
    mpu.getAcceleration(&ax, &ay, &az);
    
    // Map raw accelerometer values (-32768 to 32767) to 0-180
    int x_mapped = map(ax, -32768, 32767, 0, 180);
    int y_mapped = map(ay, -32768, 32767, 0, 180);
    int z_mapped = map(az, -32768, 32767, 0, 180);

    Serial.print("Sending: X=");
    Serial.print(x_mapped);
    Serial.print(" Y=");
    Serial.print(y_mapped);
    Serial.print(" Z=");
    Serial.println(z_mapped);

    // Send data via LoRa
    LoRa.beginPacket();
    LoRa.print(x_mapped);
    LoRa.print(",");
    LoRa.print(y_mapped);
    LoRa.print(",");
    LoRa.print(z_mapped);
    LoRa.endPacket();

    delay(100);
}
