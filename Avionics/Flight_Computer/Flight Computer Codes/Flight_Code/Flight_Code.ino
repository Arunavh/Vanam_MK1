#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <LoRa.h>


#define LORA_SS PA4
#define LORA_RST PB0
#define LORA_DIO0 PB1

float latitude, longitude, gpsAltitude;
float satellites;

#define BUZZER PB1

SoftwareSerial GPS_Serial(PB8, PB9); 

Adafruit_BMP280 bmp;
MPU6050 mpu;
TinyGPSPlus gps;

void setup() {
    Serial.begin(115200);
    GPS_Serial.begin(9600);

    Wire.begin();
    pinMode(BUZZER, OUTPUT);

    if (!bmp.begin(0x76)) {
        Serial.println("❌ BMP280 Not Found!");
        while (1);
    }
    Serial.println("✅ BMP280 Initialized!");

    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("❌ MPU6050 Not Found!");
        while (1);
    }
    Serial.println("✅ MPU6050 Initialized!");

    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    if (!LoRa.begin(433E6)) {  
        Serial.println("❌ LoRa Init Failed!");
        while (1);
    }
    Serial.println("✅ LoRa Initialized!");
    Serial.println("⏳ Waiting for GPS fix...");
}

void gps_stuff(){
      while (GPS_Serial.available()) {
        char c = GPS_Serial.read();    
        gps.encode(c);
                Serial.print("RAW GPS DATA DUMP:");
                Serial.print(gps.location.lat(), 6);
                Serial.print(gps.location.lng(), 6);
                latitude = gps.location.lat();
                longitude = gps.location.lng();
                Serial.print(gps.altitude.meters());
                gpsAltitude = gps.altitude.meters();
                Serial.println(gps.satellites.value());
                satellites = gps.satellites.value();
        }
    }

  
void loop() {
    gps_stuff(); 

    float temperature = bmp.readTemperature();
    float pressure = bmp.readPressure() / 100.0F; // hPa
    float altitude = bmp.readAltitude(1013.25); // Sea-level reference

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float accelX = map(ax, -32768, 32767, 0, 180);
    float accelY = map(ay, -32768, 32767, 0, 180);
    float accelZ = map(az, -32768, 32767, 0, 180);

    float gyroX = gx / 131.0;
    float gyroY = gy / 131.0;
    float gyroZ = gz / 131.0;

    Serial.println("===== SENSOR DATA =====");
    Serial.print("🌡 Temp: "); Serial.print(temperature); Serial.println(" °C");
    Serial.print("🌍 Pressure: "); Serial.print(pressure); Serial.println(" hPa");
    Serial.print("⛰ BMP Altitude: "); Serial.print(altitude); Serial.println(" m");
    Serial.print("📍 GPS: "); Serial.print(latitude, 6); Serial.print(", "); Serial.println(longitude, 6);
    Serial.print("📶 Satellites: "); Serial.println(satellites);
    Serial.print("📏 GPS Altitude: "); Serial.print(gpsAltitude); Serial.println(" m");
    Serial.print("🎯 Accel X: "); Serial.print(accelX); Serial.print(" Y: "); Serial.print(accelY); Serial.print(" Z: "); Serial.println(accelZ);
    Serial.print("🔄 Gyro X: "); Serial.print(gyroX); Serial.print(" Y: "); Serial.print(gyroY); Serial.print(" Z: "); Serial.println(gyroZ);
    Serial.println("=======================");

    // 📡 Send Data via LoRa
    LoRa.beginPacket();
    LoRa.print(latitude, 6); LoRa.print(",");
    LoRa.print(longitude, 6); LoRa.print(",");
    LoRa.print(altitude); LoRa.print(",");  // BMP280 Altitude
    LoRa.print(gyroX); LoRa.print(",");
    LoRa.print(gyroY); LoRa.print(",");
    LoRa.print(gyroZ); LoRa.print(",");
    LoRa.print(accelX); LoRa.print(",");
    LoRa.print(accelY); LoRa.print(",");
    LoRa.print(accelZ); LoRa.print(",");
    LoRa.print("0");
    LoRa.print("0");
    LoRa.print("0");
    LoRa.print("0");
    LoRa.print(temperature); LoRa.print(",");
    LoRa.print(pressure);
    LoRa.print(gpsAltitude); LoRa.print(",");
    LoRa.print(satellites); LoRa.print(",");
    LoRa.print("0");
    LoRa.print("0");
    LoRa.print("0");
    LoRa.endPacket();

    // 🔔 Beep Buzzer
    digitalWrite(BUZZER, HIGH);
    delay(100);
    digitalWrite(BUZZER, LOW);

    delay(500); // Collect data every 0.5s
}
