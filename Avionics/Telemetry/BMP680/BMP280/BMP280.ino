#include <Wire.h>
#include <Adafruit_BMP280.h>

#define BUZZER PB1  // Buzzer connected to PB1

Adafruit_BMP280 bmp; // Create BMP280 object

void setup() {
    Serial.begin(115200);
    Wire.begin(); // Initialize I2C

    pinMode(BUZZER, OUTPUT);

    if (!bmp.begin(0x76)) { // Check if BMP280 is detected
        Serial.println("❌ BMP280 Not Found!");
        while (1);
    }

    Serial.println("✅ BMP280 Initialized!");
}

void loop() {
    float temperature = bmp.readTemperature();
    float pressure = bmp.readPressure() / 100.0F; // Convert to hPa
    float altitude = bmp.readAltitude(1013.25); // Sea-level pressure reference

    Serial.print("🌡 Temperature: "); Serial.print(temperature); Serial.println(" °C");
    Serial.print("🌍 Pressure: "); Serial.print(pressure); Serial.println(" hPa");
    Serial.print("⛰ Altitude: "); Serial.print(altitude); Serial.println(" m");

    // Beep the buzzer
    digitalWrite(BUZZER, HIGH);
    delay(100);  // Beep duration
    digitalWrite(BUZZER, LOW);

    Serial.println("----------------------------");

    delay(1000); // Wait 1 second before next reading
}
