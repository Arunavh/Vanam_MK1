#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

SoftwareSerial GPS_Serial(PB8, PB9);  // RX = PB8, TX = PB9
TinyGPSPlus gps;

void setup() {
    Serial.begin(115200);       // Debugging Serial Monitor
    GPS_Serial.begin(9600);     // GPS default baud rate

    Serial.println("⏳ Waiting for GPS fix...");
}

void loop() {
    while (GPS_Serial.available()) {
        char c = GPS_Serial.read();
        Serial.write(c); // Print raw GPS data for debugging
        
        if (gps.encode(c)) {
            if (gps.location.isValid()) {
                Serial.print("📍 Lat: "); Serial.print(gps.location.lat(), 6);
                Serial.print(" | Lon: "); Serial.println(gps.location.lng(), 6);
            }
            if (gps.altitude.isValid()) {
                Serial.print("⛰ Altitude: "); Serial.print(gps.altitude.meters());
                Serial.println(" m");
            }
            if (gps.satellites.isValid()) {
                Serial.print("🛰 Satellites: "); Serial.println(gps.satellites.value());
            }
            Serial.println("-----------------------");
        }
    }
}
