#include <SPI.h>
#include <LoRa.h>

#define SS   10
#define RST  9
#define DIO0 2

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("LoRa Receiver Ready");

    if (!LoRa.begin(433E6)) {  
        Serial.println("LoRa initialization failed!");
        while (1);
    }
}

void loop() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        Serial.print("Received: ");
        while (LoRa.available()) {
            Serial.print((char)LoRa.read());
        }
        Serial.println();
    }
}
