#include <SPI.h>
#include <LoRa.h>

#define SS   PA4   // LoRa NSS
#define RST  PA3   // LoRa Reset
#define DIO0 PB0   // LoRa DIO0
#define POT  PA0   // Potentiometer input

void setup() {
  Serial.begin(115200);
  pinMode(POT, INPUT);
  
  while (!Serial);
  Serial.println("LoRa Sender Initialized");

  // Set the LoRa module pins
  LoRa.setPins(SS, RST, DIO0);

  // Initialize LoRa at 433 MHz
  if (!LoRa.begin(433E6)) { 
    Serial.println("LoRa startup failed!");
    while (1);
  }
}

void loop() {
  int val = analogRead(POT);  
  val = map(val, 0, 4095, 0, 255);  // For STM32, ADC range is 0-4095

  Serial.print("Sending: ");
  Serial.println(val);

  LoRa.beginPacket();
  LoRa.print(val);
  LoRa.endPacket();
  
  delay(50);
}
