#include <Wire.h>
#include <pcf8574_esp.h>
#include <Arduino.h>
#include <ADS1X15.h>

#define PCF_ADDR 0x20
PCF857x expander(PCF_ADDR, &Wire);

const uint8_t LED_PIN = 0;  // P0 do PCF8574

void setup() {
  Wire.begin();
  expander.begin();
  expander.write8(0xFF);    // todos os pinos em HIGH (LED apagado)
}

void loop() {
  // Acende o LED (puxa P0 para GND)
  expander.write(LED_PIN, LOW);
  delay(1000);               // permanece aceso por 1 s

  // Apaga o LED (libera P0)
  expander.write(LED_PIN, HIGH);
  delay(1000);               // permanece apagado por 1 s
}
