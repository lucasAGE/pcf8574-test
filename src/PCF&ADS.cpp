#include <Wire.h>
#include <pcf8574_esp.h>
#include <Arduino.h>
#include <ADS1X15.h>           // Rob Tillaart

#define PCF_ADDR     0x20      // endereço do PCF8574
#define ADS_ADDR     0x48      // endereço do ADS1115
#define LED_PIN      0         // P0 do PCF8574
#define T_THRESHOLD  40.0f     // °C para acionar o LED

// Constantes para PT100
const float R0      = 100.0f;        // resistência em 0 °C
const float A_coef  = 3.9083e-3f;    // coeficiente A
const float B_coef  = -5.775e-7f;    // coeficiente B
const float I_EXC   = 0.001f;        // corrente de excitação 1 mA

// LSB do ADS1115 em ±6.144 V, ganho 1×
const float ADS_LSB = 0.1875e-3f;    // 0.1875 mV por count

PCF857x expander(PCF_ADDR, &Wire);
ADS1115 ADS(ADS_ADDR);

// Converte raw ADC → Volts → Resistência PT100 → Temperatura em °C
float rawToCelsius(int16_t raw) {
  // 1) raw → tensão
  float volts = raw * ADS_LSB;

  // 2) tensão → resistência (I constante)
  float Rpt = volts / I_EXC;

  // 3) Callendar–Van Dusen (T >= 0 °C)
  float Z    = 1.0f - (Rpt / R0);
  float disc = A_coef * A_coef - 4.0f * B_coef * Z;
  if (disc < 0) return -1000.0f;      // fora do range válido
  float T    = (-A_coef + sqrt(disc)) / (2.0f * B_coef);
  return T;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // PCF8574
  expander.begin();
  expander.write8(0xFF);   // coloca todos em HIGH (LED apagado)

  // ADS1115
  ADS.begin();
  ADS.setGain(0);           // ±6.144 V full-scale
  ADS.setDataRate(7);       // 860 SPS
  ADS.setMode(0);           // modo contínuo
  ADS.readADC(0);           // dispara a primeira conversão
}

void loop() {
  // 1) lê raw do ADS
  int16_t raw = ADS.getValue();

  // 2) converte pra °C
  float temperature = rawToCelsius(raw);

  // 3) debug
  Serial.print(F("Raw="));      Serial.print(raw);
  Serial.print(F("  T="));      Serial.print(temperature, 1);
  Serial.println(F(" °C"));

  // 4) acende/apaga LED no PCF8574
  if (temperature > T_THRESHOLD) {
    expander.write(LED_PIN, LOW);   // sinking → LED aceso
  } else {
    expander.write(LED_PIN, HIGH);  // libera → LED apagado
  }

  delay(500);
}
