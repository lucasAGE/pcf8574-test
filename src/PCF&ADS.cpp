#include <Wire.h>
#include <pcf8574_esp.h>
#include <Arduino.h>
#include <ADS1X15.h>           // Rob Tillaart
#include <math.h>

#define PCF_ADDR     0x20      // endereço do PCF8574
#define ADS_ADDR     0x48      // endereço do ADS1115
#define LED_PIN      0         // P0 do PCF8574
#define T_THRESHOLD  30.0f     // °C para acionar o LED

// -------------------------------------------------
// Constantes do divisor de tensão (PT100K em cima)
// -------------------------------------------------
const float VCC     = 5.0f;        // tensão de alimentação
const float R_REF   = 10000.0f;    // 10 kΩ resistor de referência

// -------------------------------------------------
// Termistor PTC100K (ou NTC100K) – equação Beta
// -------------------------------------------------
const float R0      = 100000.0f;   // 100kΩ @ 25 °C
const float BETA    = 3950.0f;     // depende do seu termistor (ex: 3950 K)
const float T0      = 298.15f;     // 25 °C em Kelvin (25 + 273.15)

// -------------------------------------------------
// ADS1115 LSB com ganho ±6.144 V (gain = 1×)
// -------------------------------------------------
const float ADS_LSB = 0.1875e-3f;   // 0.1875 mV/count

PCF857x expander(PCF_ADDR, &Wire);
ADS1115 ADS(ADS_ADDR);

/**
 * rawToCelsius:
 *   · raw → tensão Vnode
 *   · tensão → resistência do termistor (divisor)
 *   · resistência → T(K) via Beta → T(°C)
 */
float rawToCelsius(int16_t raw) {
  // 1) raw → tensão no nó
  float Vnode = raw * ADS_LSB;

  // 2) divisor: PT100K em cima, Rref em baixo
  //    Rpt = Rref * (VCC - Vnode) / Vnode
  float Rpt = R_REF * (VCC - Vnode) / Vnode;

  // 3) equação Beta: 1/T = 1/T0 + (1/BETA)·ln(Rpt/R0)
  float invT = 1.0f/T0 + (1.0f/BETA) * log(Rpt / R0);
  float TK   = 1.0f / invT;        // temperatura em Kelvin
  float TC   = TK - 273.15f;       // converte para °C

  return TC;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // PCF8574
  expander.begin();
  expander.write8(0xFF);   // todos HIGH = LED apagado

  // ADS1115
  ADS.begin();
  ADS.setGain(0);           // ±6.144 V full-scale
  ADS.setDataRate(7);       // 860 SPS (rápido)
  ADS.setMode(0);           // modo contínuo
  ADS.readADC(0);           // dispara primeira conversão em AIN0
}

void loop() {
  // 1) lê raw do ADS
  int16_t raw = ADS.getValue();

  // 2) converte pra °C via Beta
  float temperature = rawToCelsius(raw);

  // 3) debug
  Serial.print(F("Raw=")); Serial.print(raw);
  Serial.print(F("  T="));  Serial.print(temperature, 1);
  Serial.println(F(" °C"));

  // 4) acende/apaga LED no PCF8574
  if (temperature > T_THRESHOLD) {
    expander.write(LED_PIN, LOW);   // sinking → LED aceso
  } else {
    expander.write(LED_PIN, HIGH);  // libera → LED apagado
  }

  delay(500);
}
