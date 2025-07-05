#include <Wire.h>
#include "ADS1X15.h"
#include "PCF8574.h"

// Endereços I²C padrões
#define ADS_ADDRESS  ADS1115_ADDRESS    // definido em ADS1X15.h :contentReference[oaicite:0]{index=0}
#define PCF_ADDRESS  0x20               // pode ajustar se for outro

// Instâncias
ADS1115 ads;                           // construtor padrão: (address = ADS1115_ADDRESS, wire = &Wire) :contentReference[oaicite:1]{index=1}
PCF8574 pcf(PCF_ADDRESS);              // construtor padrão: (deviceAddress, wire = &Wire) :contentReference[oaicite:2]{index=2}

// Pinos P0–P3 do PCF8574 para os LEDs
const uint8_t LED_PINS[4] = { 0, 1, 2, 3 };

// Limiar em °C para acender os LEDs
const float TEMP_THRESHOLD = 30.0;

// Converte tensão em °C (0–3,3 V → 0–100 °C). Ajuste conforme seu divisor/termistor.
float voltageToCelsius(float v) {
  return (v / 3.3) * 100.0;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(10000);               // I²C em 10 kHz :contentReference[oaicite:3]{index=3}

  // ——— ADS1115 ———
  if (!ads.begin()) {
    Serial.println("Erro: ADS1115 não detectado");
    while (1) yield();
  }
  ads.setWireClock(10000);            // reforce 10 kHz no driver :contentReference[oaicite:4]{index=4}
  ads.setGain(ADS1X15_GAIN_6144MV);    // ganho 0 → ±6.144 V (menor amplificação) :contentReference[oaicite:5]{index=5}
  ads.setMode(1);                      // modo single-shot
  ads.setDataRate(4);                  // 128 SPS (bom p/ termistores)

  // ——— PCF8574 ———
  if (!pcf.begin(PCF8574_INITIAL_VALUE)) {
    Serial.println("Erro: PCF8574 não detectado");
    while (1) yield();
  }
  // Inicializa P0–P3 como saída (escreve LOW)
  for (uint8_t i = 0; i < 4; i++) {
    pcf.write(LED_PINS[i], LOW);      // write(pin, value) :contentReference[oaicite:6]{index=6}
  }

  Serial.println("Setup concluído");
}

void loop() {
  for (uint8_t ch = 0; ch < 4; ch++) {
    // readADC(pin) → raw; toVoltage(raw) → volts :contentReference[oaicite:7]{index=7}
    int16_t raw   = ads.readADC(ch);
    float   volts = ads.toVoltage(raw);
    float   temp  = voltageToCelsius(volts);

    Serial.print("Canal A"); Serial.print(ch);
    Serial.print(": "); Serial.print(temp, 2); Serial.println(" °C");

    // Liga/desliga LED conforme limiar
    pcf.write(LED_PINS[ch], temp >= TEMP_THRESHOLD ? HIGH : LOW);
  }
  Serial.println();
  delay(1000);
}
