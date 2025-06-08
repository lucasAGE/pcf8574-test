#include <Wire.h>
#include <pcf8574_esp.h>
#include <Arduino.h>

#define PCF_ADDR        0x20
PCF857x expander(PCF_ADDR, &Wire);

// LEDs em P0, P1 e P2
const uint8_t LED_PINS[]  = { 0, 1, 2 };
const uint8_t NUM_LEDS    = sizeof(LED_PINS) / sizeof(LED_PINS[0]);

// Padrões de teste (full-port)
const uint8_t PATTERNS[]   = { 0x00, 0xFF, 0x55, 0xAA };
const uint8_t NUM_PATTERNS = sizeof(PATTERNS) / sizeof(PATTERNS[0]);

uint32_t totalTests = 0;
uint32_t errorCount = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  expander.begin();
  expander.write8(0xFF);  // começa com todos os bits em HIGH (leds apagados)
  Serial.println(F("=== PCF8574_ESP Blink+Diag ==="));
}

void loop() {
  // 1) Pisca os 3 LEDs sequencialmente, no modo sinking
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    uint8_t pin = LED_PINS[i];

    // acende: escreve LOW no pin, HIGH em todos os outros
    expander.write(pin, LOW);
    totalTests++;
    uint8_t state = expander.read8();
    // se o bit lido NÃO for 0, falha
    if (state & (1 << pin)) {
      errorCount++;
      Serial.print(F("LED "));
      Serial.print(i);
      Serial.print(F(" not sinking (read=0x"));
      Serial.print(state, HEX);
      Serial.println(F(")"));
    }

    delay(500);

    // apaga: libera o pino
    expander.write(pin, HIGH);
    delay(500);
  }

  // 2) Mini-diagnóstico “full-port”
  for (uint8_t i = 0; i < NUM_PATTERNS; i++) {
    uint8_t pat = PATTERNS[i];
    expander.write8(pat);
    totalTests++;
    delay(10);
    uint8_t got = expander.read8();
    if (got != pat) {
      errorCount++;
      Serial.print(F("Pattern 0x"));
      Serial.print(pat, HEX);
      Serial.print(F(" read 0x"));
      Serial.println(got, HEX);
    }
    int err = expander.lastError();
    if (err) {
      errorCount++;
      Serial.print(F("I2C error: 0x"));
      Serial.println(err, HEX);
    }
    delay(10);
  }

  // 3) Relatório
  Serial.print(F("Tests: "));
  Serial.print(totalTests);
  Serial.print(F("  Errors: "));
  Serial.print(errorCount);
  Serial.print(F("  Rate: "));
  if (totalTests) {
    float pct = errorCount * 100.0f / totalTests;
    Serial.print(pct, 2);
  } else {
    Serial.print(F("0.00"));
  }
  Serial.println(F(" %\n"));

  totalTests = errorCount = 0;
  delay(2000);
}
