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
  return (1.0 - (v / 5)) * 100.0;
}

// Varre o barramento I²C e imprime todos os dispositivos ACK
void scanI2CBus() {
  Serial.println("🔍 Escaneando barramento I2C...");
  uint8_t count = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  • Dispositivo encontrado em 0x");
      if (addr < 16) Serial.print('0');
      Serial.print(addr, HEX);
      Serial.println();
      count++;
    }
  }
  if (!count) Serial.println("  (nenhum dispositivo detectado)");
  Serial.println("🔍 Scan concluído\n");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) ;  // aguarda porta Serial
  Serial.println("\n=== Iniciando debug ADS1115 + PCF8574 ===");

  Wire.begin();
  Wire.setClock(10000);               // I²C em 10 kHz :contentReference[oaicite:3]{index=3}

  // — Varredura I2C —
  scanI2CBus();

  // — ADS1115 —
  Serial.print("📟 Iniciando ADS1115... ");
  if (!ads.begin()) {
    Serial.println("❌ NÃO detectado!");
    while (1) yield();
  }
  Serial.println("✅ detectado");
  ads.setWireClock(10000);            // reforce 10 kHz no driver :contentReference[oaicite:4]{index=4}
  ads.setGain(ADS1X15_GAIN_2048MV);    // ganho 0 → ±6.144 V (menor amplificação) :contentReference[oaicite:5]{index=5}
  ads.setMode(1);                      // modo single-shot
  ads.setDataRate(0);                  // 128 SPS (bom p/ termistores)

   // — PCF8574 —
  Serial.print("🔌 Iniciando PCF8574... ");
  if (!pcf.begin(0xFF)) {  // 0xFF = todas entradas (high)
    Serial.println("❌ NÃO detectado!");
    while (1) yield();
  }
  Serial.println("✅ detectado");
  Serial.print("  • Endereço: 0x"); Serial.println(PCF_ADDRESS, HEX);
  uint8_t portState = pcf.read8();  // lê todas as 8 portas
  Serial.print("  • Estado inicial (GPIO0–7): 0b");
  Serial.println(portState, BIN);
  
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

    // Aciona LED via PCF
    bool ledOn = (temp >= TEMP_THRESHOLD);
    pcf.write(LED_PINS[ch], ledOn ? HIGH : LOW);

    // Debug PCF8574
    bool state = pcf.read(LED_PINS[ch]);
    Serial.print("  • LED P"); Serial.print(ch);
    Serial.print(" estado físico: "); Serial.println(state ? "ON" : "OFF");
  }

  // Estado completo do PCF8574 a cada ciclo
  uint8_t fullState = pcf.read8();
  Serial.print("Porta PCF8574 inteira: 0b");
  Serial.println(fullState, BIN);

  Serial.println();
  delay(1000);
}
