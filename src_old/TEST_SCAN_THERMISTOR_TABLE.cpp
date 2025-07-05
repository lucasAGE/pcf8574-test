#include <Arduino.h>
#include <avr/pgmspace.h>

//--- Definição dos tipos e da sua tabela termistor_3000 em PROGMEM ---
typedef int16_t raw16_t;
typedef int16_t raw10_t;
typedef int16_t celsius_t;

typedef struct {
  raw10_t    value;
  celsius_t celsius;
} temp_entry_t;

const temp_entry_t temptable_3000[] PROGMEM = {
  {  23, 300 }, {  25, 295 }, {  27, 290 }, {  28, 285 },
  {  31, 280 }, {  33, 275 }, {  35, 270 }, {  38, 265 },
  {  41, 260 }, {  44, 255 }, {  48, 250 }, {  52, 245 },
  {  56, 240 }, {  61, 235 }, {  66, 230 }, {  71, 225 },
  {  78, 220 }, {  84, 215 }, {  92, 210 }, { 100, 205 },
  { 109, 200 }, { 120, 195 }, { 131, 190 }, { 143, 185 },
  { 156, 180 }, { 171, 175 }, { 187, 170 }, { 205, 165 },
  { 224, 160 }, { 245, 155 }, { 268, 150 }, { 293, 145 },
  { 320, 140 }, { 348, 135 }, { 379, 130 }, { 411, 125 },
  { 445, 120 }, { 480, 115 }, { 516, 110 }, { 553, 105 },
  { 591, 100 }, { 628,  95 }, { 665,  90 }, { 702,  85 },
  { 737,  80 }, { 770,  75 }, { 801,  70 }, { 830,  65 },
  { 857,  60 }, { 881,  55 }, { 903,  50 }, { 922,  45 },
  { 939,  40 }, { 954,  35 }, { 966,  30 }, { 977,  25 },
  { 985,  20 }, { 993,  15 }, { 999,  10 }, {1004,   5 },
  {1008,   0 }, {1012,  -5 }, {1016, -10 }, {1020, -15 }
};
const uint16_t TBL_3000_LEN = sizeof(temptable_3000) / sizeof(temp_entry_t);

//--- Macro de varredura e interpolação (como no Marlin) ---
#define SCAN_THERMISTOR_TABLE(TBL,LEN) do{                                    \
  uint16_t l = 0, r = LEN, m;                                                \
  for (;;) {                                                                 \
    m = (l + r) >> 1;                                                        \
    if (!m)                                                                  \
      return celsius_t(pgm_read_word(&TBL[0].celsius));                     \
    if (m == l || m == r)                                                    \
      return celsius_t(pgm_read_word(&TBL[LEN-1].celsius));                  \
    raw10_t v00 = pgm_read_word(&TBL[m-1].value),                            \
            v10 = pgm_read_word(&TBL[m  ].value);                            \
    if (raw < v00)      r = m;                                               \
    else if (raw > v10) l = m;                                               \
    else {                                                                   \
      celsius_t t00 = celsius_t(pgm_read_word(&TBL[m-1].celsius)),           \
                t10 = celsius_t(pgm_read_word(&TBL[m  ].celsius));           \
      return t00 + (raw - v00) * float(t10 - t00) / float(v10 - v00);        \
    }                                                                        \
  }                                                                          \
} while (0)

//--- Converte raw16 → raw10 → temperatura (10 bits) ---
celsius_t analog_to_celsius_3000(raw16_t raw16) {
  uint16_t mag = raw16 < 0 ? -raw16 : raw16;
  raw10_t raw = mag >> 5;            // escala 16→10 bits
  if (raw > 1023) raw = 1023;
  SCAN_THERMISTOR_TABLE(temptable_3000, TBL_3000_LEN);
  return 0;
}

//--- Lookup inverso: dado T (°C), retorna raw10 (~0–1023) ---
raw10_t celsius_to_raw10_3000(celsius_t T) {
  const celsius_t T0 = celsius_t(pgm_read_word(&temptable_3000[0].celsius));
  const celsius_t Tn = celsius_t(pgm_read_word(&temptable_3000[TBL_3000_LEN-1].celsius));
  if (T >= T0) return pgm_read_word(&temptable_3000[0].value);
  if (T <= Tn) return pgm_read_word(&temptable_3000[TBL_3000_LEN-1].value);

  for (uint16_t i = 1; i < TBL_3000_LEN; ++i) {
    const celsius_t c_lo = celsius_t(pgm_read_word(&temptable_3000[i-1].celsius));  // ex: 300
    const celsius_t c_hi = celsius_t(pgm_read_word(&temptable_3000[i  ].celsius));  // ex: 295
    // Agora: T está *entre* c_lo e c_hi?
    if (T <= c_lo && T >= c_hi) {
      const raw10_t v_lo = pgm_read_word(&temptable_3000[i-1].value);
      const raw10_t v_hi = pgm_read_word(&temptable_3000[i  ].value);
      // Interpola raw
      return v_lo + (T - c_lo) * (v_hi - v_lo) / (c_hi - c_lo);
    }
  }
  return 0; // Não deveria chegar aqui
}


void setup() {
  Serial.begin(115200);
  while (!Serial) { }

  Serial.println(F("Raw16\tRaw10\tTemp3000"));
  // Use um inteiro de 32 bits para iterar de -32768 a +32767
  for (int32_t r16 = -32768; r16 <= 32767; r16 += 4096) {
    raw16_t raw16 = static_cast<raw16_t>(r16);
    uint16_t mag = raw16 < 0 ? -raw16 : raw16;
    raw10_t raw10 = mag >> 5;
    if (raw10 > 1023) raw10 = 1023;
    celsius_t temp = analog_to_celsius_3000(raw16);
    Serial.print(raw16); Serial.print('\t');
    Serial.print(raw10); Serial.print('\t');
    Serial.println(temp);
  }

  Serial.println(F("\nTempC\tRaw10"));
  for (int T = -50; T <= 300; T += 10) {
    raw10_t raw10 = celsius_to_raw10_3000(celsius_t(T));
    Serial.print(T); Serial.print('\t');
    Serial.println(raw10);
  }
}

void loop() {
  // nada aqui
}