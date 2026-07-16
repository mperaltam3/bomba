/*
 * TestSensorIR.ino
 * Prueba unicamente del sensor IR HX-M121 (deteccion de burbujas), sin
 * servo ni otras interrupciones. Utilidad: verificar cableado y umbral
 * del sensor detectando por interrupcion (flanco de subida) cuando pasa
 * una burbuja frente al receptor.
 *
 * Hardware:
 *   - PIN_IR_TX      (26): emisor IR. Debe estar en HIGH para que el
 *                          emisor este encendido y el receptor pueda
 *                          detectar burbujas.
 *   - PIN_IR_BURBUJA (14): receptor IR.
 *                          Al pasar una burbuja de aire, la luz del
 *                          emisor llega directo al receptor y el pin
 *                          sube a HIGH (flanco de subida = RISING).
 *
 * Antirebote: como fisicamente no pueden pasar dos burbujas en menos de
 * unos segundos, se ignora cualquier interrupcion que llegue antes de
 * DEBOUNCE_MS desde la ultima deteccion valida (ver DEBOUNCE_MS abajo).
 */

#define PIN_IR_TX       26
#define PIN_IR_BURBUJA  14

// Antirebote: solo se acepta una deteccion cada 7 s (rango util: 5-10 s)
#define DEBOUNCE_MS 7000

volatile bool          flagIRBurbuja    = false;
volatile unsigned long tUltimaIRBurbuja = 0;

void IRAM_ATTR isrIRBurbuja() {
  unsigned long ahora = millis();
  if (ahora - tUltimaIRBurbuja < DEBOUNCE_MS) return;
  tUltimaIRBurbuja = ahora;
  flagIRBurbuja    = true;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_IR_TX, OUTPUT);
  digitalWrite(PIN_IR_TX, HIGH);   // encender emisor IR

  pinMode(PIN_IR_BURBUJA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_IR_BURBUJA), isrIRBurbuja, RISING);

  Serial.println("Prueba de sensor IR HX-M121 (burbuja).");
  Serial.println("Emisor IR encendido. Esperando interrupciones (flanco de subida)...");
}

void loop() {
  if (flagIRBurbuja) {
    flagIRBurbuja = false;
    Serial.println("BURBUJA DETECTADA (interrupcion RISING en PIN_IR_BURBUJA)");
  }
}
