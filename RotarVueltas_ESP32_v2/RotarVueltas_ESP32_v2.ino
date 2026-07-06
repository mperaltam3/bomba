/*
 * RotarVueltas_ESP32_v2.ino
 * Control del servo STS3215 en modo rueda (giro continuo) con ESP32 WROOM.
 * Funcion principal: rotarVueltas(n, t)
 *   n = numero de vueltas (decimales OK, negativo = sentido contrario)
 *   t = tiempo en segundos (decimales OK)
 *
 * Hardware:
 *   - ESP32 WROOM
 *   - Serial2 hardware: RX=16, TX=17
 *
 * Pines de interrupcion:
 *   - GPIO 4  : Parada de emergencia       (FALLING)
 *   - GPIO 5  : Fin de carrera 1           (FALLING)
 *   - GPIO 13 : Fin de carrera 2           (FALLING)
 *   - GPIO 14 : Sensor IR HX-M121 burbuja  (FALLING)
 *
 * Sensor IR HX-M121 (deteccion de burbujas en la manguera):
 *   El emisor y el receptor IR se colocan a los lados de la manguera.
 *   Con liquido lleno, el haz se refracta y NO llega directo al receptor.
 *   Cuando pasa una burbuja de aire, el haz SI llega al receptor (fototransistor
 *   conduce). El pin esta en INPUT_PULLUP, por lo que en reposo se lee HIGH y,
 *   al llegar la luz al receptor, el pin cae a LOW (flanco de bajada = FALLING).
 *   Si tu modulo esta cableado al reves (activo en alto), cambia FALLING por
 *   RISING en el attachInterrupt correspondiente.
 *
 * Fines de carrera - anti-rebote por desactivacion temporal:
 *   Al activarse un fin de carrera, ademas de detener el motor, se desactiva
 *   (detachInterrupt) la interrupcion de ESE pin especifico. Esto evita que,
 *   al mover el motor en sentido contrario para alejarse del interruptor, el
 *   rebote mecanico del propio interruptor vuelva a disparar la interrupcion.
 *   La interrupcion se reactiva sola cuando se cumplen DOS condiciones:
 *     1) Ya paso al menos REARME_FIN_CARRERA_MS desde que se desactivo.
 *     2) El pin ya no esta presionado (se lee HIGH, gracias al pull-up).
 *   Esa verificacion se hace en revisarRearmeFinesDeCarrera(), llamada tanto
 *   en loop() como dentro del while de rotarVueltas().
 *
 * Monitoreo periodico de corriente del motor:
 *   Cada INTERVALO_MONITOREO_CORRIENTE_MS milisegundos se lee la corriente
 *   del servo (sms_sts.ReadCurrent) y se imprime por Serial. La conversion a
 *   mA usa 6.5 mA/unidad (resolucion tipica de los servos STS); verificar
 *   contra el datasheet si se requiere precision exacta.
 *
 * Entrada por monitor serial: vueltas tiempo
 *   Ejemplo: 2.0 10.0  -> 2 vueltas en 10 segundos
 *
 * Unidades de velocidad de la libreria:
 *   1 unidad = 0.0146 RPM  (ajustado empiricamente para STS3215)
 */

#include "D:/DOCUMENTOS MICHAEL PERALTA/BOMBA/motor2/FTServo_Arduino/examples/SMS_STS/RotarVueltas_ESP32_v2/SCServo.h"

// --- Pines Serial2 para el servo ---
#define SERVO_RX_PIN  16
#define SERVO_TX_PIN  17

// --- Pines de interrupciones ---
#define PIN_EMERGENCIA    4
#define PIN_FIN_CARRERA1  5
#define PIN_FIN_CARRERA2  13
#define PIN_IR_BURBUJA    14

SMS_STS sms_sts;

const uint8_t  SERVO_ID       = 4;
const float    RPM_POR_UNIDAD = 0.0146f;
const int16_t  SPEED_MAX      = 4095;

// --- Antirebote: tiempo minimo entre dos disparos del mismo pin ---
#define DEBOUNCE_MS 200

// --- Antirebote de fines de carrera: tiempo minimo desactivada antes de
//     poder reactivarse (ademas de requerir que el pin ya no este presionado) ---
#define REARME_FIN_CARRERA_MS 1000

// --- Monitoreo periodico de corriente del motor ---
#define INTERVALO_MONITOREO_CORRIENTE_MS 1000

// --- Flags de interrupcion (accedidos desde ISR y codigo principal) ---
volatile bool          flagEmergencia        = false;
volatile bool          flagFinCarrera1       = false;
volatile bool          flagFinCarrera2       = false;
volatile bool          flagIRBurbuja         = false;
volatile bool          hayInterrupcion       = false;

// Marca de tiempo del ultimo disparo valido de cada ISR
volatile unsigned long tUltimaEmergencia     = 0;
volatile unsigned long tUltimaFinCarrera1    = 0;
volatile unsigned long tUltimaFinCarrera2    = 0;
volatile unsigned long tUltimaIRBurbuja      = 0;

// Estado de activacion de la interrupcion de cada fin de carrera y marca de
// tiempo de su desactivacion (para el rearme automatico).
bool          interrupcionFC1Activa = true;
bool          interrupcionFC2Activa = true;
unsigned long tDesactivacionFC1     = 0;
unsigned long tDesactivacionFC2     = 0;

// Marca de tiempo del ultimo monitoreo de corriente
unsigned long tUltimoMonitoreoCorriente = 0;

// -----------------------------------------------------------------------
// ISR - deben estar en RAM para ESP32
// Cada una ignora rebotes dentro de la ventana DEBOUNCE_MS.
// -----------------------------------------------------------------------
void IRAM_ATTR isrEmergencia() {
  unsigned long ahora = millis();
  if (ahora - tUltimaEmergencia < DEBOUNCE_MS) return;
  tUltimaEmergencia = ahora;
  flagEmergencia    = true;
  hayInterrupcion   = true;
}

void IRAM_ATTR isrFinCarrera1() {
  unsigned long ahora = millis();
  if (ahora - tUltimaFinCarrera1 < DEBOUNCE_MS) return;
  tUltimaFinCarrera1 = ahora;
  flagFinCarrera1    = true;
  hayInterrupcion    = true;
}

void IRAM_ATTR isrFinCarrera2() {
  unsigned long ahora = millis();
  if (ahora - tUltimaFinCarrera2 < DEBOUNCE_MS) return;
  tUltimaFinCarrera2 = ahora;
  flagFinCarrera2    = true;
  hayInterrupcion    = true;
}

// Se dispara cuando la luz IR del HX-M121 llega al receptor (pin pasa de
// HIGH a LOW), es decir, cuando una burbuja de aire cruza la manguera.
void IRAM_ATTR isrIRBurbuja() {
  unsigned long ahora = millis();
  if (ahora - tUltimaIRBurbuja < DEBOUNCE_MS) return;
  tUltimaIRBurbuja = ahora;
  flagIRBurbuja    = true;
  hayInterrupcion  = true;
}

// -----------------------------------------------------------------------
// revisarRearmeFinesDeCarrera: reactiva la interrupcion de un fin de carrera
// una vez que ya paso el tiempo de espera y el interruptor ya no esta
// presionado. Se debe llamar periodicamente desde loop() y desde el while
// de rotarVueltas(), para que el rearme ocurra aunque el motor siga girando.
// -----------------------------------------------------------------------
void revisarRearmeFinesDeCarrera() {
  unsigned long ahora = millis();

  if (!interrupcionFC1Activa &&
      (ahora - tDesactivacionFC1 >= REARME_FIN_CARRERA_MS) &&
      digitalRead(PIN_FIN_CARRERA1) == HIGH) {
    attachInterrupt(digitalPinToInterrupt(PIN_FIN_CARRERA1), isrFinCarrera1, FALLING);
    interrupcionFC1Activa = true;
    Serial.println("[INFO] Interrupcion de Fin de carrera 1 reactivada.");
  }

  if (!interrupcionFC2Activa &&
      (ahora - tDesactivacionFC2 >= REARME_FIN_CARRERA_MS) &&
      digitalRead(PIN_FIN_CARRERA2) == HIGH) {
    attachInterrupt(digitalPinToInterrupt(PIN_FIN_CARRERA2), isrFinCarrera2, FALLING);
    interrupcionFC2Activa = true;
    Serial.println("[INFO] Interrupcion de Fin de carrera 2 reactivada.");
  }
}

// -----------------------------------------------------------------------
// monitorearCorriente: imprime por Serial la corriente del motor cada
// INTERVALO_MONITOREO_CORRIENTE_MS ms.
// -----------------------------------------------------------------------
void monitorearCorriente() {
  unsigned long ahora = millis();
  if (ahora - tUltimoMonitoreoCorriente < INTERVALO_MONITOREO_CORRIENTE_MS) return;
  tUltimoMonitoreoCorriente = ahora;

  int corriente = sms_sts.ReadCurrent(SERVO_ID);
  if (corriente == -1) {
    Serial.println("[MONITOREO] Error leyendo corriente del motor.");
  } else {
    Serial.print("[MONITOREO] Corriente del motor: ");
    Serial.print(corriente * 6.5f);
    Serial.println(" mA");
  }
}

// -----------------------------------------------------------------------
// procesarInterrupciones: detiene el motor 5 s si hay algun flag activo.
// Retorna true si se detuvo el motor.
// -----------------------------------------------------------------------
bool procesarInterrupciones() {
  if (!hayInterrupcion) return false;

  sms_sts.WriteSpe(SERVO_ID, 0, 0);   // detener motor de inmediato

  if (flagEmergencia) {
    Serial.println("[INTERRUPCION] Parada de emergencia detectada! Motor detenido 5 segundos.");
    flagEmergencia = false;
  }
  if (flagFinCarrera1) {
    Serial.println("[INTERRUPCION] Fin de carrera 1 detectado! Motor detenido 5 segundos.");
    flagFinCarrera1 = false;
    if (interrupcionFC1Activa) {
      detachInterrupt(digitalPinToInterrupt(PIN_FIN_CARRERA1));
      interrupcionFC1Activa = false;
      tDesactivacionFC1     = millis();
      Serial.println("[INFO] Interrupcion de Fin de carrera 1 desactivada temporalmente.");
    }
  }
  if (flagFinCarrera2) {
    Serial.println("[INTERRUPCION] Fin de carrera 2 detectado! Motor detenido 5 segundos.");
    flagFinCarrera2 = false;
    if (interrupcionFC2Activa) {
      detachInterrupt(digitalPinToInterrupt(PIN_FIN_CARRERA2));
      interrupcionFC2Activa = false;
      tDesactivacionFC2     = millis();
      Serial.println("[INFO] Interrupcion de Fin de carrera 2 desactivada temporalmente.");
    }
  }
  if (flagIRBurbuja) {
    Serial.println("[INTERRUPCION] Burbuja detectada por sensor IR HX-M121! Motor detenido 5 segundos.");
    flagIRBurbuja = false;
  }

  hayInterrupcion = false;
  delay(5000);
  return true;
}

// -----------------------------------------------------------------------
// rotarVueltas: gira n vueltas en t segundos.
// Se cancela si se detecta una interrupcion durante la rotacion.
// -----------------------------------------------------------------------
void rotarVueltas(float n, float t) {
  if (t <= 0.0f || n == 0.0f) return;

  float   rpm      = (fabsf(n) / t) * 60.0f;
  int16_t speedVal = (int16_t)(rpm / RPM_POR_UNIDAD + 0.5f);

  if (speedVal < 1)         speedVal = 1;
  if (speedVal > SPEED_MAX) speedVal = SPEED_MAX;
  if (n < 0)                speedVal = -speedVal;

  sms_sts.WriteSpe(SERVO_ID, speedVal, 0);

  unsigned long tStart = millis();
  unsigned long tTotal = (unsigned long)(t * 1000.0f);

  while (millis() - tStart < tTotal) {
    if (procesarInterrupciones()) {
      Serial.println("Rotacion cancelada por interrupcion.");
      return;
    }
    revisarRearmeFinesDeCarrera();
    monitorearCorriente();
    delay(10);
  }

  sms_sts.WriteSpe(SERVO_ID, 0, 0);
  delay(50);

  int posFinal = sms_sts.ReadPos(SERVO_ID);
  Serial.print("<< Detenido. Pos=");
  Serial.println(posFinal);
}

// -----------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  sms_sts.pSerial = &Serial2;
  delay(1000);

  // Configurar pines de interrupcion con pull-up interno
  pinMode(PIN_EMERGENCIA,   INPUT_PULLUP);
  pinMode(PIN_FIN_CARRERA1, INPUT_PULLUP);
  pinMode(PIN_FIN_CARRERA2, INPUT_PULLUP);
  pinMode(PIN_IR_BURBUJA, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(4),  isrEmergencia,  FALLING);
  attachInterrupt(digitalPinToInterrupt(5),  isrFinCarrera1, FALLING);
  attachInterrupt(digitalPinToInterrupt(13), isrFinCarrera2, FALLING);
  attachInterrupt(digitalPinToInterrupt(14), isrIRBurbuja,   FALLING);

  // Verificar comunicacion con el servo
  int ping = sms_sts.Ping(SERVO_ID);
  if (ping < 0) {
    Serial.println("ERROR: No responde el servo. Verifica:");
    Serial.println("  1) Baud rate (STS3215 default = 1,000,000 - debe estar en 115200)");
    Serial.println("  2) Cables en RX=16, TX=17");
    Serial.println("  3) ID del servo");
    while (true) delay(1000);
  }
  Serial.print("Servo detectado. ID=");
  Serial.println(ping);

  // Desbloquear EPROM y poner en modo rueda (giro continuo)
  sms_sts.unLockEprom(SERVO_ID);
  delay(100);
  sms_sts.WheelMode(SERVO_ID);
  delay(100);
  sms_sts.LockEprom(SERVO_ID);
  delay(100);

  // Habilitar torque
  sms_sts.EnableTorque(SERVO_ID, 1);
  delay(100);

  Serial.println("Servo listo. Interrupciones activas.");
  Serial.println("Ingresa: vueltas tiempo   (ej: 2.0 10.0)");
}

void loop() {
  procesarInterrupciones();
  revisarRearmeFinesDeCarrera();
  monitorearCorriente();

  if (Serial.available()) {
    float vueltas = Serial.parseFloat();
    float tiempo  = Serial.parseFloat();

    // Limpiar cualquier basura restante en el buffer
    while (Serial.available()) Serial.read();

    if (vueltas != 0.0f && tiempo > 0.0f) {
      Serial.print("Rotando ");
      Serial.print(vueltas);
      Serial.print(" vuelta(s) en ");
      Serial.print(tiempo);
      Serial.println(" segundo(s)...");
      rotarVueltas(vueltas, tiempo);
      Serial.println("Listo. Ingresa: vueltas tiempo");
    } else {
      Serial.println("Formato invalido. Usa: vueltas tiempo   (ej: 2.0 10.0)");
    }
  }
}
