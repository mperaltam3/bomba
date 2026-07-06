/*
 * CalibracionRPM.ino
 * Mide la RPM real del STS3215 para un valor de registro dado y calcula
 * el factor RPM_POR_UNIDAD que debe usarse en RotarVueltas.ino
 *
 * Como funciona:
 *   1. Gira el servo a SPEED_TEST durante DURACION_MS milisegundos
 *   2. Lee la posicion continuamente (0-4095, una vuelta = 4096 pasos)
 *   3. Acumula los pasos totales detectando el wraparound automaticamente
 *   4. Calcula RPM real y RPM_POR_UNIDAD
 *   5. Imprime el resultado listo para copiar en RotarVueltas.ino
 *
 * Hardware: Arduino Nano, SoftwareSerial pin 8=RX, 9=TX
 */

#include <SoftwareSerial.h>
#include "D:/DOCUMENTOS MICHAEL PERALTA/BOMBA/motor2/FTServo_Arduino/examples/SMS_STS/CalibracionRPM/SCServo.h"

SoftwareSerial serialServo(8, 9);
SMS_STS sms_sts;

const uint8_t  SERVO_ID    = 4;
const int16_t  SPEED_TEST  = 100;    // valor de registro a calibrar (ajusta si el servo no gira)
const uint32_t DURACION_MS = 10000;  // duracion de la prueba en ms (10 segundos)

void setup() {
  Serial.begin(115200);
  serialServo.begin(115200);
  sms_sts.pSerial = &serialServo;
  delay(1000);

  int ping = sms_sts.Ping(SERVO_ID);
  if (ping < 0) {
    Serial.println("ERROR: no responde el servo (baud rate o ID incorrecto).");
    while (true);
  }
  Serial.print("Servo OK. ID=");
  Serial.println(ping);

  sms_sts.unLockEprom(SERVO_ID);
  delay(100);
  sms_sts.WheelMode(SERVO_ID);
  delay(100);
  sms_sts.LockEprom(SERVO_ID);
  delay(100);
  sms_sts.EnableTorque(SERVO_ID, 1);
  delay(100);

  Serial.println();
  Serial.println("=== CALIBRACION RPM_POR_UNIDAD ===");
  Serial.print("SPEED_TEST  = "); Serial.println(SPEED_TEST);
  Serial.print("Duracion    = "); Serial.print(DURACION_MS / 1000); Serial.println(" s");
  Serial.println("Iniciando en 3 segundos...");
  delay(3000);

  // Lectura inicial
  int posAnterior = sms_sts.ReadPos(SERVO_ID);
  if (posAnterior < 0) {
    Serial.println("ERROR: no se pudo leer la posicion inicial.");
    while (true);
  }

  long totalPasos = 0;

  sms_sts.WriteSpe(SERVO_ID, SPEED_TEST, 0);

  uint32_t inicio = millis();
  while (millis() - inicio < DURACION_MS) {
    int posActual = sms_sts.ReadPos(SERVO_ID);
    if (posActual < 0) continue;  // ignorar lecturas fallidas

    // Diferencia entre muestras; 4096 pasos = 1 vuelta completa
    int delta = posActual - posAnterior;

    // Detectar wraparound: si el salto es mayor a media vuelta, hubo overflow
    if (delta >  2048) delta -= 4096;
    if (delta < -2048) delta += 4096;

    totalPasos += delta;
    posAnterior = posActual;
  }

  sms_sts.WriteSpe(SERVO_ID, 0, 0);

  // Calcular resultados
  float vueltas        = (float)totalPasos / 4096.0f;
  float duracionMin    = DURACION_MS / 60000.0f;
  float rpm            = vueltas / duracionMin;
  float rpmPorUnidad   = rpm / (float)SPEED_TEST;

  Serial.println();
  Serial.println("=== RESULTADO ===");
  Serial.print("Pasos totales  : "); Serial.println(totalPasos);
  Serial.print("Vueltas reales : "); Serial.println(vueltas, 3);
  Serial.print("RPM real       : "); Serial.println(rpm, 3);
  Serial.println();
  Serial.println("Copia esta linea en RotarVueltas.ino:");
  Serial.print("  const float RPM_POR_UNIDAD = ");
  Serial.print(rpmPorUnidad, 4);
  Serial.println("f;");
}

void loop() {}
