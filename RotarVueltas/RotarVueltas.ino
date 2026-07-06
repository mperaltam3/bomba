/*
 * RotarVueltas.ino
 * Control del servo STS3215 en modo rueda (giro continuo) con Arduino Nano.
 * Funcion principal: rotarVueltas(n, t)
 *   n = numero de vueltas (decimales OK, negativo = sentido contrario)
 *   t = tiempo en segundos (decimales OK)
 *
 * Hardware:
 *   - Arduino Nano
 *   - SoftwareSerial: pin 8 = RX, pin 9 = TX
 *
 * IMPORTANTE - Baud rate:
 *   El STS3215 viene de fabrica a 1,000,000 baud.
 *   SoftwareSerial en Nano no puede manejar esa velocidad.
 *   Debes configurar el servo a 115200 baud una sola vez con otro Arduino/board
 *   que tenga Serial1 hardware, usando el sketch CambiarBaudRate.ino
 *   o el software de Feetech.
 *
 * Unidades de velocidad de la libreria:
 *   1 unidad = 0.732 RPM  (WriteSpe ID 60 -> 60*0.732 = 43.92 RPM)
 */

#include <SoftwareSerial.h>
#include "D:/DOCUMENTOS MICHAEL PERALTA/BOMBA/motor2/FTServo_Arduino/examples/SMS_STS/RotarVueltas/SCServo.h"

SoftwareSerial serialServo(8, 9);  // RX=8, TX=9

SMS_STS sms_sts;

const uint8_t SERVO_ID       = 4;
const float RPM_POR_UNIDAD = 0.0146f;
const int16_t SPEED_MAX      = 4095;
// -----------------------------------------------------------------------
// rotarVueltas: gira n vueltas en t segundos e imprime posicion al final
// -----------------------------------------------------------------------
void rotarVueltas(float n, float t) {
  if (t <= 0.0f || n == 0.0f) return;

  float   rpm      = (abs(n) / t) * 60.0f;
  int16_t speedVal = (int16_t)(rpm / RPM_POR_UNIDAD + 0.5f);

  if (speedVal < 1)         speedVal = 1;
  if (speedVal > SPEED_MAX) speedVal = SPEED_MAX;
  if (n < 0)                speedVal = -speedVal;

  sms_sts.WriteSpe(SERVO_ID, speedVal, 0);
  delay((unsigned long)(t * 1000.0f));   // esperar el tiempo de rotacion
  sms_sts.WriteSpe(SERVO_ID, 0, 0);
  delay(50);

  int posFinal = sms_sts.ReadPos(SERVO_ID);
  Serial.print("<< Detenido. Pos=");
  Serial.println(posFinal);
}

// -----------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  serialServo.begin(115200);
  sms_sts.pSerial = &serialServo;
  delay(1000);

  // Verificar comunicacion con el servo
  int ping = sms_sts.Ping(SERVO_ID);
  if (ping < 0) {
    Serial.println("ERROR: No responde el servo.");
    Serial.println("Causas posibles:");
    Serial.println("  1) Baud rate incorrecto (STS3215 default = 1,000,000)");
    Serial.println("  2) Cables mal conectados");
    Serial.println("  3) ID del servo incorrecto");
    while (true);
  }
  Serial.print("Servo detectado. ID=");
  Serial.println(ping);

  // Desbloquear EPROM y poner en modo rueda (modo 1 = giro continuo)
  sms_sts.unLockEprom(SERVO_ID);
  delay(100);
  sms_sts.WheelMode(SERVO_ID);
  delay(100);
  sms_sts.LockEprom(SERVO_ID);
  delay(100);

  // Habilitar torque
  sms_sts.EnableTorque(SERVO_ID, 1);
  delay(100);

  Serial.println("Servo listo en modo rueda con torque activado.");
}

void loop() {
  Serial.println("=== 2 vueltas hacia adelante en 4 segundos ===");
  //rotarVueltas(cantidad[ml], tiempo[s]);
  rotarVueltas(2.0f, 10.0f);
  delay(5000);

/*

  Serial.println("=== 1.5 vueltas hacia atras en 3 segundos ===");
  rotarVueltas(-1.5f, 3.0f);
  delay(2000);
   * 
 */
}
