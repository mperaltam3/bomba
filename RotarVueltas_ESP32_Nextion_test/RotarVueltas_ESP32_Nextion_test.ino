/*
 * RotarVueltas_ESP32_Nextion_test.ino
 * Prueba de envio de alertas graficas a una pantalla Nextion, basada en
 * las interrupciones de RotarVueltas_ESP32_v2. NO controla el motor/servo,
 * solo prueba el envio de datos hacia la Nextion.
 *
 * Nextion:
 *   Conectada a los pines nativos RX0/TX0 del ESP32 (los mismos del USB).
 *   Por eso este sketch NO usa Serial.println para depuracion: el UART0
 *   queda dedicado por completo al protocolo Nextion. Si necesitas ver algo
 *   por el Monitor Serial durante pruebas, desconecta la Nextion primero.
 *   Baud rate: 115200. Debe coincidir con el baud configurado en el
 *   proyecto Nextion (atributo "baud" de la pagina/HMI).
 *
 * Componentes Nextion requeridos (crear en el editor Nextion):
 *   - Picture "p0": imagen de alerta de EMERGENCIA. Colocarla donde debe
 *     verse, con la imagen (.pic) ya asignada, y su atributo vis en 0
 *     (oculta) para que solo aparezca cuando se dispare la interrupcion.
 *   - Picture "p1": imagen de alerta de BURBUJA, mismas condiciones.
 *   Si en tu proyecto ya tienes componentes con otros nombres, cambia
 *   OBJ_EMERGENCIA y OBJ_BURBUJA mas abajo.
 *
 * Pines de interrupcion (igual que en v2):
 *   - GPIO 4  : Parada de emergencia       (FALLING) -> muestra p0
 *   - GPIO 14 : Sensor IR HX-M121 burbuja  (FALLING) -> muestra p1
 *
 * Comportamiento:
 *   Al dispararse la interrupcion correspondiente, se hace visible (vis 1)
 *   el componente Picture de esa alerta. Permanece visible
 *   ALERTA_DURACION_MS y luego se oculta (vis 0) automaticamente.
 */

#define NEXTION_BAUD 115200

// --- Nombres de los componentes Picture en el proyecto Nextion ---
#define OBJ_EMERGENCIA "p0"
#define OBJ_BURBUJA    "p1"

// --- Pines de interrupcion ---
#define PIN_EMERGENCIA  4
#define PIN_IR_BURBUJA  14

// --- Antirebote: tiempo minimo entre dos disparos del mismo pin ---
#define DEBOUNCE_MS 200

// --- Duracion de la alerta en pantalla antes de ocultarse ---
#define ALERTA_DURACION_MS 3000

// --- Flags de interrupcion (accedidos desde ISR y codigo principal) ---
volatile bool flagEmergencia = false;
volatile bool flagIRBurbuja  = false;

// Marca de tiempo del ultimo disparo valido de cada ISR (para antirebote)
volatile unsigned long tUltimaEmergencia = 0;
volatile unsigned long tUltimaIRBurbuja  = 0;

// Estado de visibilidad de cada alerta y su hora de ocultado programada
bool          emergenciaVisible   = false;
bool          burbujaVisible      = false;
unsigned long emergenciaOcultarEn = 0;
unsigned long burbujaOcultarEn    = 0;

// -----------------------------------------------------------------------
// ISR - deben estar en RAM para ESP32
// -----------------------------------------------------------------------
void IRAM_ATTR isrEmergencia() {
  unsigned long ahora = millis();
  if (ahora - tUltimaEmergencia < DEBOUNCE_MS) return;
  tUltimaEmergencia = ahora;
  flagEmergencia    = true;
}

void IRAM_ATTR isrIRBurbuja() {
  unsigned long ahora = millis();
  if (ahora - tUltimaIRBurbuja < DEBOUNCE_MS) return;
  tUltimaIRBurbuja = ahora;
  flagIRBurbuja    = true;
}

// -----------------------------------------------------------------------
// enviarComandoNextion: envia un comando a la Nextion terminado con los
// 3 bytes 0xFF que exige su protocolo.
// -----------------------------------------------------------------------
void enviarComandoNextion(const String &cmd) {
  Serial.print(cmd);
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
}

void mostrarImagenNextion(const char *objeto, bool visible) {
  String cmd = "vis " + String(objeto) + "," + String(visible ? 1 : 0);
  enviarComandoNextion(cmd);
}

// -----------------------------------------------------------------------
// procesarInterrupciones: revisa los flags puestos por las ISR y muestra
// la imagen de alerta correspondiente en la Nextion.
// -----------------------------------------------------------------------
void procesarInterrupciones() {
  if (flagEmergencia) {
    flagEmergencia = false;
    mostrarImagenNextion(OBJ_EMERGENCIA, true);
    emergenciaVisible   = true;
    emergenciaOcultarEn = millis() + ALERTA_DURACION_MS;
  }

  if (flagIRBurbuja) {
    flagIRBurbuja = false;
    mostrarImagenNextion(OBJ_BURBUJA, true);
    burbujaVisible   = true;
    burbujaOcultarEn = millis() + ALERTA_DURACION_MS;
  }
}

// -----------------------------------------------------------------------
// revisarOcultarAlertas: oculta cada imagen de alerta cuando ya paso su
// tiempo de duracion en pantalla.
// -----------------------------------------------------------------------
void revisarOcultarAlertas() {
  unsigned long ahora = millis();

  if (emergenciaVisible && (long)(ahora - emergenciaOcultarEn) >= 0) {
    mostrarImagenNextion(OBJ_EMERGENCIA, false);
    emergenciaVisible = false;
  }

  if (burbujaVisible && (long)(ahora - burbujaOcultarEn) >= 0) {
    mostrarImagenNextion(OBJ_BURBUJA, false);
    burbujaVisible = false;
  }
}

// -----------------------------------------------------------------------
void setup() {
  Serial.begin(NEXTION_BAUD);

  pinMode(PIN_EMERGENCIA, INPUT_PULLUP);
  pinMode(PIN_IR_BURBUJA, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_EMERGENCIA), isrEmergencia, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_IR_BURBUJA), isrIRBurbuja,  FALLING);

  // Ocultar ambas imagenes de alerta al arrancar
  mostrarImagenNextion(OBJ_EMERGENCIA, false);
  mostrarImagenNextion(OBJ_BURBUJA, false);
}

void loop() {
  procesarInterrupciones();
  revisarOcultarAlertas();
}
