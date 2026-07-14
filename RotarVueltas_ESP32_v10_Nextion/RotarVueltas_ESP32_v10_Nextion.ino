/*
 * RotarVueltas_ESP32_v10_Nextion.ino
 * Version de calibracion de RotarVueltas_ESP32_v9_Nextion.
 *
 * Diferencia con v9:
 *   - Controlador de vueltas+tiempo en rotarVueltas(): antes, la velocidad
 *     del servo se calculaba UNA sola vez al inicio (rpm = |n|/t) y se
 *     mantenia fija durante todo el giro; el giro se detenia SOLO quando se
 *     cumplia el tiempo, sin importar cuantas vueltas se hubieran contado
 *     realmente. Eso provocaba que, sobre todo en tiempos largos, el motor
 *     terminara por debajo de las vueltas solicitadas (ej: se pedian 25.80
 *     vueltas y solo se contaban 22.123): la velocidad REAL alcanzada por
 *     el servo (friccion, carga, no linealidad) no es identica a la
 *     velocidad comandada, y ese pequeno error se acumula sin correccion
 *     durante todo el tiempo del giro.
 *
 *     Ahora, ademas de la velocidad inicial (feedforward, igual que antes),
 *     una nueva funcion controlarVelocidadObjetivo() recalcula la velocidad
 *     comandada cada INTERVALO_CONTROL_VELOCIDAD_MS, usando SIEMPRE las
 *     vueltas que YA FALTAN y el tiempo que YA QUEDA (no los valores
 *     originales):
 *         velocidad_necesaria = (vueltas_restantes / tiempo_restante)
 *     Esto es autocorrectivo: si el servo va atrasado respecto de lo
 *     planeado, el tiempo que queda es el mismo pero las vueltas que faltan
 *     son mas, asi que la velocidad necesaria sube para compensar - y
 *     viceversa si va adelantado. Al llegar al final del tiempo solicitado,
 *     las vueltas restantes deberian ser (idealmente) cero, cumpliendo
 *     tiempo Y vueltas a la vez. Si el atraso es tan grande que la velocidad
 *     necesaria supera SPEED_MAX, se satura al maximo (igual de acotado que
 *     antes) y se avisa UNA vez por giro con "[CONTROL] ... saturada".
 *
 *     Cerca de la meta (vueltas restantes menores a
 *     DEADBAND_VUELTAS_RESTANTES) se deja de recalcular y se detiene el
 *     servo, para evitar "chatter" (oscilar velocidad/direccion) por ruido
 *     de lectura cuando el restante es practicamente cero; el motor
 *     permanece quieto hasta que se cumpla el tiempo solicitado.
 *
 *     debugVueltas() ahora tambien recibe "n" y "tTotal" para mostrar, ademas
 *     de las vueltas acumuladas, cuantas vueltas "deberian" llevarse a esa
 *     altura del tiempo transcurrido (objetivo proporcional) y la velocidad
 *     comandada en ese momento - util para ver en vivo, durante la
 *     calibracion, si el controlador esta corrigiendo como se espera.
 *
 *     secuenciaRegreso() NO usa este controlador: su tramo en reversa se
 *     detiene por el Fin de carrera 1 (no por tiempo/vueltas objetivo) y su
 *     ajuste final se detiene por conteo de vueltas puro, ninguno de los
 *     dos tiene un presupuesto de tiempo que cumplir simultaneamente.
 *
 * Diferencia con v8:
 *   - Los mensajes de DEPURACION (los que antes se mandaban con
 *     enviarSerial()) ahora se mandan con la nueva funcion enviarDebug(),
 *     que agrega un salto de linea ("\r\n") al final del texto ANTES del
 *     terminador 0xFF 0xFF 0xFF. Antes, todos los mensajes quedaban pegados
 *     uno tras otro en el Monitor Serial (sin salto de linea), lo que
 *     dificultaba la lectura durante la calibracion.
 *     IMPORTANTE: enviarSerial() (sin el \r\n) se sigue usando tal cual
 *     para los 3 comandos que la Nextion SI interpreta como comandos reales:
 *     "vis ..." (mostrarImagenNextion), "page ..." (cambiarPaginaNextion) y
 *     "bkcmd=0" (setup). Meterles un \r\n de mas ahi formaria parte del
 *     texto del comando y podria invalidarlo. La Nextion de por si descarta
 *     en silencio (gracias a bkcmd=0) cualquier texto que no sea un comando
 *     valido, asi que el \r\n de los mensajes de depuracion no le afecta en
 *     absoluto - el cambio es unicamente cosmetico, para el Monitor Serial.
 *
 * Diferencia con v7:
 *   - Buzzer en PIN_BUZZER (GPIO 27) como retroalimentacion auditiva para
 *     el operador. Se asume buzzer activo en HIGH (si el tuyo es activo en
 *     LOW, invierte HIGH/LOW en beep()). Tres patrones, via beep():
 *       - sonarInicioMovimiento() -> 1 beep corto (150 ms). Se llama al
 *         iniciar rotarVueltas() (el giro de "vueltas tiempo"), al iniciar
 *         secuenciaRegreso() (el tramo en reversa) y al reanudar un giro
 *         pausado con COMANDO_CONTINUAR.
 *       - sonarFinMovimiento() -> 2 beeps cortos. Se llama al terminar
 *         normalmente rotarVueltas() y al terminar secuenciaRegreso()
 *         (despues del ajuste final). NO suena en una cancelacion (esos
 *         casos usan sonarAlerta() en su lugar, dentro de
 *         procesarInterrupciones()/monitorearCorriente()).
 *       - sonarAlerta() -> 3 beeps rapidos. Se llama dentro de
 *         procesarInterrupciones(), una vez por cada evento REAL confirmado
 *         (emergencia, Fin de carrera 1, Fin de carrera 2), y dentro de
 *         monitorearCorriente() cuando se dispara la proteccion por
 *         sobrecorriente.
 *     Nota de diseno: secuenciaRegreso() tiene dos tramos (reversa + ajuste
 *     final hacia adelante), pero se trata como UN solo "movimiento" desde
 *     el punto de vista del operador (presiono 'r' una vez): el beep de
 *     inicio suena solo al arrancar la reversa, y el de fin solo al
 *     terminar el ajuste final, no en cada tramo por separado.
 *
 * Diferencia con v6:
 *   - En rotarVueltas() (el giro normal de "vueltas tiempo"), la sobre-
 *     corriente ya NO cancela el movimiento de inmediato: despues de
 *     detener el motor y mandar la Nextion a PAGINA_SOBRECORRIENTE (eso lo
 *     sigue haciendo monitorearCorriente()), el codigo queda esperando un
 *     comando por Serial/Nextion (mismo UART0):
 *       - COMANDO_REGRESO ('r'/'R'): cancela el giro solicitado y ejecuta
 *         secuenciaRegreso() (regresar a home), igual que si se hubiera
 *         mandado 'r' directamente.
 *       - COMANDO_CONTINUAR ('c'/'C'): reanuda el giro a la MISMA velocidad
 *         que tenia, y el tiempo que estuvo en pausa se descuenta del
 *         presupuesto (tStart se corre hacia adelante esa misma duracion),
 *         para que el giro reanudado siga cumpliendo tanto el tiempo como
 *         la cantidad de vueltas originalmente solicitados - no se
 *         reinicia el calculo, solo se retoma donde quedo.
 *     Si durante esta espera ocurre una interrupcion real (emergencia o
 *     fin de carrera), se cancela todo igual que antes.
 *     secuenciaRegreso() (el comando 'r' independiente) NO cambia: su
 *     sobrecorriente sigue cancelando la secuencia de inmediato, porque ya
 *     es en si misma un "ir a home" y no tiene un objetivo fijo de tiempo/
 *     cantidad al cual "continuar" tenga sentido aplicarle.
 *
 * Diferencia de v6 con v5:
 *   - Proteccion por sobrecorriente: monitorearCorriente() ahora, ademas de
 *     imprimir la corriente cada INTERVALO_MONITOREO_CORRIENTE_MS, compara
 *     su magnitud contra CORRIENTE_MAX_MA (200 mA). Si la supera, detiene
 *     el motor de inmediato, cambia la Nextion a PAGINA_EMERGENCIA (la
 *     misma pantalla de error que usa el boton de emergencia) y mantiene el
 *     motor detenido 5 s, igual que procesarInterrupciones() hace con la
 *     emergencia/fines de carrera. monitorearCorriente() paso de void a
 *     bool: retorna true cuando disparo esta proteccion, y rotarVueltas()
 *     y secuenciaRegreso() revisan ese retorno (igual que ya revisaban
 *     procesarInterrupciones()) para cancelar el movimiento en curso con un
 *     mensaje de depuracion. Es una parada por software (no una interrupcion
 *     de hardware): se evalua una vez por cada intervalo de monitoreo, no
 *     al instante en que la corriente sube.
 *
 * Diferencias de v5 con v3:
 *   - Advertencia de saturacion de velocidad: si "vueltas tiempo" pide una
 *     RPM mayor a la maxima representable (SPEED_MAX * RPM_POR_UNIDAD), la
 *     velocidad se limita a SPEED_MAX igual que antes, pero ahora se avisa
 *     por enviarSerial() ANTES de girar, indicando la RPM maxima real y las
 *     vueltas aproximadas que se alcanzaran en el tiempo pedido. Esto es lo
 *     que probablemente explica que al pedir giros rapidos (p.ej. en
 *     sentido negativo, "vueltas de regreso" en poco tiempo) el conteo
 *     final de vueltas quede muy por debajo de lo solicitado: la formula de
 *     velocidad (rpm = |vueltas|/tiempo) es identica para positivos y
 *     negativos (usa fabsf), asi que NO es un problema de signo/tipos de
 *     variable - es que se pidio mas RPM de la que SPEED_MAX permite
 *     alcanzar, y el motor giro a su maximo posible durante todo el tiempo
 *     sin lograr completar las vueltas pedidas.
 *   - Nuevo comando de una letra 'r' (o 'R'), utilizable tanto desde el
 *     Monitor Serial como desde la Nextion (comparten el mismo UART0):
 *     dispara secuenciaRegreso(), que primero gira en reversa a velocidad
 *     maxima (VELOCIDAD_REGRESO, negativa) de forma INDEFINIDA - sin limite
 *     de tiempo - hasta detectar el Fin de carrera 1; y luego, ya en el Fin
 *     de carrera 1, avanza REGRESO_VUELTAS_FINALES vuelta(s) hacia adelante
 *     tambien a velocidad maxima, deteniendose por conteo de vueltas (no
 *     por tiempo), dejando el eje en el punto exacto para iniciar un nuevo
 *     ciclo de calibracion. Si en el tramo de reversa se dispara emergencia
 *     o el Fin de carrera 2 (en vez de Fin de carrera 1), la secuencia se
 *     cancela sin hacer el ajuste final.
 *
 * Diferencias de v3 con v2:
 *   - Se sigue recibiendo "vueltas tiempo" por el Monitor Serial, igual que
 *     en v2, pero ahora "tiempo" se interpreta en MINUTOS (antes eran
 *     segundos), con un maximo de TIEMPO_MAX_MINUTOS (60 min).
 *   - Se cuentan las vueltas REALES girando mediante la posicion del servo
 *     (mismo metodo que CalibracionRPM.ino: 4096 pasos = 1 vuelta, con
 *     correccion de wraparound) y se muestran periodicamente como mensaje
 *     de depuracion, para poder comparar vueltas solicitadas vs. vueltas
 *     realmente giradas durante la calibracion.
 *   - Se verifica que el motor este realmente girando durante todo el
 *     tiempo solicitado: si la posicion no cambia durante
 *     INTERVALO_VERIFICACION_MOVIMIENTO_MS, se envia una advertencia de
 *     depuracion (posible bloqueo mecanico o falla del servo).
 *   - Filtro de ruido/EMI en las 4 interrupciones (confirmarEventoReal):
 *     GPIO4 (emergencia) y GPIO14 (IR burbuja) son pines RTC/touch del
 *     ESP32 y son mas sensibles a acoplamiento capacitivo (ej. se disparan
 *     solos al tocar una parte metalica cercana al motor). Antes de tratar
 *     un flag como un evento real se exige que el pin siga activo (LOW)
 *     CONFIRMACION_RUIDO_MS completos; si se recupero antes, fue ruido y se
 *     descarta. Esto es ADICIONAL al DEBOUNCE_MS de las ISR (que solo evita
 *     repeticiones tras un primer disparo, no un pulso de ruido aislado).
 *
 *     NOTA: attachInterrupt()/digitalRead() NO activan el periferico de
 *     touch del ESP32 (eso solo ocurre con touchRead()/touchAttachInterrupt(),
 *     que este codigo no usa) - no hay "funcion touch" que desactivar. El
 *     problema es puramente electrico/EMI en pines RTC con pull-up interno
 *     debil; si el filtro de software no basta, ademas conviene: 1) aterrizar
 *     el eje/carcasa del motor al mismo GND del ESP32, 2) poner un capacitor
 *     ceramico de 100 nF entre cada pin de interrupcion y GND (cerca del
 *     ESP32), y 3) si persiste, mover PIN_EMERGENCIA y PIN_IR_BURBUJA a
 *     pines que no sean RTC/touch (ej. 18, 19, 22, 23, 25, 26).
 *   - La burbuja IR es SOLO una alerta visual: el motor NUNCA se detiene por
 *     ella, sigue girando igual mientras se muestra "p1" en la Nextion.
 *     Emergencia y los fines de carrera SI detienen el motor: de inmediato
 *     ante el flag y, si se confirma real, se mantiene detenido 5 s; si
 *     resulta ser ruido, se reanuda sin ese bloqueo.
 *
 * Se mantiene intacta la logica de v2 para:
 *   - Interrupcion de EMERGENCIA (GPIO 4)     -> detiene el motor y cambia a
 *                                                 la pagina 5 de la Nextion.
 *   - Interrupcion de BURBUJA IR (GPIO 14)    -> muestra la imagen de alerta "p1"
 *                                                 durante ALERTA_DURACION_MS y luego
 *                                                 la oculta (NO detiene el motor).
 *   - Antirebote por software en las 4 ISR (DEBOUNCE_MS).
 *   - Bloqueo temporal (detachInterrupt) de cada fin de carrera al disparar,
 *     con rearme automatico (REARME_FIN_CARRERA_MS + pin en HIGH).
 *   - Envio/recepcion de datos hacia la pantalla Nextion por Serial (UART0),
 *     incluyendo bkcmd=0 y el terminador 0xFF 0xFF 0xFF en enviarSerial().
 *
 * Hardware:
 *   - ESP32 WROOM
 *   - Serial2 hardware: RX=16, TX=17 (comunicacion con el servo)
 *   - Serial (UART0, pines nativos RX0/TX0): comunicacion con la Nextion Y
 *     entrada de comandos por el Monitor Serial (mismo cable fisico).
 *
 * IMPORTANTE - por que TODO se envia con enviarSerial()/enviarDebug() y
 * nunca con Serial.print/println directo:
 *   La Nextion no ejecuta un comando hasta detectar 3 bytes 0xFF seguidos.
 *   Si se manda texto de depuracion sin ese terminador, la Nextion lo
 *   concatena con el siguiente comando real y termina rechazando ambos.
 *   enviarSerial() cierra siempre el mensaje con 0xFF 0xFF 0xFF para que
 *   cada envio sea un frame independiente. enviarDebug() es exactamente lo
 *   mismo, pero agrega "\r\n" al texto antes de ese terminador, solo para
 *   que los mensajes de depuracion se vean en lineas separadas en el
 *   Monitor Serial (ver "Diferencia con v8" arriba).
 *   Ademas, en setup() se manda "bkcmd=0" para que la Nextion deje de
 *   contestar con codigos de exito/error por cada comando: esas respuestas
 *   llegaban por el mismo RX0 que usa Serial.parseFloat() para leer
 *   "vueltas tiempo" desde el Monitor Serial y corrompian esa lectura.
 *
 * Pines de interrupcion:
 *   - GPIO 4  : Parada de emergencia       (FALLING)
 *   - GPIO 5  : Fin de carrera 1           (FALLING)
 *   - GPIO 21 : Fin de carrera 2           (FALLING)
 *   - GPIO 14 : Sensor IR HX-M121 burbuja  (FALLING)
 *
 * NOTA: los pines de interrupcion se definen UNA sola vez, en los #define
 * PIN_EMERGENCIA / PIN_FIN_CARRERA1 / PIN_FIN_CARRERA2 / PIN_IR_BURBUJA de
 * mas abajo, y attachInterrupt() los referencia por esos mismos nombres
 * (no por numero literal), para que cambiar un pin ahi sea suficiente y no
 * queden pines "sueltos" desincronizados entre el pinMode() y el
 * attachInterrupt() de cada senal.
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
 * Conteo de vueltas para calibracion:
 *   sms_sts.ReadPos() entrega la posicion del servo en el rango 0-4095
 *   (4096 pasos = 1 vuelta). actualizarConteoVueltas() compara cada lectura
 *   con la anterior y corrige el salto cuando hay wraparound (paso por
 *   0/4095), acumulando pasosAcumulados. vueltasActuales() lo convierte a
 *   vueltas (pasosAcumulados / 4096.0). El mismo conteo alimenta la
 *   verificacion de movimiento (revisarFuncionamientoMotor) y ahora tambien
 *   el controlador de vueltas+tiempo (controlarVelocidadObjetivo).
 *
 * Entrada por monitor serial: vueltas tiempo_en_minutos
 *   Ejemplo: 2.0 5.0  -> 2 vueltas en 5 minutos (maximo 60 minutos).
 *
 * Unidades de velocidad de la libreria:
 *   1 unidad = 0.0146 RPM  (ajustado empiricamente para STS3215)
 *
 * Componente Nextion requerido:
 *   - Picture "p1": imagen de alerta de BURBUJA. Colocarla donde debe
 *     verse, con la imagen (.pic) ya asignada, y su atributo vis inicial
 *     en 0 (oculta). Si tu componente tiene otro nombre, cambia OBJ_BURBUJA.
 *   - Pagina numero 5: pagina de alerta de emergencia en el proyecto Nextion.
 */

#include "D:/DOCUMENTOS MICHAEL PERALTA/BOMBA/motor2/FTServo_Arduino/examples/SMS_STS/RotarVueltas_ESP32_v10_Nextion/SCServo.h"

// --- Pines Serial2 para el servo ---
#define SERVO_RX_PIN  16
#define SERVO_TX_PIN  17

// --- Pines de interrupciones ---
#define PIN_EMERGENCIA    4
#define PIN_FIN_CARRERA1  5
#define PIN_FIN_CARRERA2  21
#define PIN_IR_BURBUJA    14

// --- Buzzer de retroalimentacion auditiva. Se asume activo en HIGH; si el
//     tuyo es activo en LOW, invierte HIGH/LOW dentro de beep(). ---
#define PIN_BUZZER 27

// --- Nextion: baud (debe coincidir con el proyecto Nextion) ---
#define NEXTION_BAUD 115200

// --- Nextion: componente Picture de alerta de burbuja ---
#define OBJ_BURBUJA "p1"

// --- Nextion: pagina a la que se salta en caso de emergencia ---
#define PAGINA_EMERGENCIA 6

// --- Nextion: pagina a la que se salta en caso de sobrecorriente ---
#define PAGINA_SOBRECORRIENTE 5

// --- Nextion: duracion en pantalla de la imagen de alerta de burbuja ---
#define ALERTA_DURACION_MS 3000

SMS_STS sms_sts;

const uint8_t  SERVO_ID       = 4;
const float    RPM_POR_UNIDAD = 0.0146f;
const int16_t  SPEED_MAX      = 4095;

// --- Tiempo maximo permitido para una prueba, en minutos ---
#define TIEMPO_MAX_MINUTOS 60.0f

// --- Comando de regreso a home (letra recibida por Serial/Nextion) ---
#define COMANDO_REGRESO 'r'   // se acepta 'r' o 'R'

// --- Comando de continuar tras una pausa por sobrecorriente (letra
//     recibida por Serial/Nextion, ver esperarComandoSobrecorriente()) ---
#define COMANDO_CONTINUAR 'c'   // se acepta 'c' o 'C'

// --- Regreso: velocidad (magnitud) usada tanto en el tramo en reversa como
//     en el ajuste final hacia adelante. Se usa SPEED_MAX (velocidad
//     maxima): el tramo en reversa lo detiene el Fin de carrera 1 (tope
//     mecanico) y el ajuste final lo detiene el conteo de vueltas, ninguno
//     de los dos depende de un tiempo objetivo. Si el golpe contra el
//     interruptor resulta muy brusco, bajar este valor. ---
#define VELOCIDAD_REGRESO SPEED_MAX

// --- Regreso: ajuste final de posicion una vez tocado el Fin de carrera 1:
//     avanza esta cantidad de vueltas hacia adelante (a VELOCIDAD_REGRESO,
//     sin tiempo objetivo: se detiene solo por conteo de vueltas). ---
#define REGRESO_VUELTAS_FINALES 1.75f

// --- Antirebote: tiempo minimo entre dos disparos del mismo pin ---
#define DEBOUNCE_MS 200

// --- Antirebote de fines de carrera: tiempo minimo desactivada antes de
//     poder reactivarse (ademas de requerir que el pin ya no este presionado) ---
#define REARME_FIN_CARRERA_MS 1000

// --- Monitoreo periodico de corriente del motor ---
#define INTERVALO_MONITOREO_CORRIENTE_MS 1000

// --- Sobrecorriente: si la magnitud de la corriente leida supera este
//     umbral, se detiene el motor y se manda la Nextion a PAGINA_EMERGENCIA,
//     igual que con el boton de emergencia. ---
#define CORRIENTE_MAX_MA 150.0f

// --- Debug de calibracion: cada cuanto se imprime el conteo de vueltas ---
#define INTERVALO_DEBUG_VUELTAS_MS 1000

// --- Verificacion de movimiento: si el servo no cambia de posicion durante
//     este tiempo mientras deberia estar girando, se avisa por depuracion ---
#define INTERVALO_VERIFICACION_MOVIMIENTO_MS 2000

// --- Controlador de vueltas+tiempo (rotarVueltas): cada cuanto se recalcula
//     la velocidad del servo en funcion de las vueltas que faltan y el
//     tiempo que queda, para converger a cumplir AMBOS objetivos en vez de
//     solo el tiempo. Ver controlarVelocidadObjetivo(). ---
#define INTERVALO_CONTROL_VELOCIDAD_MS 300

// --- Controlador de vueltas+tiempo: margen (en vueltas) dentro del cual se
//     considera la meta ya alcanzada. Evita "chatter" (oscilar velocidad o
//     invertir direccion) por ruido de lectura cuando lo que falta es
//     practicamente cero; el motor simplemente se detiene y espera a que se
//     cumpla el tiempo restante. ---
#define DEADBAND_VUELTAS_RESTANTES 0.02f

// --- Filtro de ruido/EMI: GPIO4 (emergencia) y GPIO14 (IR burbuja) son pines
//     RTC/touch del ESP32, mas sensibles a acoplamiento capacitivo (ej. al
//     tocar una parte metalica cercana al motor). El DEBOUNCE_MS de las ISR
//     solo evita repeticiones tras un primer disparo; no filtra un pulso de
//     ruido aislado. Por eso, antes de aceptar un flag como evento real, se
//     confirma que el pin siga activo (LOW) durante CONFIRMACION_RUIDO_MS: un
//     pulso de ruido se recupera casi de inmediato, una pulsacion/senal real
//     se mantiene. Si sube de 20 ms y sigues viendo falsos positivos, sube
//     este valor (o revisa el cableado/blindaje de esos pines). ---
#define CONFIRMACION_RUIDO_MS 20

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

// Estado de visibilidad de la imagen de alerta de burbuja en la Nextion
bool          burbujaVisible   = false;
unsigned long burbujaOcultarEn = 0;

// Ultima velocidad comandada al servo (para poder reanudar el giro sin los
// 5 s de espera cuando un flag resulta ser ruido en vez de un evento real,
// y como referencia para el controlador de vueltas+tiempo)
int16_t velocidadActual = 0;

// Motivo de la ultima parada REAL confirmada por procesarInterrupciones():
// 0 = ninguna, 1 = emergencia, 2 = Fin de carrera 1, 3 = Fin de carrera 2.
// Permite a secuenciaRegreso() distinguir si el motor se detuvo por el Fin
// de carrera 1 (lo esperado) o por otra causa (emergencia, Fin de carrera 2
// inesperado), sin depender de un retorno adicional de esa funcion.
int ultimaTipoParada = 0;

// --- Conteo de vueltas (calibracion) ---
int           posAnteriorVueltas  = 0;
long          pasosAcumulados     = 0;
unsigned long tUltimoDebugVueltas = 0;

// --- Verificacion de movimiento del motor ---
unsigned long tUltimoCambioPosicion     = 0;
bool          advertenciaBloqueoEnviada = false;

// --- Controlador de vueltas+tiempo (solo usado dentro de rotarVueltas()) ---
unsigned long tUltimoControlVelocidad             = 0;
bool          advertenciaSaturacionControlEnviada = false;

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
// enviarSerial: unica funcion que escribe en Serial (UART0). Usarla SIEMPRE
// en vez de Serial.print/println, para que cada mensaje -sea de depuracion
// o un comando Nextion- quede cerrado con el terminador 0xFF 0xFF 0xFF y
// nunca se mezcle con el siguiente envio.
// Uso directo: SOLO para comandos reales de la Nextion ("vis ...",
// "page ...", "bkcmd=0"). Para mensajes de depuracion usar enviarDebug().
// -----------------------------------------------------------------------
void enviarSerial(const String &msg) {
  Serial.print(msg);
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
}

// -----------------------------------------------------------------------
// enviarDebug: igual que enviarSerial(), pero agrega "\r\n" al final del
// texto ANTES del terminador 0xFF 0xFF 0xFF. Usar SIEMPRE para mensajes de
// depuracion/log (nunca para "vis ...", "page ..." o "bkcmd=0"), asi cada
// mensaje aparece en su propia linea en el Monitor Serial. La Nextion
// descarta este texto en silencio igual que antes (ver bkcmd=0 en setup()),
// asi que el \r\n no le afecta.
// -----------------------------------------------------------------------
void enviarDebug(const String &msg) {
  enviarSerial(msg + "\r\n");
}

void mostrarImagenNextion(const char *objeto, bool visible) {
  enviarSerial("vis " + String(objeto) + "," + String(visible ? 1 : 0));
}

void cambiarPaginaNextion(int pagina) {
  enviarSerial("page " + String(pagina));
}

// -----------------------------------------------------------------------
// beep: emite "veces" pitidos de "duracionMs" ms, separados por "pausaMs".
// Bloqueante (usa delay()), igual que el resto del codigo.
// -----------------------------------------------------------------------
void beep(int veces, int duracionMs, int pausaMs) {
  for (int i = 0; i < veces; i++) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(duracionMs);
    digitalWrite(PIN_BUZZER, LOW);
    if (i < veces - 1) delay(pausaMs);
  }
}

// 1 beep corto: inicio de un movimiento (rotarVueltas, secuenciaRegreso, o
// reanudar tras una pausa por sobrecorriente con COMANDO_CONTINUAR).
void sonarInicioMovimiento() {
  beep(1, 150, 0);
}

// 2 beeps cortos: fin normal de un movimiento (no suena en cancelaciones,
// esas usan sonarAlerta()).
void sonarFinMovimiento() {
  beep(2, 150, 100);
}

// 3 beeps rapidos: emergencia, Fin de carrera 1/2, o sobrecorriente reales.
void sonarAlerta() {
  beep(3, 100, 80);
}

// -----------------------------------------------------------------------
// revisarOcultarAlertaBurbuja: oculta la imagen de alerta de burbuja cuando
// ya paso su tiempo de duracion en pantalla. Se debe llamar periodicamente
// desde loop() y desde el while de rotarVueltas().
// -----------------------------------------------------------------------
void revisarOcultarAlertaBurbuja() {
  if (burbujaVisible && (long)(millis() - burbujaOcultarEn) >= 0) {
    mostrarImagenNextion(OBJ_BURBUJA, false);
    burbujaVisible = false;
  }
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
    enviarDebug("[INFO] Interrupcion de Fin de carrera 1 reactivada.");
  }

  if (!interrupcionFC2Activa &&
      (ahora - tDesactivacionFC2 >= REARME_FIN_CARRERA_MS) &&
      digitalRead(PIN_FIN_CARRERA2) == HIGH) {
    attachInterrupt(digitalPinToInterrupt(PIN_FIN_CARRERA2), isrFinCarrera2, FALLING);
    interrupcionFC2Activa = true;
    enviarDebug("[INFO] Interrupcion de Fin de carrera 2 reactivada.");
  }
}

// -----------------------------------------------------------------------
// monitorearCorriente: imprime por Serial la corriente del motor cada
// INTERVALO_MONITOREO_CORRIENTE_MS ms, y si su magnitud supera
// CORRIENTE_MAX_MA, detiene el motor y manda la Nextion a PAGINA_EMERGENCIA
// (misma pantalla que usa el boton de emergencia). Retorna true solo en
// ese caso, para que rotarVueltas()/secuenciaRegreso() puedan cancelar el
// movimiento en curso igual que ya hacen con procesarInterrupciones().
// NOTA: a diferencia de la emergencia/fines de carrera (que son
// interrupciones de hardware), esto es una revision por software que solo
// se evalua una vez por INTERVALO_MONITOREO_CORRIENTE_MS, no al instante.
// -----------------------------------------------------------------------
bool monitorearCorriente() {
  unsigned long ahora = millis();
  if (ahora - tUltimoMonitoreoCorriente < INTERVALO_MONITOREO_CORRIENTE_MS) return false;
  tUltimoMonitoreoCorriente = ahora;

  int corrienteRaw = sms_sts.ReadCurrent(SERVO_ID);
  if (corrienteRaw == -1) {
    enviarDebug("[MONITOREO] Error leyendo corriente del motor.");
    return false;
  }

  float corrienteMA = fabsf(corrienteRaw * 6.5f);
  enviarDebug("[MONITOREO] Corriente del motor: " + String(corrienteMA) + " mA");

  if (corrienteMA > CORRIENTE_MAX_MA) {
    velocidadActual = 0;
    sms_sts.WriteSpe(SERVO_ID, 0, 0);
    enviarDebug("[INTERRUPCION] Sobrecorriente detectada (" + String(corrienteMA, 1) +
                 " mA > " + String(CORRIENTE_MAX_MA, 0) + " mA)! Motor detenido 5 segundos.");
    cambiarPaginaNextion(PAGINA_SOBRECORRIENTE);
    sonarAlerta();
    delay(5000);
    return true;
  }

  return false;
}

// -----------------------------------------------------------------------
// iniciarConteoVueltas: reinicia el acumulador de pasos y las referencias
// de posicion. Llamar justo antes de poner al motor a girar.
// -----------------------------------------------------------------------
void iniciarConteoVueltas() {
  int pos = sms_sts.ReadPos(SERVO_ID);
  if (pos < 0) pos = 0;

  posAnteriorVueltas        = pos;
  pasosAcumulados           = 0;
  tUltimoCambioPosicion     = millis();
  advertenciaBloqueoEnviada = false;
}

// -----------------------------------------------------------------------
// actualizarConteoVueltas: lee la posicion actual del servo y acumula los
// pasos girados, corrigiendo el wraparound (0-4095, 4096 pasos = 1 vuelta
// completa). Mismo metodo que CalibracionRPM.ino. Tambien alimenta la
// verificacion de movimiento del motor y el controlador de vueltas+tiempo.
// -----------------------------------------------------------------------
void actualizarConteoVueltas() {
  int posActual = sms_sts.ReadPos(SERVO_ID);
  if (posActual < 0) return;  // ignorar lectura fallida

  int delta = posActual - posAnteriorVueltas;
  if (delta >  2048) delta -= 4096;
  if (delta < -2048) delta += 4096;

  pasosAcumulados    += delta;
  posAnteriorVueltas  = posActual;

  if (delta != 0) {
    tUltimoCambioPosicion     = millis();
    advertenciaBloqueoEnviada = false;
  }
}

float vueltasActuales() {
  return (float)pasosAcumulados / 4096.0f;
}

// -----------------------------------------------------------------------
// revisarFuncionamientoMotor: verifica que el motor SI este girando durante
// el tiempo solicitado. Si la posicion no cambia durante
// INTERVALO_VERIFICACION_MOVIMIENTO_MS, envia una advertencia de depuracion
// (posible bloqueo mecanico o falla del servo). Solo avisa una vez por
// bloqueo, hasta que el eje vuelva a moverse.
// -----------------------------------------------------------------------
void revisarFuncionamientoMotor() {
  if (advertenciaBloqueoEnviada) return;

  if (millis() - tUltimoCambioPosicion >= INTERVALO_VERIFICACION_MOVIMIENTO_MS) {
    enviarDebug("[ADVERTENCIA] El motor no presenta movimiento hace " +
                 String(INTERVALO_VERIFICACION_MOVIMIENTO_MS / 1000) +
                 " s. Verifique bloqueo mecanico o conexion del servo.");
    advertenciaBloqueoEnviada = true;
  }
}

// -----------------------------------------------------------------------
// controlarVelocidadObjetivo: recalcula periodicamente (cada
// INTERVALO_CONTROL_VELOCIDAD_MS) la velocidad comandada al servo dentro de
// rotarVueltas(), usando SIEMPRE las vueltas que FALTAN (n - vueltasActuales())
// y el tiempo que QUEDA (tTotal - transcurrido), en vez de la velocidad fija
// calculada al inicio:
//     velocidad_necesaria = (vueltas_restantes / tiempo_restante)
// Es autocorrectivo: si el servo va atrasado respecto de lo planeado, el
// tiempo que queda es el mismo pero las vueltas que faltan son mas, asi que
// la velocidad necesaria sube para compensar (y baja si va adelantado). Al
// llegar al final del tiempo solicitado, las vueltas restantes deberian ser
// (idealmente) cero, cumpliendo tiempo Y vueltas a la vez.
// Si el atraso es tan grande que la velocidad necesaria supera SPEED_MAX, se
// satura al maximo (no se puede pedir mas velocidad de la que el servo
// alcanza) y se avisa UNA sola vez por giro. Cerca de la meta (vueltas
// restantes menores a DEADBAND_VUELTAS_RESTANTES) se detiene el recalculo y
// el servo, para no oscilar velocidad/direccion por ruido de lectura cuando
// lo que falta es practicamente cero.
// No se llama cuando el tiempo restante es menor a un intervalo de control,
// para evitar que el calculo se dispare por dividir entre un tiempo casi
// nulo; en ese tramo final se mantiene la ultima velocidad comandada.
// -----------------------------------------------------------------------
void controlarVelocidadObjetivo(float n, unsigned long tStart, unsigned long tTotal) {
  unsigned long ahora = millis();
  if (ahora - tUltimoControlVelocidad < INTERVALO_CONTROL_VELOCIDAD_MS) return;
  tUltimoControlVelocidad = ahora;

  long tRestanteMs = (long)tTotal - (long)(ahora - tStart);
  if (tRestanteMs < (long)INTERVALO_CONTROL_VELOCIDAD_MS) return;  // tramo final: no recalcular

  float vueltasRestantes = n - vueltasActuales();

  if (fabsf(vueltasRestantes) < DEADBAND_VUELTAS_RESTANTES) {
    if (velocidadActual != 0) {
      velocidadActual = 0;
      sms_sts.WriteSpe(SERVO_ID, 0, 0);
    }
    return;
  }

  float   tRestanteSeg = tRestanteMs / 1000.0f;
  float   rpmNecesaria = (vueltasRestantes / tRestanteSeg) * 60.0f;
  float   crudo        = rpmNecesaria / RPM_POR_UNIDAD;
  int16_t speedNuevo   = (int16_t)(crudo + (crudo >= 0.0f ? 0.5f : -0.5f));

  if (speedNuevo > SPEED_MAX || speedNuevo < -SPEED_MAX) {
    speedNuevo = (speedNuevo > 0) ? SPEED_MAX : -SPEED_MAX;
    if (!advertenciaSaturacionControlEnviada) {
      enviarDebug("[CONTROL] Velocidad de correccion saturada al maximo (" + String(SPEED_MAX) +
                   "): no se alcanzara a compensar todo el atraso en el tiempo restante.");
      advertenciaSaturacionControlEnviada = true;
    }
  }
  if (speedNuevo == 0) speedNuevo = (vueltasRestantes > 0) ? 1 : -1;

  if (speedNuevo != velocidadActual) {
    velocidadActual = speedNuevo;
    sms_sts.WriteSpe(SERVO_ID, speedNuevo, 0);
  }
}

// -----------------------------------------------------------------------
// debugVueltas: imprime periodicamente (mensaje de depuracion para la fase
// de calibracion) las vueltas acumuladas, cuantas "deberian" llevarse a esa
// altura del tiempo transcurrido (objetivo proporcional: n * transcurrido/
// tTotal) y la velocidad comandada en ese momento - util para ver en vivo si
// el controlador de vueltas+tiempo esta corrigiendo como se espera.
// -----------------------------------------------------------------------
void debugVueltas(unsigned long tInicio, float n, unsigned long tTotal) {
  unsigned long ahora = millis();
  if (ahora - tUltimoDebugVueltas < INTERVALO_DEBUG_VUELTAS_MS) return;
  tUltimoDebugVueltas = ahora;

  float transcurridoSeg = (ahora - tInicio) / 1000.0f;
  float vueltasObjetivo = n * ((float)(ahora - tInicio) / (float)tTotal);

  enviarDebug("[CALIBRACION] Vueltas acumuladas: " + String(vueltasActuales(), 3) +
               " | Objetivo a este tiempo: " + String(vueltasObjetivo, 3) +
               " | Transcurrido: " + String(transcurridoSeg, 1) +
               " s | Velocidad: " + String(velocidadActual));
}

// -----------------------------------------------------------------------
// confirmarEventoReal: exige que "pin" siga activo (LOW) durante todo
// CONFIRMACION_RUIDO_MS antes de aceptar el flag correspondiente como un
// evento real. Un pulso de ruido/EMI (ej. al tocar el eje metalico del
// motor) genera un FALLING momentaneo que se recupera casi de inmediato;
// una pulsacion real de fin de carrera, la parada de emergencia o el
// sensor IR se mantiene. Distinto del antirebote DEBOUNCE_MS de las ISR,
// que solo evita repeticiones despues del primer disparo.
// -----------------------------------------------------------------------
bool confirmarEventoReal(uint8_t pin) {
  unsigned long inicio = millis();
  while (millis() - inicio < CONFIRMACION_RUIDO_MS) {
    if (digitalRead(pin) == HIGH) return false;  // se recupero antes de tiempo: fue ruido
    delay(1);
  }
  return true;
}

// -----------------------------------------------------------------------
// procesarInterrupciones: la burbuja IR NUNCA detiene el motor - solo es
// una alerta visual en la Nextion, el motor sigue girando igual. Emergencia
// y los fines de carrera SI detienen el motor: de inmediato ante el flag
// (para no perder reaccion), y luego se confirman con confirmarEventoReal().
// Si esa parada resulta ser ruido, se reanuda el giro a la misma velocidad
// sin los 5 s de espera; si es real, se procesa (mensaje, pagina Nextion,
// bloqueo de fin de carrera) y el motor queda detenido 5 s.
// Retorna true si el motor quedo detenido por una interrupcion real de
// parada (emergencia o fin de carrera) - la burbuja nunca hace que retorne
// true, para no cancelar la rotacion en curso.
// -----------------------------------------------------------------------
bool procesarInterrupciones() {
  if (!hayInterrupcion) return false;
  hayInterrupcion = false;

  bool huboEmergencia   = flagEmergencia;
  bool huboFinCarrera1  = flagFinCarrera1;
  bool huboFinCarrera2  = flagFinCarrera2;
  bool huboIRBurbuja    = flagIRBurbuja;
  flagEmergencia  = false;
  flagFinCarrera1 = false;
  flagFinCarrera2 = false;
  flagIRBurbuja   = false;

  // La burbuja se confirma y se muestra como alerta, pero de forma
  // totalmente independiente de la logica de parada del motor.
  if (huboIRBurbuja && confirmarEventoReal(PIN_IR_BURBUJA)) {
    enviarDebug("[ALERTA] Burbuja detectada por sensor IR HX-M121. El motor continua girando.");
    mostrarImagenNextion(OBJ_BURBUJA, true);
    burbujaVisible   = true;
    burbujaOcultarEn = millis() + ALERTA_DURACION_MS;
  }

  bool huboParada = huboEmergencia || huboFinCarrera1 || huboFinCarrera2;
  if (!huboParada) return false;   // solo hubo burbuja (o ruido en burbuja): el motor no se toca

  sms_sts.WriteSpe(SERVO_ID, 0, 0);   // detener motor de inmediato, sin esperar la confirmacion

  bool emergenciaReal  = huboEmergencia  && confirmarEventoReal(PIN_EMERGENCIA);
  bool finCarrera1Real = huboFinCarrera1 && confirmarEventoReal(PIN_FIN_CARRERA1);
  bool finCarrera2Real = huboFinCarrera2 && confirmarEventoReal(PIN_FIN_CARRERA2);

  if (!emergenciaReal && !finCarrera1Real && !finCarrera2Real) {
    enviarDebug("[FILTRO] Pulso descartado por ruido/EMI (pin ya no estaba activo). Motor reanudado.");
    sms_sts.WriteSpe(SERVO_ID, velocidadActual, 0);
    return false;
  }

  // Evento real: el motor queda detenido. velocidadActual en 0 evita que un
  // futuro pulso de ruido (mientras esta detenido) lo vuelva a poner en marcha.
  velocidadActual = 0;

  if (emergenciaReal) {
    ultimaTipoParada = 1;
    enviarDebug("[INTERRUPCION] Parada de emergencia detectada! Motor detenido 5 segundos.");
    cambiarPaginaNextion(PAGINA_EMERGENCIA);
    sonarAlerta();
  }
  if (finCarrera1Real) {
    ultimaTipoParada = 2;
    enviarDebug("[INTERRUPCION] Fin de carrera 1 detectado! Motor detenido 5 segundos.");
    sonarAlerta();
    if (interrupcionFC1Activa) {
      detachInterrupt(digitalPinToInterrupt(PIN_FIN_CARRERA1));
      interrupcionFC1Activa = false;
      tDesactivacionFC1     = millis();
      enviarDebug("[INFO] Interrupcion de Fin de carrera 1 desactivada temporalmente.");
    }
  }
  if (finCarrera2Real) {
    ultimaTipoParada = 3;
    enviarDebug("[INTERRUPCION] Fin de carrera 2 detectado! Motor detenido 5 segundos.");
    sonarAlerta();
    if (interrupcionFC2Activa) {
      detachInterrupt(digitalPinToInterrupt(PIN_FIN_CARRERA2));
      interrupcionFC2Activa = false;
      tDesactivacionFC2     = millis();
      enviarDebug("[INFO] Interrupcion de Fin de carrera 2 desactivada temporalmente.");
    }
  }

  delay(5000);
  return true;
}

// -----------------------------------------------------------------------
// esperarComandoSobrecorriente: se llama SOLO desde rotarVueltas() justo
// despues de que monitorearCorriente() detecta sobrecorriente (motor ya
// detenido y Nextion ya en PAGINA_SOBRECORRIENTE). Bloquea esperando que el
// operador decida por Serial/Nextion (mismo UART0):
//   COMANDO_REGRESO   ('r'/'R') -> retorna 1 (ir a home)
//   COMANDO_CONTINUAR ('c'/'C') -> retorna 0 (reanudar el giro)
// Mientras espera, sigue vigilando emergencia/fines de carrera: si ocurre
// una interrupcion real durante la espera, corta la espera y retorna 2
// (cancelar todo, ni home ni continuar). Cualquier otro byte recibido
// (respuestas/eventos de la Nextion, o un numero tecleado por error) se
// descarta en silencio.
// -----------------------------------------------------------------------
int esperarComandoSobrecorriente() {
  while (true) {
    if (procesarInterrupciones()) return 2;
    revisarRearmeFinesDeCarrera();
    revisarOcultarAlertaBurbuja();

    if (Serial.available()) {
      int b = Serial.read();
      if (b == 'r' || b == 'R') {
        while (Serial.available()) Serial.read();
        return 1;
      }
      if (b == 'c' || b == 'C') {
        while (Serial.available()) Serial.read();
        return 0;
      }
      // byte espurio: se ignora y se sigue esperando
    }
    delay(10);
  }
}

// -----------------------------------------------------------------------
// secuenciaRegreso: rutina de "home", activada con el comando COMANDO_REGRESO
// ('r'/'R') recibido por Serial (Monitor Serial o Nextion, mismo UART0).
//   1) Gira en reversa a VELOCIDAD_REGRESO (velocidad maxima, negativa) de
//      forma INDEFINIDA - sin limite de tiempo - hasta que se confirme el
//      Fin de carrera 1. Solo se corta antes si salta una emergencia o
//      (inesperadamente) el Fin de carrera 2.
//   2) Confirmado el Fin de carrera 1, avanza REGRESO_VUELTAS_FINALES
//      vuelta(s) hacia adelante tambien a VELOCIDAD_REGRESO (velocidad
//      maxima), deteniendose por conteo de vueltas (no por tiempo), y deja
//      el eje en el punto exacto para iniciar un nuevo ciclo.
// Ninguno de los dos tramos usa el controlador de vueltas+tiempo (no tienen
// un presupuesto de tiempo objetivo que cumplir simultaneamente): se
// detienen por Fin de carrera 1 y por conteo de vueltas, respectivamente.
// Si el tramo de reversa termina por emergencia o Fin de carrera 2 en vez
// de Fin de carrera 1, la secuencia se cancela y NO se hace el ajuste
// final, para no aplicar el offset de home a partir de una posicion
// incorrecta.
// -----------------------------------------------------------------------
void secuenciaRegreso() {
  enviarDebug("[REGRESO] Iniciando: reversa indefinida hasta Fin de carrera 1...");
  sonarInicioMovimiento();

  ultimaTipoParada = 0;
  iniciarConteoVueltas();
  velocidadActual = -VELOCIDAD_REGRESO;
  sms_sts.WriteSpe(SERVO_ID, -VELOCIDAD_REGRESO, 0);

  bool detenidoPorFC1           = false;
  bool detenidoPorSobrecorriente = false;

  while (true) {
    if (procesarInterrupciones()) {
      detenidoPorFC1 = (ultimaTipoParada == 2);
      break;
    }
    revisarRearmeFinesDeCarrera();
    revisarOcultarAlertaBurbuja();
    if (monitorearCorriente()) {
      detenidoPorSobrecorriente = true;
      break;
    }
    actualizarConteoVueltas();
    delay(10);
  }

  if (detenidoPorSobrecorriente) {
    enviarDebug("[REGRESO] Cancelado por sobrecorriente. Vueltas recorridas: " +
                 String(vueltasActuales(), 3) + ". No se hizo el ajuste final.");
    return;
  }

  if (!detenidoPorFC1) {
    enviarDebug("[REGRESO] Cancelado: la parada fue por emergencia o Fin de carrera 2, no por "
                 "Fin de carrera 1. Vueltas recorridas: " + String(vueltasActuales(), 3) +
                 ". No se hizo el ajuste final.");
    return;
  }

  enviarDebug("[REGRESO] Fin de carrera 1 confirmado. Vueltas de regreso: " + String(vueltasActuales(), 3));
  enviarDebug("[REGRESO] Ajuste final: " + String(REGRESO_VUELTAS_FINALES, 1) +
               " vuelta(s) hacia adelante a velocidad maxima...");

  iniciarConteoVueltas();
  velocidadActual = VELOCIDAD_REGRESO;
  sms_sts.WriteSpe(SERVO_ID, VELOCIDAD_REGRESO, 0);

  while (vueltasActuales() < REGRESO_VUELTAS_FINALES) {
    if (procesarInterrupciones()) {
      enviarDebug("[REGRESO] Ajuste final cancelado por interrupcion. Vueltas avanzadas: " +
                   String(vueltasActuales(), 3));
      return;
    }
    revisarRearmeFinesDeCarrera();
    revisarOcultarAlertaBurbuja();
    if (monitorearCorriente()) {
      enviarDebug("[REGRESO] Ajuste final cancelado por sobrecorriente. Vueltas avanzadas: " +
                   String(vueltasActuales(), 3));
      return;
    }
    actualizarConteoVueltas();
    delay(10);
  }

  velocidadActual = 0;
  sms_sts.WriteSpe(SERVO_ID, 0, 0);
  delay(50);
  actualizarConteoVueltas();

  enviarDebug("[REGRESO] Completado. Vueltas avanzadas: " + String(vueltasActuales(), 3) +
               ". Posicion lista para iniciar un nuevo ciclo.");
  sonarFinMovimiento();
}

// -----------------------------------------------------------------------
// rotarVueltas: gira n vueltas en t minutos (maximo TIEMPO_MAX_MINUTOS).
// Cuenta las vueltas reales mediante la posicion del servo, y usa
// controlarVelocidadObjetivo() para recalcular la velocidad periodicamente
// en funcion de las vueltas que faltan y el tiempo que queda, de forma que
// el giro converja a cumplir AMBOS objetivos (tiempo Y vueltas) en vez de
// solo el tiempo. Tambien muestra el progreso periodicamente por depuracion
// (fase de calibracion) y verifica que el motor si este girando durante
// todo el tiempo solicitado.
// Se cancela si se detecta una interrupcion durante la rotacion.
// -----------------------------------------------------------------------
void rotarVueltas(float n, float minutos) {
  if (minutos <= 0.0f || minutos > TIEMPO_MAX_MINUTOS || n == 0.0f) return;

  float   t        = minutos * 60.0f;  // segundos, para el calculo de RPM
  float   rpm      = (fabsf(n) / t) * 60.0f;
  int16_t speedVal = (int16_t)(rpm / RPM_POR_UNIDAD + 0.5f);

  if (speedVal < 1) speedVal = 1;

  // La RPM pedida (|n|/t) puede superar la maxima representable con
  // SPEED_MAX; en ese caso el motor gira a su maximo durante TODO el
  // tiempo solicitado sin llegar a completar las vueltas pedidas (y el
  // controlador de vueltas+tiempo tampoco podra compensarlo, porque ya
  // esta usando el maximo posible). Esto ocurre igual para giros positivos
  // y negativos (la formula usa fabsf), pero es mas facil de notar en giros
  // rapidos (ej. "vueltas de regreso" en poco tiempo). Se avisa ANTES de
  // girar para que quede claro que no es un error de signo/tipos sino una
  // velocidad no alcanzable.
  if (speedVal > SPEED_MAX) {
    float rpmMax      = SPEED_MAX * RPM_POR_UNIDAD;
    float vueltasMax  = rpmMax * t / 60.0f;
    enviarDebug("[ADVERTENCIA] RPM solicitada (" + String(rpm, 1) +
                 ") supera la maxima alcanzable (" + String(rpmMax, 1) +
                 " rpm). Se limitara la velocidad: en " + String(minutos, 2) +
                 " min se completaran aprox. " + String(vueltasMax, 2) +
                 " vuelta(s) de las " + String(fabsf(n), 2) + " solicitadas.");
    speedVal = SPEED_MAX;
  }
  if (n < 0) speedVal = -speedVal;

  iniciarConteoVueltas();
  velocidadActual = speedVal;
  sms_sts.WriteSpe(SERVO_ID, speedVal, 0);
  sonarInicioMovimiento();

  unsigned long tStart = millis();
  unsigned long tTotal = (unsigned long)(minutos * 60000.0f);
  tUltimoDebugVueltas                 = tStart;
  tUltimoControlVelocidad             = tStart;
  advertenciaSaturacionControlEnviada = false;

  while (millis() - tStart < tTotal) {
    if (procesarInterrupciones()) {
      enviarDebug("Rotacion cancelada por interrupcion. Vueltas acumuladas: " +
                   String(vueltasActuales(), 3));
      return;
    }
    revisarRearmeFinesDeCarrera();
    revisarOcultarAlertaBurbuja();
    if (monitorearCorriente()) {
      enviarDebug("[SOBRECORRIENTE] Movimiento pausado. Envia '" + String(COMANDO_REGRESO) +
                   "' para ir a home o '" + String(COMANDO_CONTINUAR) +
                   "' para continuar (se respetan el tiempo y las vueltas pendientes).");

      unsigned long tPausaInicio = millis();
      int decision = esperarComandoSobrecorriente();

      if (decision == 2) {
        enviarDebug("Rotacion cancelada por interrupcion durante la pausa de sobrecorriente. "
                     "Vueltas acumuladas: " + String(vueltasActuales(), 3));
        return;
      }
      if (decision == 1) {
        enviarDebug("Rotacion cancelada por sobrecorriente: yendo a home. Vueltas acumuladas: " +
                     String(vueltasActuales(), 3));
        secuenciaRegreso();
        return;
      }

      // decision == 0 (COMANDO_CONTINUAR): el tiempo en pausa NO cuenta
      // contra el presupuesto de tTotal, para que el giro reanudado siga
      // cumpliendo tanto el tiempo como las vueltas originalmente pedidos.
      // La velocidad se retoma con el valor nominal (speedVal) como punto
      // de partida; controlarVelocidadObjetivo() la recalculara en el
      // siguiente ciclo de control usando el tiempo restante ya actualizado.
      tStart += (millis() - tPausaInicio);
      velocidadActual = speedVal;
      sms_sts.WriteSpe(SERVO_ID, speedVal, 0);
      sonarInicioMovimiento();
      enviarDebug("[SOBRECORRIENTE] Reanudando movimiento...");
    }
    actualizarConteoVueltas();
    revisarFuncionamientoMotor();
    controlarVelocidadObjetivo(n, tStart, tTotal);
    debugVueltas(tStart, n, tTotal);
    delay(10);
  }

  velocidadActual = 0;
  sms_sts.WriteSpe(SERVO_ID, 0, 0);
  delay(50);
  actualizarConteoVueltas();

  int posFinal = sms_sts.ReadPos(SERVO_ID);
  enviarDebug("<< Detenido. Pos=" + String(posFinal) +
               " | Vueltas contadas: " + String(vueltasActuales(), 3) +
               " (solicitadas: " + String(n) + ")");
  sonarFinMovimiento();
}

// -----------------------------------------------------------------------
void setup() {
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  Serial.begin(NEXTION_BAUD);
  Serial2.begin(115200, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  sms_sts.pSerial = &Serial2;
  delay(1000);

  // Silenciar las respuestas automaticas de la Nextion (exito/error por
  // cada comando). Deben dejar de llegar por RX0 antes de que empecemos a
  // leer "vueltas tiempo" desde el Monitor Serial por ese mismo pin.
  // Comando real de la Nextion: se usa enviarSerial() directo, sin \r\n.
  enviarSerial("bkcmd=0");
  delay(50);
  while (Serial.available()) Serial.read();  // descartar cualquier respuesta residual

  // Configurar pines de interrupcion con pull-up interno
  pinMode(PIN_EMERGENCIA,   INPUT_PULLUP);
  pinMode(PIN_FIN_CARRERA1, INPUT_PULLUP);
  pinMode(PIN_FIN_CARRERA2, INPUT_PULLUP);
  pinMode(PIN_IR_BURBUJA, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_EMERGENCIA),   isrEmergencia,  FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_FIN_CARRERA1), isrFinCarrera1, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_FIN_CARRERA2), isrFinCarrera2, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_IR_BURBUJA),   isrIRBurbuja,   FALLING);

  // Ocultar la imagen de alerta de burbuja al arrancar
  mostrarImagenNextion(OBJ_BURBUJA, false);

  // Verificar comunicacion con el servo
  int ping = sms_sts.Ping(SERVO_ID);
  if (ping < 0) {
    enviarDebug("ERROR: No responde el servo. Verifica:\r\n"
                 "  1) Baud rate (STS3215 default = 1,000,000 - debe estar en 115200)\r\n"
                 "  2) Cables en RX=16, TX=17\r\n"
                 "  3) ID del servo");
    while (true) delay(1000);
  }
  enviarDebug("Servo detectado. ID=" + String(ping));

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

  enviarDebug("Servo listo. Interrupciones activas. Modo calibracion.\r\n"
               "Ingresa: vueltas tiempo_en_minutos   (ej: 2.0 5.0, max " +
               String((int)TIEMPO_MAX_MINUTOS) + " min)\r\n"
               "O envia '" + String(COMANDO_REGRESO) + "' para regresar a home "
               "(reversa hasta Fin de carrera 1 + ajuste de " +
               String(REGRESO_VUELTAS_FINALES, 1) + " vuelta(s)).\r\n"
               "Si hay sobrecorriente durante un giro, queda en pausa: '" +
               String(COMANDO_REGRESO) + "' = ir a home, '" +
               String(COMANDO_CONTINUAR) + "' = continuar.");
}

void loop() {
  procesarInterrupciones();
  revisarRearmeFinesDeCarrera();
  revisarOcultarAlertaBurbuja();
  monitorearCorriente();

  if (Serial.available()) {
    int primerByte = Serial.peek();

    // Comando de una letra ('r'/'R'): dispara la secuencia de regreso a
    // home. Se acepta tanto desde el Monitor Serial como desde la Nextion,
    // ya que comparten el mismo UART0.
    if (primerByte == 'r' || primerByte == 'R') {
      Serial.read();
      while (Serial.available()) Serial.read();  // limpiar resto de la linea
      secuenciaRegreso();
      return;
    }

    bool esInicioDeNumero = (primerByte == '-' || primerByte == '+' || primerByte == '.' ||
                             (primerByte >= '0' && primerByte <= '9'));

    // Byte espurio (respuesta/evento de la Nextion, no un numero real):
    // descartarlo en silencio, sin interpretarlo como comando invalido.
    if (!esInicioDeNumero) {
      Serial.read();
      return;
    }

    float vueltas = Serial.parseFloat()*25.8/60.0;
    float tiempo  = Serial.parseFloat();  // minutos

    // Limpiar cualquier basura restante en el buffer
    while (Serial.available()) Serial.read();

    if (vueltas != 0.0f && tiempo > 0.0f && tiempo <= TIEMPO_MAX_MINUTOS) {
      enviarDebug("Rotando " + String(vueltas) + " vuelta(s) en " + String(tiempo) + " minuto(s)...");
      rotarVueltas(vueltas, tiempo);
      enviarDebug("Listo. Ingresa: vueltas tiempo_en_minutos");
    } else if (tiempo > TIEMPO_MAX_MINUTOS) {
      enviarDebug("Tiempo invalido: excede el maximo permitido de " +
                   String((int)TIEMPO_MAX_MINUTOS) + " minutos.");
    } else {
      enviarDebug("Formato invalido. Usa: vueltas tiempo_en_minutos   (ej: 2.0 5.0, max " +
                   String((int)TIEMPO_MAX_MINUTOS) + " min)");
    }
  }
}
