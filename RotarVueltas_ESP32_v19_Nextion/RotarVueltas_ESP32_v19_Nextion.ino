/*
 * RotarVueltas_ESP32_v19_Nextion.ino
 * Version de calibracion de RotarVueltas_ESP32_v18_Nextion.
 *
 * Diferencia con v18:
 *   - El sensor IR de burbuja (HX-M121) cambia de deteccion por flanco de
 *     BAJADA (FALLING) a flanco de SUBIDA (RISING): se confirmo con pruebas
 *     de hardware (ver TestSensorIR.ino) que en el cableado real, cuando una
 *     burbuja de aire cruza la manguera y la luz del emisor (PIN_IR_TX)
 *     llega directo al receptor (PIN_IR_BURBUJA), el pin SUBE a HIGH (no cae
 *     a LOW como se asumia en v18). activarSensorIR() ahora arma la
 *     interrupcion con RISING en vez de FALLING; el pin sigue en
 *     INPUT_PULLUP, sin cambios ahi.
 *
 *   - Nuevo antirebote DEDICADO para la burbuja, mucho mas largo que el
 *     DEBOUNCE_MS general (200 ms, el que usan emergencia y fines de
 *     carrera): DEBOUNCE_IR_BURBUJA_MS (7000 ms, dentro del rango util de
 *     5000-10000 ms). Motivo: fisicamente no pueden pasar dos burbujas
 *     separadas por menos de unos segundos, asi que una ventana tan corta
 *     como 200 ms dejaba que un mismo evento (o rebote electrico del
 *     fototransistor) disparara la interrupcion varias veces seguidas. La
 *     ISR isrIRBurbuja() ahora compara contra DEBOUNCE_IR_BURBUJA_MS en vez
 *     de DEBOUNCE_MS; las demas ISR (emergencia, fin de carrera 1 y 2) no
 *     cambian, siguen usando DEBOUNCE_MS.
 *
 *   - confirmarEventoReal() ahora recibe un segundo parametro (nivelActivo:
 *     LOW o HIGH) en vez de asumir siempre LOW. Con la burbuja detectando
 *     por RISING (activa en HIGH), el filtro de ruido de v18 (que exigia que
 *     el pin siguiera en LOW durante CONFIRMACION_RUIDO_MS) quedaba invertido
 *     y habria descartado como "ruido" cualquier deteccion real de burbuja.
 *     Ahora cada llamado pasa el nivel que corresponde a SU flanco: LOW para
 *     emergencia/fin de carrera 1/fin de carrera 2 (FALLING, sin cambios), y
 *     HIGH para la burbuja (RISING, nuevo).
 *
 *   - Sin cambios en la logica de activacion/desactivacion del sensor: sigue
 *     encendido (emisor + interrupcion) SOLO durante rotarVueltas() con
 *     n > 0 (giro hacia adelante en modo funcionamiento normal), y apagado
 *     en calibracion (calibrarCorrientePorVelocidad()), regreso a home
 *     (secuenciaRegreso()) y giro en reversa (n < 0) - ver activarSensorIR()/
 *     desactivarSensorIR(), sin modificar. La burbuja sigue sin detener el
 *     motor: solo muestra la imagen "p1" en la Nextion durante
 *     ALERTA_DURACION_MS (ver procesarInterrupciones()/revisarOcultarAlertaBurbuja()).
 *
 * Diferencia con v17:
 *   - El margen porcentual de sobrecorriente (antes MARGEN_CORRIENTE_
 *     PORCENTAJE, un #define fijo que exigia recompilar y recargar el
 *     codigo para cambiarlo) ahora es la variable margenCorrientePorcentaje,
 *     ajustable EN CALIENTE por Serial/Nextion (mismo UART0 que el resto de
 *     comandos) con el nuevo comando COMANDO_MARGEN_CORRIENTE: se envia la
 *     letra 'T' (o 't') seguida del numero, SIN espacio, ej. "T69" deja el
 *     margen en 69%. Se valida contra MARGEN_CORRIENTE_PORCENTAJE_MIN/MAX
 *     (0-300%); un valor invalido o fuera de rango se rechaza y se avisa sin
 *     tocar el margen vigente.
 *
 *     El valor aplicado se guarda de inmediato en la memoria NVS (mismo
 *     namespace que la calibracion de corriente, ver guardarMargenCorriente()/
 *     cargarMargenCorriente()), asi que sobrevive apagados/reinicios igual
 *     que la tabla de calibracion - no hace falta reenviarlo cada vez que se
 *     energiza el equipo. MARGEN_CORRIENTE_PORCENTAJE_DEFECTO (40%) sigue
 *     siendo el valor de arranque SOLO la primera vez que se usa el equipo
 *     (o si se borra la memoria NVS).
 *
 *     setup() imprime el margen vigente al arrancar, y monitorearCorriente()/
 *     corrienteUmbralParaVelocidad() usan margenCorrientePorcentaje en vivo -
 *     un cambio con "T" durante un giro en curso se aplica desde la
 *     siguiente vez que monitorearCorriente() evalue la corriente (hasta
 *     INTERVALO_MONITOREO_CORRIENTE_MS despues). MARGEN_CORRIENTE_MINIMO_MA
 *     (el piso aditivo en mA de v17) sigue siendo fijo de compilacion; no se
 *     pidio hacerlo ajustable.
 *
 *   - Tabla de calibracion de corriente re-enfocada: la calibracion seguia
 *     sin funcionar bien a velocidades muy bajas (que es donde realmente se
 *     opera casi siempre). Nuevo techo VELOCIDAD_MAX_CALIBRACION_CORRIENTE
 *     (2000, en vez de SPEED_MAX = 4095): no hace falta calibrar mas alla,
 *     porque en el uso real casi no se llega a esas velocidades altas.
 *     Los mismos 10 puntos (NUM_PUNTOS_CALIBRACION_CORRIENTE) ahora quedan
 *     TODOS dentro de 0-2000, con una progresion mucho mas concentrada en
 *     el extremo bajo (1%, 2.5%, 4.5%, 7%, 11%, 17.5%, 27.5%, 42.5%, 65%,
 *     100% de ese techo -> velocidades 20, 50, 90, 140, 220, 350, 550, 850,
 *     1300, 2000): los primeros 5 puntos caen entre 20 y 220, todos por
 *     debajo del punto mas bajo que tenia v17 (81). Por encima de 2000,
 *     corrienteUmbralParaVelocidad() sigue sin extrapolar - usa el umbral
 *     del punto de 2000 tal cual, igual que siempre hizo con el extremo
 *     superior de la tabla.
 *
 * Diferencia con v16:
 *   - CORREGIDO: corrienteUmbralParaVelocidad() tenia una formula rota
 *     (umbral = normal * (0.79 + margen/100)) que dejaba el umbral de
 *     disparo POR DEBAJO de la propia corriente normal calibrada, en vez de
 *     por encima. Con eso, apenas se retomaba un giro (o incluso en
 *     operacion normal, una vez calibrado) el umbral quedaba "bajo" y
 *     cualquier lectura normal ya lo superaba - la causa mas probable de que
 *     "el umbral se quede bajo" al continuar tras una pausa. Restaurada la
 *     formula correcta: umbral = normal * (1 + margen/100), SIEMPRE por
 *     encima de la corriente normal medida para esa velocidad. El umbral
 *     sigue calculandose con corrienteUmbralParaVelocidad(velocidadActual):
 *     usa siempre la velocidad ACTUAL, nunca una velocidad vieja/congelada.
 *
 *   - MARGEN_CORRIENTE_PORCENTAJE bajado de 40% (v15) a 12% por defecto:
 *     mucho mas sensible, pensado para que un leve empujon con el dedo
 *     sobre el eje ya alcance a dispararlo. Es un punto de partida: ajustalo
 *     hacia arriba si dispara solo, o hacia abajo si un toque leve no lo
 *     dispara (ver comentario junto al #define).
 *
 *   - Lectura de corriente mejorada: leerCorrienteMA() antes hacia una sola
 *     lectura cruda de sms_sts.ReadCurrent() (resolucion de 6.5 mA por
 *     paso, por lo que saltaba entre valores como 0 y 6.5 mA con bastante
 *     ruido). Ahora promedia NUM_MUESTRAS_LECTURA_CORRIENTE (10) lecturas
 *     crudas consecutivas antes de convertir a mA, lo que da una resolucion
 *     efectiva mas fina y suaviza el ruido instantaneo - relevante ahora
 *     que el margen es mucho mas chico y por lo tanto mas sensible a ese
 *     ruido.
 *
 *   - Nuevo periodo de asentamiento (DURACION_ASENTAMIENTO_ARRANQUE_MS,
 *     1500 ms) al iniciar rotarVueltas() y cada vez que se reanuda con
 *     COMANDO_CONTINUAR tras una pausa por sobrecorriente: mientras dura,
 *     monitorearCorriente() compara contra el limite absoluto
 *     (CORRIENTE_ABSOLUTA_MAX_MA) en vez del umbral adaptativo. Motivo: con
 *     el margen tan sensible, el pico normal de corriente al arrancar o
 *     reanudar desde velocidad 0 (inercia + friccion estatica) podia
 *     confundirse con una sobrecarga real. Fuera de ese periodo, se sigue
 *     usando el umbral adaptativo sensible normal.
 *
 *   - monitorearCorriente() se simplifico a un solo parametro
 *     (usarSoloLimiteAbsoluto) que decide CUAL umbral usar (adaptativo vs.
 *     limite absoluto), en vez del ignorarUmbral de v16 que saltaba
 *     cualquier chequeo por completo. Efecto en secuenciaRegreso() (regreso
 *     a home): sigue sin dispararse por el esfuerzo normal de empujar
 *     contra el Fin de carrera 1 (igual que en v16), pero ahora ADEMAS
 *     queda protegida por el limite absoluto ante una falla real
 *     (>CORRIENTE_ABSOLUTA_MAX_MA) - en v16 esa proteccion quedaba
 *     completamente desactivada durante el regreso; en v17 nunca se
 *     desactiva del todo.
 *
 *   - Tabla de calibracion de corriente ampliada de 5 a 10 puntos
 *     (NUM_PUNTOS_CALIBRACION_CORRIENTE), y con las velocidades de prueba
 *     (VELOCIDADES_CALIBRACION_CORRIENTE[]) concentradas en el extremo BAJO
 *     del rango en vez de igualmente espaciadas entre 20% y 100%: ahora son
 *     2%, 5%, 8%, 12%, 17%, 25%, 35%, 50%, 70% y 100% de SPEED_MAX. Motivo:
 *     en el uso real casi siempre se opera a velocidades bajas/medias - las
 *     velocidades altas (mayor a 35%) se usan poco - asi que conviene mas
 *     resolucion de calibracion justo donde el umbral adaptativo mas importa
 *     (velocidades bajas, donde ademas el margen del 12% deja menos margen
 *     absoluto en mA que a velocidades altas). Como calibrarCorrientePor
 *     Velocidad() recorre un punto por vez, con estos 10 puntos la
 *     calibracion completa tarda mas (unos 45 s en vez de ~22 s, ver
 *     DURACION_ASENTAMIENTO_CALIBRACION_MS y DURACION_MEDICION_CALIBRACION_MS).
 *     IMPORTANTE: la velocidad mas baja (2% de SPEED_MAX) es muy chica;
 *     verifica durante la calibracion que el eje SI se vea girar en ese
 *     punto (si tu mecanismo tiene mucha friccion estatica podria no
 *     arrancar a esa velocidad) - si no gira, sube el primer porcentaje.
 *
 *   - CORREGIDO: al concentrar puntos de calibracion en velocidades bajas
 *     (punto anterior), aparecio un problema de fondo en el margen: a esas
 *     velocidades la corriente normal calibrada puede ser de solo unos
 *     pocos mA, y un margen puramente PORCENTUAL sobre un numero tan chico
 *     da un umbral casi identico a ese numero (ej.: normal=2 mA, margen
 *     +40% -> umbral=2.8 mA =~ 3 mA), que la corriente real en operacion
 *     normal (variacion tipica de 5-6 mA a esas velocidades) supera sin que
 *     haya ninguna sobrecarga - se disparaba en falso en operacion normal.
 *     Nuevo MARGEN_CORRIENTE_MINIMO_MA (15 mA): corrienteUmbralParaVelocidad()
 *     ahora usa el MAYOR entre el margen porcentual y este piso aditivo fijo,
 *     asi el umbral nunca queda pegado a la propia corriente normal cuando
 *     esta es chica. A velocidades altas (corriente normal grande) el
 *     margen porcentual sigue siendo el que manda, sin cambios ahi.
 *
 * Diferencia con v15:
 *   - Durante secuenciaRegreso() (el regreso a home, comando COMANDO_REGRESO
 *     'r'/'R') ya NO se detiene el motor por sobrecorriente. Motivo: al
 *     buscar el Fin de carrera 1 el eje puede necesitar un empuje/esfuerzo
 *     puntual mayor al normal (fin de recorrido, friccion del tope mecanico),
 *     y no tiene sentido pausar ahi con las mismas opciones de "continuar/ir
 *     a home" que tiene rotarVueltas() (secuenciaRegreso() YA ES el regreso a
 *     home). monitorearCorriente() ahora recibe un parametro ignorarUmbral:
 *     si es true, sigue leyendo e imprimiendo la corriente (para no perder
 *     visibilidad en el log) pero nunca detiene el motor ni cambia de pagina
 *     en la Nextion, sin importar cuanto supere el umbral. secuenciaRegreso()
 *     llama monitorearCorriente(true) en sus dos tramos (reversa y ajuste
 *     final); todos los demas llamados (loop(), rotarVueltas()) siguen
 *     pasando monitorearCorriente(false), sin cambios de comportamiento.
 *     El limite absoluto CORRIENTE_ABSOLUTA_MAX_MA de calibrarCorrientePor
 *     Velocidad() no se toca (esa funcion no pasa por monitorearCorriente()).
 *
 * Diferencia con v14:
 *   - La tabla de calibracion de corriente (corrienteNormalMA[] y
 *     calibracionCorrienteValida) vivia solo en RAM: se perdia cada vez que
 *     el ESP32 se reiniciaba o se quedaba sin alimentacion, obligando a
 *     recalibrar (comando 'i') en cada uso del equipo. Ahora se guarda en la
 *     memoria NVS (flash no volatil) del ESP32 con la libreria Preferences.h
 *     (incluida en el core de Arduino para ESP32, no requiere instalar nada
 *     adicional), asi que la calibracion sobrevive apagados/reinicios y solo
 *     hace falta hacerla UNA vez.
 *
 *     guardarCalibracionCorriente(): escribe corrienteNormalMA[] (los
 *     NUM_PUNTOS_CALIBRACION_CORRIENTE valores medidos) y el flag de validez
 *     en el namespace NVS "rotarv". Se llama automaticamente al final de
 *     calibrarCorrientePorVelocidad(), justo despues de marcar la
 *     calibracion como valida - no hace falta ningun paso manual adicional.
 *
 *     cargarCalibracionCorriente(): lee esos mismos datos de NVS y, si
 *     existen (se guardo una calibracion valida en algun momento anterior),
 *     restaura corrienteNormalMA[] y pone calibracionCorrienteValida = true.
 *     Se llama automaticamente al inicio de setup(), antes de verificar el
 *     servo, e imprime por Serial si encontro o no una calibracion guardada.
 *     Si nunca se calibro, arranca igual que antes: calibracionCorriente
 *     Valida en false y corrienteUmbralParaVelocidad() usando
 *     CORRIENTE_ABSOLUTA_MAX_MA hasta que se calibre.
 *
 *     El comando 'i'/'I' (COMANDO_CALIBRAR_CORRIENTE) sigue disponible por
 *     si en algun momento se quiere recalibrar (ej: cambio de manguera, de
 *     liquido, de bomba) - simplemente vuelve a sobreescribir lo guardado en
 *     NVS.
 *
 * Diferencia con v13:
 *   - Umbral de sobrecorriente ADAPTATIVO segun la velocidad, en vez del
 *     CORRIENTE_MAX_MA fijo (150 mA) de v13. Motivo: la corriente normal del
 *     motor NO es constante - crece con la velocidad (mas RPM = mas
 *     corriente, para la misma carga), asi que un umbral fijo o dispara en
 *     falso a velocidades altas (donde la corriente normal ya se acerca al
 *     umbral) o no protege lo suficiente a velocidades bajas (donde el
 *     umbral queda demasiado holgado respecto de la corriente normal real).
 *
 *     Nueva funcion calibrarCorrientePorVelocidad(), disparada con el nuevo
 *     comando COMANDO_CALIBRAR_CORRIENTE ('i'/'I') por Serial/Nextion:
 *       1) Recorre NUM_PUNTOS_CALIBRACION_CORRIENTE velocidades de prueba
 *          (VELOCIDADES_CALIBRACION_CORRIENTE[], igualmente espaciadas entre
 *          20% y 100% de SPEED_MAX), con el sistema YA MONTADO (misma carga
 *          real que en "vueltas tiempo").
 *       2) En cada velocidad, descarta el transitorio de arranque/cambio de
 *          velocidad (DURACION_ASENTAMIENTO_CALIBRACION_MS) y luego promedia
 *          la corriente durante DURACION_MEDICION_CALIBRACION_MS, ya en
 *          estado estable.
 *       3) Guarda ese promedio como la "corriente normal" para esa
 *          velocidad, en corrienteNormalMA[].
 *     Solo se calibra en UN sentido de giro (positivo): se asume que la
 *     corriente normal es practicamente igual en el sentido contrario, para
 *     la misma carga mecanica (no depende de la direccion, solo de cuanto
 *     esfuerzo hace el motor).
 *
 *     En el codigo principal, monitorearCorriente() ya NO compara contra un
 *     numero fijo: llama a corrienteUmbralParaVelocidad(velocidadActual), que
 *     interpola linealmente entre los dos puntos calibrados mas cercanos a
 *     la velocidad actual y le suma un margen de MARGEN_CORRIENTE_PORCENTAJE
 *     (10% por defecto, ej: si a esa velocidad se midieron 120 mA "normales",
 *     el umbral de disparo queda en 132 mA). Fuera del rango calibrado se
 *     usa el punto extremo mas cercano (sin extrapolar mas alla).
 *
 *     Respaldo de seguridad: CORRIENTE_ABSOLUTA_MAX_MA es un limite DURO que
 *     nunca se supera, pase lo que pase - se usa como umbral por defecto
 *     mientras no se haya calibrado (calibracionCorrienteValida = false) y
 *     como techo del umbral adaptativo una vez calibrado (por si el ajuste
 *     calculado saliera mal). La propia calibracion tambien lo respeta: si
 *     la corriente supera ese limite durante la prueba, se cancela de
 *     inmediato sin guardar la tabla.
 *
 *     Como cualquier otro movimiento, calibrarCorrientePorVelocidad() respeta
 *     emergencia/fines de carrera (procesarInterrupciones()) durante la
 *     prueba, y el eje queda desplazado al terminar (recomienda enviar
 *     COMANDO_REGRESO despues).
 *
 * Diferencia con v12 (CORRECCION DE SEGURIDAD):
 *   - El controlador de la curva en S de v12 (controlarVelocidadObjetivo())
 *     hacia que el motor oscilara girando en AMBOS sentidos. Causa: el
 *     termino de correccion (error de seguimiento / CONSTANTE_TIEMPO_
 *     CORRECCION_S) no tenia ningun limite de signo. Si el servo se
 *     adelantaba lo suficiente a la curva de referencia (por ejemplo, si
 *     la velocidad real sigue sin coincidir exactamente con lo que asume
 *     RPM_POR_UNIDAD), el error de seguimiento se volvia negativo y grande,
 *     y la correccion terminaba pidiendo velocidad en sentido CONTRARIO al
 *     de "n" - es decir, ordenaba invertir el giro para "retroceder" el
 *     adelanto. Esto es inaceptable para una bomba de infusion: el motor
 *     NUNCA debe girar en reversa mientras una rotacion esta en curso (el
 *     fluido ya se movio, no se puede "deshacer" ese movimiento, y muchos
 *     mecanismos de bomba no estan pensados para revertir bajo presion).
 *
 *     Correccion: controlarVelocidadObjetivo() ahora aplica una restriccion
 *     de seguridad ANTES de comandar cualquier velocidad: si el resultado
 *     (feedforward + correccion) tiene signo contrario al de "n", se satura
 *     a 0 en vez de invertir - el motor se pausa (velocidad 0) hasta que la
 *     curva de referencia "lo alcance" (es decir, hasta que vuelva a hacer
 *     falta girar en el sentido correcto), pero NUNCA gira hacia atras.
 *     Se avisa UNA vez por giro con "[CONTROL] ... nunca gira en reversa"
 *     cuando esto ocurre, para que quede registrado en el log.
 *
 *     Se corrigio el mismo riesgo en el "empuje minimo" que forzaba
 *     velocidad +-1 cuando el calculo redondeaba a 0: antes se basaba en el
 *     signo de "vueltas restantes hasta la meta final", que puede volverse
 *     negativo si el motor ya se paso de la meta (sobre-giro), lo cual
 *     tambien hubiera forzado una velocidad en reversa. Ahora ese empuje
 *     minimo SOLO se aplica en el sentido de "n" (nunca en el contrario),
 *     y no se aplica en absoluto si ya se disparo la saturacion de reversa
 *     de arriba (para no pisar esa decision con un valor +-1).
 *
 * Diferencia con v10:
 *   - Se abandona el intento de "atacar" el problema calibrando RPM_POR_UNIDAD
 *     en tiempo real bajo carga (eso se probo en v11 y no se sigue por ese
 *     camino). En su lugar, se rediseno controlarVelocidadObjetivo() (la
 *     funcion que ajusta la velocidad dentro de rotarVueltas(), introducida
 *     en v9) con una CURVA DE REFERENCIA SUAVE en vez del calculo de
 *     "promedio para lo que resta" que traia desde v9.
 *
 *     Motivo: con el controlador de v9/v10, la velocidad comandada en
 *     t=0 YA ES el promedio nominal completo (vueltas_restantes/tiempo_
 *     restante = n/tTotal en el primer instante) - no hay ningun arranque
 *     gradual. Si la velocidad real que alcanza el servo para ese comando
 *     no coincide exactamente con lo que asume RPM_POR_UNIDAD (friccion,
 *     carga, no linealidad - lo normal en un sistema real), el motor puede
 *     adelantarse mucho antes de que el controlador tenga oportunidad de
 *     notarlo y corregir, dejando un patron de "arranca rapido, despues
 *     frena/corrige el resto del tiempo" (lo que se observo: menos de un
 *     20% restante habiendo transcurrido apenas el 50% del tiempo).
 *
 *     Nueva logica (ver controlarVelocidadObjetivo()): en vez de una
 *     trayectoria de referencia LINEAL (x_ref(t) = n * t/tTotal, que exige
 *     velocidad maxima desde el primer instante), se usa una curva en S
 *     tipo "smoothstep" (x_ref(t) = n * s(u), con u = t/tTotal y
 *     s(u) = 3u^2 - 2u^3), cuya PENDIENTE (la velocidad feedforward) es CERO
 *     al inicio y al final, y maxima a la mitad del tiempo. Esto acota la
 *     velocidad inicial sin importar que tan desviado este RPM_POR_UNIDAD:
 *     el motor arranca suave siempre, por diseno de la curva, no porque se
 *     haya medido bien el factor de conversion.
 *
 *     Ademas del termino feedforward (la pendiente de la curva en ese
 *     instante), se suma un termino de correccion PROPORCIONAL AL ERROR de
 *     seguimiento (diferencia entre donde "deberia" ir la curva ahora mismo
 *     y las vueltas realmente contadas), dividido entre una CONSTANTE DE
 *     TIEMPO FIJA (CONSTANTE_TIEMPO_CORRECCION_S, no "tiempo restante hasta
 *     el final" como en v9/v10). Esto corrige continuamente cualquier
 *     desvio de RPM_POR_UNIDAD a lo largo de TODO el giro (no solo al
 *     final), y evita el problema de v9/v10 donde la ganancia de correccion
 *     crecia sin limite cerca del final del tiempo (de ahi que ya no haga
 *     falta un caso especial para "no recalcular en el tramo final").
 *
 *     En resumen, el perfil de velocidad ahora es una campana suave (sube
 *     gradual, pasa por un maximo hacia la mitad del tiempo, baja gradual al
 *     final) en vez de "velocidad nominal fija desde el instante 0,
 *     corrigiendo solo cuando el error ya es grande". debugVueltas() ahora
 *     muestra el objetivo de la curva en S (no el lineal de v9/v10) para
 *     que se pueda comparar en vivo.
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
 *   - GPIO 14 : Sensor IR HX-M121 burbuja  (RISING, desde v19 - ver
 *                                           "Diferencia con v18")
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

#include "D:/DOCUMENTOS MICHAEL PERALTA/BOMBA/motor2/FTServo_Arduino/examples/SMS_STS/RotarVueltas_ESP32_v19_Nextion/SCServo.h"
#include <Preferences.h>

// --- Pines Serial2 para el servo ---
#define SERVO_RX_PIN  16
#define SERVO_TX_PIN  17

// --- Pines de interrupciones ---
#define PIN_EMERGENCIA    4
#define PIN_FIN_CARRERA1  5
#define PIN_FIN_CARRERA2  21
#define PIN_IR_BURBUJA    14

// --- Emisor (TX) del sensor IR de burbuja HX-M121, GPIO26. Se alimenta
//     SOLO durante el giro hacia adelante en modo normal (rotarVueltas()
//     con n > 0): en calibracion (calibrarCorrientePorVelocidad()), regreso
//     a home (secuenciaRegreso()), giro en reversa (n < 0) y reposo
//     permanece apagado. La interrupcion del receptor (PIN_IR_BURBUJA)
//     sigue la misma regla: solo esta activa (attachInterrupt) mientras el
//     emisor esta encendido - ver activarSensorIR()/desactivarSensorIR(). ---
#define PIN_IR_TX 26

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

// --- Nextion: pagina estado regreso ---
#define PAGINA_HOME 7

// --- Nextion: duracion en pantalla de la imagen de alerta de burbuja ---
#define ALERTA_DURACION_MS 3000

SMS_STS sms_sts;

// --- Almacenamiento persistente (NVS/flash) de la calibracion de corriente,
//     para no tener que recalibrar en cada arranque - ver guardarCalibracion
//     Corriente()/cargarCalibracionCorriente() y "Diferencia con v14". ---
Preferences prefs;
#define NVS_NAMESPACE_CALIBRACION "rotarv"

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

// --- Antirebote: tiempo minimo entre dos disparos del mismo pin. Se usa para
//     emergencia y ambos fines de carrera (necesitan reaccionar rapido). ---
#define DEBOUNCE_MS 200

// --- Antirebote DEDICADO para el sensor IR de burbuja: fisicamente no puede
//     pasar mas de una burbuja en pocos segundos, asi que se usa una ventana
//     mucho mas larga que DEBOUNCE_MS para que un mismo evento (incluido el
//     rebote electrico del fototransistor) se cuente una sola vez. Rango
//     util: 5000-10000 ms; ajustar segun que tan seguido pasan burbujas
//     reales por la manguera. ---
#define DEBOUNCE_IR_BURBUJA_MS 7000

// --- Antirebote de fines de carrera: tiempo minimo desactivada antes de
//     poder reactivarse (ademas de requerir que el pin ya no este presionado) ---
#define REARME_FIN_CARRERA_MS 1000

// --- Monitoreo periodico de corriente del motor ---
#define INTERVALO_MONITOREO_CORRIENTE_MS 1000

// --- Sobrecorriente adaptativa: si la magnitud de la corriente leida supera
//     el umbral (ver corrienteUmbralParaVelocidad()), se detiene el motor y
//     se manda la Nextion a PAGINA_EMERGENCIA, igual que con el boton de
//     emergencia. ---

// Cantidad de velocidades de prueba usadas por calibrarCorrientePorVelocidad().
#define NUM_PUNTOS_CALIBRACION_CORRIENTE 10

// Techo de las velocidades de prueba de calibracion (bien por debajo de
// SPEED_MAX = 4095): en el uso real casi no se pasa de aqui, asi que no
// tiene sentido gastar puntos de calibracion en velocidades mayores. Ver
// VELOCIDADES_CALIBRACION_CORRIENTE[] mas abajo, ahora concentradas en el
// extremo BAJO de este rango (que es donde peor andaba el umbral).
#define VELOCIDAD_MAX_CALIBRACION_CORRIENTE 2000

// Tiempo a descartar tras cambiar de velocidad durante la calibracion
// (transitorio de arranque/cambio), antes de empezar a promediar.
#define DURACION_ASENTAMIENTO_CALIBRACION_MS 1500

// Duracion de la ventana de promediado en estado estable, por velocidad.
#define DURACION_MEDICION_CALIBRACION_MS 3000

// Margen sobre la corriente normal medida en cada velocidad, EXPRESADO EN
// PORCENTAJE (12.0 = +12%, es decir umbral = normal * 1.12) para fijar el
// umbral de disparo real. Muy sensible a proposito: el objetivo es que un
// leve empujon con el dedo sobre el eje ya sea suficiente para dispararlo.
// Si en la practica dispara solo, sube este valor; si no dispara con un
// toque leve, bajalo. NUNCA pongas aqui un numero que haga que el umbral
// quede POR DEBAJO de la corriente normal (eso dispararia todo el tiempo,
// incluso sin ninguna carga extra) - por eso el umbral siempre se calcula
// como normal * (1 + margen/100), nunca con un factor menor a 1.
// YA NO ES UN VALOR FIJO DE COMPILACION: este numero es solo el valor por
// defecto la PRIMERA vez que se usa el equipo (o si se borra la memoria).
// A partir de ahi se ajusta en caliente por Serial/Nextion con el comando
// COMANDO_MARGEN_CORRIENTE ('T'/'t' + numero, ej. "T69" = 69%), sin volver a
// compilar/cargar el codigo - ver margenCorrientePorcentaje mas abajo y
// procesarComandoMargenCorriente().
#define MARGEN_CORRIENTE_PORCENTAJE_DEFECTO 40.0f

// Limites aceptados para el margen porcentual ajustable por Serial (ver
// COMANDO_MARGEN_CORRIENTE): cualquier valor fuera de este rango se rechaza
// y se conserva el margen anterior.
#define MARGEN_CORRIENTE_PORCENTAJE_MIN 0.0f
#define MARGEN_CORRIENTE_PORCENTAJE_MAX 300.0f

// Comando para ajustar el margen porcentual de sobrecorriente por Serial/
// Nextion sin recompilar: se envia la letra seguida del numero, sin espacio,
// ej. "T69" deja el margen en 69%. Acepta 'T' o 't'.
#define COMANDO_MARGEN_CORRIENTE 'T'

// Margen ADITIVO minimo, en mA, que se suma sobre la corriente normal antes
// de comparar con el margen porcentual - se usa el que de un umbral MAS
// ALTO de los dos (ver corrienteUmbralParaVelocidad()). Es indispensable a
// velocidades bajas: ahi la corriente normal calibrada puede ser de solo
// unos pocos mA (cerca del piso de ruido/cuantizacion del sensor, 1 LSB =
// 6.5 mA), y un margen puramente PORCENTUAL sobre un numero tan chico da un
// umbral casi igual a ese mismo numero chico (ej.: normal=2 mA, margen 40% ->
// umbral=2.8 mA), que cualquier lectura normal supera por simple variacion
// (no por sobrecarga real) - exactamente el sintoma de "umbral en 3 mA con
// corriente real de 5-6 mA" a velocidades bajas. Con este piso, el umbral
// nunca baja de normal + MARGEN_CORRIENTE_MINIMO_MA, sin importar que tan
// chica sea la corriente normal calibrada. Ajustalo segun lo que veas en el
// log de [MONITOREO] durante operacion normal (debe quedar comodamente por
// encima de la corriente normal observada, pero seguir reaccionando ante un
// empujon real). ---
#define MARGEN_CORRIENTE_MINIMO_MA 15.0f

// Limite absoluto de seguridad, independiente de la calibracion adaptativa:
// respaldo por si no se ha calibrado aun o el umbral calculado saliera mal.
// NUNCA se debe superar este valor, sin importar la velocidad.
#define CORRIENTE_ABSOLUTA_MAX_MA 400.0f

// --- Lectura de corriente: sms_sts.ReadCurrent() entrega un entero crudo en
//     pasos de 6.5 mA (1 LSB = 6.5 mA), asi que una sola lectura salta entre
//     valores como 0 y 6.5 mA sin resolucion intermedia y con bastante ruido
//     de un instante a otro. leerCorrienteMA() ahora promedia varias
//     lecturas crudas consecutivas ANTES de convertir a mA, lo que reduce
//     tanto el escalon de cuantizacion (el promedio de enteros puede caer
//     entre 0 y 6.5) como el ruido instantaneo. ---
#define NUM_MUESTRAS_LECTURA_CORRIENTE 10
#define INTERVALO_ENTRE_MUESTRAS_LECTURA_MS 2

// --- Periodo de asentamiento (gracia) al iniciar o reanudar rotarVueltas():
//     durante este tiempo, monitorearCorriente() compara contra el limite
//     absoluto (CORRIENTE_ABSOLUTA_MAX_MA) en vez del umbral adaptativo
//     (mucho mas sensible ahora, ver MARGEN_CORRIENTE_PORCENTAJE), para no
//     confundir el pico normal de arranque/aceleracion (inercia + friccion
//     estatica al partir desde velocidad 0) con una sobrecarga real. Mismo
//     valor que el asentamiento de la calibracion, por consistencia. ---
#define DURACION_ASENTAMIENTO_ARRANQUE_MS 1500

// Comando para disparar calibrarCorrientePorVelocidad() por Serial/Nextion.
#define COMANDO_CALIBRAR_CORRIENTE 'i'   // se acepta 'i' o 'I' (de "intensidad")

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
//     practicamente cero. ---
#define DEADBAND_VUELTAS_RESTANTES 0.02f

// --- Controlador de vueltas+tiempo: constante de tiempo (fija, en segundos)
//     del termino de correccion por error de seguimiento contra la curva en
//     S de referencia. Mas chica = corrige el error mas rapido pero es mas
//     sensible al ruido de lectura de posicion; mas grande = correccion mas
//     suave pero tarda mas en compensar un desvio de RPM_POR_UNIDAD. No
//     depende del tiempo restante hasta el final (a diferencia de v9/v10),
//     por eso no crece sin limite cerca del final del giro. ---
#define CONSTANTE_TIEMPO_CORRECCION_S 4.0f

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

// true mientras el emisor IR (PIN_IR_TX) y la interrupcion de su receptor
// (PIN_IR_BURBUJA) estan activos - ver activarSensorIR()/desactivarSensorIR().
bool sensorIRActivo = false;

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
bool          advertenciaReversaControlEnviada    = false;

// --- Periodo de asentamiento de corriente (ver DURACION_ASENTAMIENTO_
//     ARRANQUE_MS): marca de tiempo de cuando arranco o se reanudo el giro
//     actual de rotarVueltas(). Mientras no pase ese tiempo, monitorearCorriente()
//     usa el limite absoluto en vez del umbral adaptativo sensible. ---
unsigned long tInicioAsentamientoCorriente = 0;

// --- Sobrecorriente adaptativa: velocidades de prueba y tabla de corriente
//     normal medida por calibrarCorrientePorVelocidad() para cada una.
//     El rango completo de prueba va de 1% a 100% de VELOCIDAD_MAX_
//     CALIBRACION_CORRIENTE (2000, NO de SPEED_MAX: no hace falta calibrar
//     mas alla porque casi no se usa), y los 10 puntos estan MUY
//     concentrados en el extremo bajo (progresion aprox. geometrica: 1%,
//     2.5%, 4.5%, 7%, 11%, 17.5%, 27.5%, 42.5%, 65%, 100% de ese techo) -
//     los primeros 5 puntos caen entre velocidad 20 y 220 (0.5%-5.4% de
//     SPEED_MAX), justo donde el umbral andaba mal (corriente normal muy
//     chica, cerca del piso de ruido del sensor). Por encima de
//     VELOCIDAD_MAX_CALIBRACION_CORRIENTE, corrienteUmbralParaVelocidad() no
//     extrapola: usa el umbral del ultimo punto calibrado (2000) tal cual.
//     corrienteNormalMA[] queda en 0 y calibracionCorrienteValida en false
//     hasta que se corra la calibracion; mientras tanto
//     corrienteUmbralParaVelocidad() usa CORRIENTE_ABSOLUTA_MAX_MA como
//     umbral (conservador). ---
const int16_t VELOCIDADES_CALIBRACION_CORRIENTE[NUM_PUNTOS_CALIBRACION_CORRIENTE] = {
  (int16_t)(VELOCIDAD_MAX_CALIBRACION_CORRIENTE * 0.01f),
  (int16_t)(VELOCIDAD_MAX_CALIBRACION_CORRIENTE * 0.025f),
  (int16_t)(VELOCIDAD_MAX_CALIBRACION_CORRIENTE * 0.045f),
  (int16_t)(VELOCIDAD_MAX_CALIBRACION_CORRIENTE * 0.07f),
  (int16_t)(VELOCIDAD_MAX_CALIBRACION_CORRIENTE * 0.11f),
  (int16_t)(VELOCIDAD_MAX_CALIBRACION_CORRIENTE * 0.175f),
  (int16_t)(VELOCIDAD_MAX_CALIBRACION_CORRIENTE * 0.275f),
  (int16_t)(VELOCIDAD_MAX_CALIBRACION_CORRIENTE * 0.425f),
  (int16_t)(VELOCIDAD_MAX_CALIBRACION_CORRIENTE * 0.65f),
  VELOCIDAD_MAX_CALIBRACION_CORRIENTE
};
float corrienteNormalMA[NUM_PUNTOS_CALIBRACION_CORRIENTE] = {
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};
bool  calibracionCorrienteValida = false;

// Margen porcentual de sobrecorriente VIGENTE (ver MARGEN_CORRIENTE_
// PORCENTAJE_DEFECTO): ajustable en caliente por Serial/Nextion con
// COMANDO_MARGEN_CORRIENTE ('T'/'t' + numero), y persistido en NVS por
// guardarMargenCorriente() para sobrevivir apagados/reinicios igual que la
// tabla de calibracion. cargarMargenCorriente() lo restaura en setup().
float margenCorrientePorcentaje = MARGEN_CORRIENTE_PORCENTAJE_DEFECTO;

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
// LOW a HIGH, flanco de subida), es decir, cuando una burbuja de aire cruza
// la manguera. Usa su propio antirebote (DEBOUNCE_IR_BURBUJA_MS, 5-10 s) en
// vez del DEBOUNCE_MS general, para registrar una sola deteccion por burbuja.
void IRAM_ATTR isrIRBurbuja() {
  unsigned long ahora = millis();
  if (ahora - tUltimaIRBurbuja < DEBOUNCE_IR_BURBUJA_MS) return;
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
// activarSensorIR / desactivarSensorIR: encienden/apagan el emisor (TX,
// PIN_IR_TX) del sensor IR de burbuja y activan/desactivan la interrupcion
// de su receptor (PIN_IR_BURBUJA). Deben usarse SIEMPRE en conjunto para que
// el sensor solo pueda reportar mientras esta encendido. Llamadas solo desde
// rotarVueltas(), unicamente cuando gira hacia adelante (n > 0, modo
// funcionamiento normal) - ver comentario junto a PIN_IR_TX. Idempotentes:
// llamarlas de nuevo con el sensor ya en ese estado no hace nada.
// -----------------------------------------------------------------------
void activarSensorIR() {
  if (sensorIRActivo) return;
  digitalWrite(PIN_IR_TX, HIGH);
  flagIRBurbuja = false;
  attachInterrupt(digitalPinToInterrupt(PIN_IR_BURBUJA), isrIRBurbuja, RISING);
  sensorIRActivo = true;
}

void desactivarSensorIR() {
  if (!sensorIRActivo) return;
  detachInterrupt(digitalPinToInterrupt(PIN_IR_BURBUJA));
  digitalWrite(PIN_IR_TX, LOW);
  flagIRBurbuja  = false;
  sensorIRActivo = false;

  if (burbujaVisible) {
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
// leerCorrienteMA: lee la corriente del servo y la convierte a mA (magnitud).
// Promedia NUM_MUESTRAS_LECTURA_CORRIENTE lecturas CRUDAS consecutivas
// (separadas por INTERVALO_ENTRE_MUESTRAS_LECTURA_MS) antes de multiplicar
// por 6.5 mA/paso: como cada lectura cruda es un entero, promediar varias
// da una resolucion efectiva mas fina que 6.5 mA y suaviza el ruido
// instantaneo (ver definicion de NUM_MUESTRAS_LECTURA_CORRIENTE). Ignora
// las lecturas individuales fallidas (-1); retorna false solo si TODAS
// las muestras del bloque fallaron (sin tocar corrienteMAOut).
// -----------------------------------------------------------------------
bool leerCorrienteMA(float &corrienteMAOut) {
  long sumaRaw        = 0;
  int  muestrasValidas = 0;

  for (int i = 0; i < NUM_MUESTRAS_LECTURA_CORRIENTE; i++) {
    int corrienteRaw = sms_sts.ReadCurrent(SERVO_ID);
    if (corrienteRaw != -1) {
      sumaRaw += abs(corrienteRaw);
      muestrasValidas++;
    }
    if (i < NUM_MUESTRAS_LECTURA_CORRIENTE - 1) delay(INTERVALO_ENTRE_MUESTRAS_LECTURA_MS);
  }

  if (muestrasValidas == 0) return false;
  corrienteMAOut = ((float)sumaRaw / muestrasValidas) * 6.5f;
  return true;
}

// -----------------------------------------------------------------------
// corrienteUmbralParaVelocidad: calcula el umbral de sobrecorriente para la
// velocidad indicada, interpolando linealmente entre los dos puntos de
// VELOCIDADES_CALIBRACION_CORRIENTE / corrienteNormalMA[] mas cercanos a
// |velocidad|, y sumandole a esa corriente normal interpolada el MAYOR entre
// dos margenes: el porcentual (margenCorrientePorcentaje, ajustable en
// caliente por Serial/Nextion - ver COMANDO_MARGEN_CORRIENTE) y el aditivo
// fijo (MARGEN_CORRIENTE_MINIMO_MA). Esto evita que a velocidades bajas -donde la
// corriente normal calibrada puede ser de solo unos pocos mA- un margen
// puramente porcentual deje un umbral practicamente igual a la corriente
// normal (disparando con cualquier variacion normal, sin sobrecarga real).
// Fuera del rango calibrado, usa el punto extremo mas cercano (no
// extrapola). Si aun no se ha calibrado (calibracionCorrienteValida ==
// false), o el resultado supera CORRIENTE_ABSOLUTA_MAX_MA, se devuelve
// CORRIENTE_ABSOLUTA_MAX_MA (limite duro de respaldo).
// -----------------------------------------------------------------------
float corrienteUmbralParaVelocidad(int16_t velocidad) {
  if (!calibracionCorrienteValida) return CORRIENTE_ABSOLUTA_MAX_MA;

  float v = fabsf((float)velocidad);
  float corrienteNormalInterpolada;

  if (v <= VELOCIDADES_CALIBRACION_CORRIENTE[0]) {
    corrienteNormalInterpolada = corrienteNormalMA[0];
  } else if (v >= VELOCIDADES_CALIBRACION_CORRIENTE[NUM_PUNTOS_CALIBRACION_CORRIENTE - 1]) {
    corrienteNormalInterpolada = corrienteNormalMA[NUM_PUNTOS_CALIBRACION_CORRIENTE - 1];
  } else {
    corrienteNormalInterpolada = corrienteNormalMA[NUM_PUNTOS_CALIBRACION_CORRIENTE - 1];  // respaldo
    for (int i = 0; i < NUM_PUNTOS_CALIBRACION_CORRIENTE - 1; i++) {
      float v0 = VELOCIDADES_CALIBRACION_CORRIENTE[i];
      float v1 = VELOCIDADES_CALIBRACION_CORRIENTE[i + 1];
      if (v >= v0 && v <= v1) {
        float frac = (v1 > v0) ? (v - v0) / (v1 - v0) : 0.0f;
        corrienteNormalInterpolada = corrienteNormalMA[i] + frac * (corrienteNormalMA[i + 1] - corrienteNormalMA[i]);
        break;
      }
    }
  }

  float umbralPorcentual = corrienteNormalInterpolada * (1.0f + margenCorrientePorcentaje / 100.0f);
  float umbralConPiso    = corrienteNormalInterpolada + MARGEN_CORRIENTE_MINIMO_MA;
  float umbral           = (umbralConPiso > umbralPorcentual) ? umbralConPiso : umbralPorcentual;
  if (umbral > CORRIENTE_ABSOLUTA_MAX_MA) umbral = CORRIENTE_ABSOLUTA_MAX_MA;
  return umbral;
}

// -----------------------------------------------------------------------
// guardarCalibracionCorriente / cargarCalibracionCorriente: persisten
// corrienteNormalMA[] y calibracionCorrienteValida en la memoria NVS (flash
// no volatil) del ESP32, para no perder la calibracion al apagar/reiniciar
// y no tener que repetirla en cada uso del equipo (ver "Diferencia con v14").
// guardarCalibracionCorriente() se llama SOLO al terminar una calibracion
// exitosa (dentro de calibrarCorrientePorVelocidad()). cargarCalibracion
// Corriente() se llama UNA vez en setup(): si no hay nada guardado (primer
// uso del equipo, o memoria borrada), no cambia nada y el sistema arranca
// como siempre - calibracionCorrienteValida en false, usando
// CORRIENTE_ABSOLUTA_MAX_MA hasta calibrar con el comando 'i'.
// -----------------------------------------------------------------------
void guardarCalibracionCorriente() {
  prefs.begin(NVS_NAMESPACE_CALIBRACION, false);
  for (int i = 0; i < NUM_PUNTOS_CALIBRACION_CORRIENTE; i++) {
    prefs.putFloat(("curCal" + String(i)).c_str(), corrienteNormalMA[i]);
  }
  prefs.putBool("curValida", calibracionCorrienteValida);
  prefs.end();
}

void cargarCalibracionCorriente() {
  prefs.begin(NVS_NAMESPACE_CALIBRACION, true);  // solo lectura
  bool habiaCalibracion = prefs.getBool("curValida", false);
  if (habiaCalibracion) {
    for (int i = 0; i < NUM_PUNTOS_CALIBRACION_CORRIENTE; i++) {
      corrienteNormalMA[i] = prefs.getFloat(("curCal" + String(i)).c_str(), 0.0f);
    }
    calibracionCorrienteValida = true;
  }
  prefs.end();

  if (habiaCalibracion) {
    enviarDebug("[CALIBRACION-CORRIENTE] Calibracion guardada encontrada en memoria - restaurada, no hace "
                 "falta recalibrar.");
  } else {
    enviarDebug("[CALIBRACION-CORRIENTE] No hay calibracion guardada. Usando limite absoluto (" +
                 String(CORRIENTE_ABSOLUTA_MAX_MA, 0) + " mA) hasta calibrar con '" +
                 String(COMANDO_CALIBRAR_CORRIENTE) + "'.");
  }
}

// -----------------------------------------------------------------------
// guardarMargenCorriente / cargarMargenCorriente: persisten
// margenCorrientePorcentaje en la misma memoria NVS que la calibracion de
// corriente, para que el valor ajustado por Serial/Nextion (ver
// procesarComandoMargenCorriente()) sobreviva apagados/reinicios y no haya
// que reenviarlo cada vez que se energiza el equipo. guardarMargenCorriente()
// se llama solo cuando el comando cambia el valor con exito;
// cargarMargenCorriente() se llama UNA vez en setup(): si no hay nada
// guardado (primer uso, o memoria borrada), deja el valor por defecto
// MARGEN_CORRIENTE_PORCENTAJE_DEFECTO tal como esta.
// -----------------------------------------------------------------------
void guardarMargenCorriente() {
  prefs.begin(NVS_NAMESPACE_CALIBRACION, false);
  prefs.putFloat("margenPct", margenCorrientePorcentaje);
  prefs.end();
}

void cargarMargenCorriente() {
  prefs.begin(NVS_NAMESPACE_CALIBRACION, true);  // solo lectura
  margenCorrientePorcentaje = prefs.getFloat("margenPct", MARGEN_CORRIENTE_PORCENTAJE_DEFECTO);
  prefs.end();

  enviarDebug("[MARGEN] Margen de sobrecorriente: " + String(margenCorrientePorcentaje, 1) +
               "% (ajustable sin recompilar con '" + String(COMANDO_MARGEN_CORRIENTE) +
               "' + numero, ej. \"T69\" = 69%).");
}

// -----------------------------------------------------------------------
// procesarComandoMargenCorriente: atiende el comando COMANDO_MARGEN_CORRIENTE
// ('T'/'t' + numero, ej. "T69" = 69%) recibido por Serial/Nextion (mismo
// UART0), para ajustar margenCorrientePorcentaje SIN recompilar/recargar el
// codigo. Se llama desde loop() cuando el primer byte disponible es 'T'/'t'.
// Valida que el numero este dentro de MARGEN_CORRIENTE_PORCENTAJE_MIN/MAX;
// si es invalido o esta fuera de rango, se avisa y NO se toca el valor
// vigente. Si es valido, se aplica de inmediato (el proximo monitorearCorriente()
// ya lo usa) y se guarda en NVS con guardarMargenCorriente().
// -----------------------------------------------------------------------
void procesarComandoMargenCorriente() {
  Serial.read();  // consumir la letra del comando ('T'/'t')
  float nuevoMargen = Serial.parseFloat();
  while (Serial.available()) Serial.read();  // limpiar resto de la linea

  if (nuevoMargen < MARGEN_CORRIENTE_PORCENTAJE_MIN || nuevoMargen > MARGEN_CORRIENTE_PORCENTAJE_MAX) {
    enviarDebug("[MARGEN] Valor invalido. Usa '" + String(COMANDO_MARGEN_CORRIENTE) +
                 "' seguido del porcentaje, entre " + String(MARGEN_CORRIENTE_PORCENTAJE_MIN, 0) +
                 " y " + String(MARGEN_CORRIENTE_PORCENTAJE_MAX, 0) + " (ej. \"T69\" = 69%). "
                 "Margen sin cambios: " + String(margenCorrientePorcentaje, 1) + "%.");
    return;
  }

  margenCorrientePorcentaje = nuevoMargen;
  guardarMargenCorriente();
  enviarDebug("[MARGEN] Margen de sobrecorriente actualizado a " + String(margenCorrientePorcentaje, 1) +
               "% y guardado en memoria (no hace falta reenviarlo en el proximo arranque).");
}

// -----------------------------------------------------------------------
// dentroDeAsentamientoArranque: true durante los DURACION_ASENTAMIENTO_
// ARRANQUE_MS posteriores a marcar tInicioAsentamientoCorriente (llamado al
// iniciar rotarVueltas() y cada vez que se reanuda con COMANDO_CONTINUAR
// tras una pausa por sobrecorriente). Mientras es true, monitorearCorriente()
// usa el limite absoluto en vez del umbral adaptativo sensible, para no
// confundir el pico normal de arranque/aceleracion con sobrecarga real.
// -----------------------------------------------------------------------
bool dentroDeAsentamientoArranque() {
  return (millis() - tInicioAsentamientoCorriente) < DURACION_ASENTAMIENTO_ARRANQUE_MS;
}

// -----------------------------------------------------------------------
// monitorearCorriente: imprime por Serial la corriente del motor cada
// INTERVALO_MONITOREO_CORRIENTE_MS ms, y si su magnitud supera el umbral
// vigente, detiene el motor y manda la Nextion a PAGINA_EMERGENCIA (misma
// pantalla que usa el boton de emergencia). Retorna true solo en ese caso,
// para que rotarVueltas()/secuenciaRegreso() puedan cancelar el movimiento
// en curso igual que ya hacen con procesarInterrupciones().
// NOTA: a diferencia de la emergencia/fines de carrera (que son
// interrupciones de hardware), esto es una revision por software que solo
// se evalua una vez por INTERVALO_MONITOREO_CORRIENTE_MS, no al instante.
//
// usarSoloLimiteAbsoluto: el umbral vigente SIEMPRE se calcula con la
// velocidad ACTUAL (corrienteUmbralParaVelocidad(velocidadActual) -
// nunca queda "pegado" a una velocidad vieja). Este parametro solo decide
// CUAL umbral usar:
//   - false (uso normal: loop() en reposo, y rotarVueltas() fuera del
//     periodo de asentamiento): umbral adaptativo, sensible, el que da
//     corrienteUmbralParaVelocidad().
//   - true: en vez del adaptativo se usa el limite absoluto duro
//     (CORRIENTE_ABSOLUTA_MAX_MA). Se usa en dos casos, donde el motor
//     necesita un esfuerzo puntual mayor al normal sin que eso se confunda
//     con sobrecarga real: 1) secuenciaRegreso() (regreso a home: el eje
//     puede necesitar mas empuje al buscar el Fin de carrera 1), y
//     2) rotarVueltas() durante DURACION_ASENTAMIENTO_ARRANQUE_MS tras
//     iniciar o reanudar un giro (pico normal de arranque/aceleracion).
//     En ambos casos sigue habiendo proteccion real ante una falla dura
//     (>CORRIENTE_ABSOLUTA_MAX_MA), solo se relaja el umbral sensible.
// -----------------------------------------------------------------------
bool monitorearCorriente(bool usarSoloLimiteAbsoluto) {
  unsigned long ahora = millis();
  if (ahora - tUltimoMonitoreoCorriente < INTERVALO_MONITOREO_CORRIENTE_MS) return false;
  tUltimoMonitoreoCorriente = ahora;

  float corrienteMA;
  if (!leerCorrienteMA(corrienteMA)) {
    enviarDebug("[MONITOREO] Error leyendo corriente del motor.");
    return false;
  }

  float umbral = usarSoloLimiteAbsoluto ? CORRIENTE_ABSOLUTA_MAX_MA
                                         : corrienteUmbralParaVelocidad(velocidadActual);
  enviarDebug("[MONITOREO] Corriente del motor: " + String(corrienteMA) +
               " mA (umbral " + (usarSoloLimiteAbsoluto ? "absoluto: " : "adaptativo: ") +
               String(umbral, 1) + " mA)");

  if (corrienteMA > umbral) {
    velocidadActual = 0;
    sms_sts.WriteSpe(SERVO_ID, 0, 0);
    enviarDebug("[INTERRUPCION] Sobrecorriente detectada (" + String(corrienteMA, 1) +
                 " mA > " + String(umbral, 1) + " mA)! Motor detenido 5 segundos.");
    cambiarPaginaNextion(PAGINA_SOBRECORRIENTE);
    sonarAlerta();
    delay(1000);
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
// curvaS / derivadaCurvaS: forma de la trayectoria de referencia (smoothstep)
// usada por controlarVelocidadObjetivo() y debugVueltas(). u = t/tTotal,
// acotado a [0,1].
//   curvaS(u)      = 3u^2 - 2u^3   -> fraccion de "n" ya recorrida en la
//                                     referencia (0 en u=0, 1 en u=1).
//   derivadaCurvaS(u) = 6u(1-u)    -> pendiente de curvaS respecto de u
//                                     (0 en los extremos, maxima -1.5- a
//                                     la mitad; promedio exacto = 1 en
//                                     [0,1], por eso el area total sigue
//                                     dando "n" vueltas completas).
// -----------------------------------------------------------------------
float curvaS(float u) {
  return (3.0f - 2.0f * u) * u * u;
}

float derivadaCurvaS(float u) {
  return 6.0f * u * (1.0f - u);
}

// -----------------------------------------------------------------------
// controlarVelocidadObjetivo: recalcula periodicamente (cada
// INTERVALO_CONTROL_VELOCIDAD_MS) la velocidad comandada al servo dentro de
// rotarVueltas(), seudo-controlador feedforward + feedback que sigue una
// CURVA EN S (smoothstep) de referencia en vez de una linea recta:
//
//   u          = transcurrido / tTotal                      (0..1)
//   x_ref(t)   = n * curvaS(u)                               [vueltas]
//   ff_rpm(t)  = (n / (tTotal en minutos)) * derivadaCurvaS(u) [RPM]
//   error(t)   = x_ref(t) - vueltasActuales()                 [vueltas]
//   fb_rpm(t)  = (error(t) / CONSTANTE_TIEMPO_CORRECCION_S) * 60
//   rpm_total  = ff_rpm + fb_rpm
//   velocidad  = rpm_total / RPM_POR_UNIDAD
//
// ff_rpm es CERO al inicio y al final (derivadaCurvaS(0)=derivadaCurvaS(1)=0)
// y maxima a la mitad del tiempo: el arranque y la frenada son siempre
// graduales, sin importar que tan preciso sea RPM_POR_UNIDAD. fb_rpm corrige
// continuamente cualquier desvio entre lo planeado y lo realmente girado,
// con una ganancia FIJA (1/CONSTANTE_TIEMPO_CORRECCION_S) que no depende de
// cuanto tiempo quede, asi que no se dispara sin control cerca del final
// (a diferencia del "vueltas_restantes/tiempo_restante" de v9/v10).
// Si rpm_total supera lo que SPEED_MAX puede representar, se satura al
// maximo (se avisa UNA sola vez por giro). Cuando lo que falta para el
// objetivo final ya es menor a DEADBAND_VUELTAS_RESTANTES, no se fuerza una
// velocidad minima aunque el redondeo de rpm_total de cero (evita chatter).
//
// SEGURIDAD (bomba de infusion): el resultado NUNCA puede tener signo
// contrario al de "n". Si fb_rpm (la correccion) es tan negativa que
// invertiria el sentido de giro -tipicamente porque el motor se adelanto
// a la curva de referencia-, se satura a 0 (el motor se pausa) en vez de
// invertir. El motor jamas gira en reversa mientras rotarVueltas() esta en
// curso.
// -----------------------------------------------------------------------
void controlarVelocidadObjetivo(float n, unsigned long tStart, unsigned long tTotal) {
  unsigned long ahora = millis();
  if (ahora - tUltimoControlVelocidad < INTERVALO_CONTROL_VELOCIDAD_MS) return;
  tUltimoControlVelocidad = ahora;

  float u = (float)(ahora - tStart) / (float)tTotal;
  if (u < 0.0f) u = 0.0f;
  if (u > 1.0f) u = 1.0f;

  float tTotalMin = tTotal / 60000.0f;
  float ffRpm      = (n / tTotalMin) * derivadaCurvaS(u);

  float vueltasObjetivoAhora = n * curvaS(u);
  float errorSeguimiento     = vueltasObjetivoAhora - vueltasActuales();
  float fbRpm                = (errorSeguimiento / CONSTANTE_TIEMPO_CORRECCION_S) * 60.0f;

  float   rpmTotal   = ffRpm + fbRpm;
  float   crudo      = rpmTotal / RPM_POR_UNIDAD;
  int16_t speedNuevo = (int16_t)(crudo + (crudo >= 0.0f ? 0.5f : -0.5f));

  // SEGURIDAD: nunca invertir el sentido de giro solicitado (n), ni siquiera
  // para "corregir" un adelanto sobre la curva de referencia. Se satura a 0
  // (pausa) en vez de retroceder - ver nota de seguridad arriba.
  bool pediaReversa = (n > 0.0f && speedNuevo < 0) || (n < 0.0f && speedNuevo > 0);
  if (pediaReversa) {
    speedNuevo = 0;
    if (!advertenciaReversaControlEnviada) {
      enviarDebug("[CONTROL] El motor se adelanto a la curva de referencia. Velocidad pausada en 0: "
                   "esta bomba nunca gira en reversa durante una rotacion en curso.");
      advertenciaReversaControlEnviada = true;
    }
  } else if (speedNuevo > SPEED_MAX || speedNuevo < -SPEED_MAX) {
    speedNuevo = (speedNuevo > 0) ? SPEED_MAX : -SPEED_MAX;
    if (!advertenciaSaturacionControlEnviada) {
      enviarDebug("[CONTROL] Velocidad de correccion saturada al maximo (" + String(SPEED_MAX) +
                   "): la curva en S pide mas velocidad de la alcanzable en este punto.");
      advertenciaSaturacionControlEnviada = true;
    }
  }

  // Empuje minimo SOLO en el sentido de "n" (nunca en el contrario) cuando
  // aun falta llegar a la meta final y el redondeo dio exactamente 0; no se
  // aplica si ya se disparo la saturacion de reversa de arriba.
  if (speedNuevo == 0 && !pediaReversa) {
    float faltanteFinal = n - vueltasActuales();
    bool  faltaEnSentidoDeN = (n > 0.0f && faltanteFinal > DEADBAND_VUELTAS_RESTANTES) ||
                              (n < 0.0f && faltanteFinal < -DEADBAND_VUELTAS_RESTANTES);
    if (faltaEnSentidoDeN) speedNuevo = (n > 0.0f) ? 1 : -1;
  }

  if (speedNuevo != velocidadActual) {
    velocidadActual = speedNuevo;
    sms_sts.WriteSpe(SERVO_ID, speedNuevo, 0);
  }
}

// -----------------------------------------------------------------------
// debugVueltas: imprime periodicamente (mensaje de depuracion para la fase
// de calibracion) las vueltas acumuladas, cuantas "deberian" llevarse a esa
// altura del tiempo transcurrido segun la curva en S de referencia (ver
// controlarVelocidadObjetivo()) y la velocidad comandada en ese momento -
// util para ver en vivo si el controlador esta corrigiendo como se espera.
// -----------------------------------------------------------------------
void debugVueltas(unsigned long tInicio, float n, unsigned long tTotal) {
  unsigned long ahora = millis();
  if (ahora - tUltimoDebugVueltas < INTERVALO_DEBUG_VUELTAS_MS) return;
  tUltimoDebugVueltas = ahora;

  float transcurridoSeg = (ahora - tInicio) / 1000.0f;
  float u = (float)(ahora - tInicio) / (float)tTotal;
  if (u < 0.0f) u = 0.0f;
  if (u > 1.0f) u = 1.0f;
  float vueltasObjetivo = n * curvaS(u);

  enviarDebug("[CALIBRACION] Vueltas acumuladas: " + String(vueltasActuales(), 3) +
               " | Objetivo curva-S a este tiempo: " + String(vueltasObjetivo, 3) +
               " | Transcurrido: " + String(transcurridoSeg, 1) +
               " s | Velocidad: " + String(velocidadActual));
}

// -----------------------------------------------------------------------
// confirmarEventoReal: exige que "pin" se mantenga en "nivelActivo" durante
// todo CONFIRMACION_RUIDO_MS antes de aceptar el flag correspondiente como un
// evento real. Un pulso de ruido/EMI (ej. al tocar el eje metalico del
// motor) genera un cambio momentaneo que se recupera casi de inmediato; una
// pulsacion real de fin de carrera, la parada de emergencia o el sensor IR
// se mantiene. Distinto del antirebote (DEBOUNCE_MS/DEBOUNCE_IR_BURBUJA_MS)
// de las ISR, que solo evita repeticiones despues del primer disparo.
// nivelActivo: LOW para emergencia/fin de carrera 1/fin de carrera 2
// (disparan por FALLING, activos en LOW); HIGH para el sensor IR de burbuja
// (dispara por RISING desde v19, activo en HIGH - ver "Diferencia con v18").
// -----------------------------------------------------------------------
bool confirmarEventoReal(uint8_t pin, uint8_t nivelActivo) {
  unsigned long inicio = millis();
  while (millis() - inicio < CONFIRMACION_RUIDO_MS) {
    if (digitalRead(pin) != nivelActivo) return false;  // se recupero antes de tiempo: fue ruido
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
  if (huboIRBurbuja && confirmarEventoReal(PIN_IR_BURBUJA, HIGH)) {
    enviarDebug("[ALERTA] Burbuja detectada por sensor IR HX-M121. El motor continua girando.");
    mostrarImagenNextion(OBJ_BURBUJA, true);
    burbujaVisible   = true;
    burbujaOcultarEn = millis() + ALERTA_DURACION_MS;
  }

  bool huboParada = huboEmergencia || huboFinCarrera1 || huboFinCarrera2;
  if (!huboParada) return false;   // solo hubo burbuja (o ruido en burbuja): el motor no se toca

  sms_sts.WriteSpe(SERVO_ID, 0, 0);   // detener motor de inmediato, sin esperar la confirmacion

  bool emergenciaReal  = huboEmergencia  && confirmarEventoReal(PIN_EMERGENCIA, LOW);
  bool finCarrera1Real = huboFinCarrera1 && confirmarEventoReal(PIN_FIN_CARRERA1, LOW);
  bool finCarrera2Real = huboFinCarrera2 && confirmarEventoReal(PIN_FIN_CARRERA2, LOW);

  if (!emergenciaReal && !finCarrera1Real && !finCarrera2Real) {
    //enviarDebug("[FILTRO] Pulso descartado por ruido/EMI (pin ya no estaba activo). Motor reanudado.");
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

  delay(1000);
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

  // El regreso a home NUNCA es "giro hacia adelante en modo normal": el
  // sensor IR (emisor + interrupcion) debe quedar apagado durante toda la
  // secuencia, sin importar el estado en que haya quedado antes.
  desactivarSensorIR();

  //mover pagina1234@

  cambiarPaginaNextion(PAGINA_HOME);
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
    if (monitorearCorriente(true)) {
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
    if (monitorearCorriente(true)) {
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
  cambiarPaginaNextion(0);
}

// -----------------------------------------------------------------------
// calibrarCorrientePorVelocidad: mide la corriente NORMAL del motor en
// NUM_PUNTOS_CALIBRACION_CORRIENTE velocidades de prueba
// (VELOCIDADES_CALIBRACION_CORRIENTE[]), BAJO LA CARGA REAL del sistema ya
// montado, y llena corrienteNormalMA[] con esos promedios. Solo gira en un
// sentido (velocidades positivas): se asume que la corriente normal no
// depende de la direccion, solo de la carga mecanica.
// En cada velocidad: 1) descarta el transitorio de arranque/cambio de
// velocidad (DURACION_ASENTAMIENTO_CALIBRACION_MS), 2) promedia la lectura
// de corriente durante DURACION_MEDICION_CALIBRACION_MS ya en estado
// estable. Respeta emergencia/fines de carrera (procesarInterrupciones())
// igual que cualquier otro movimiento, y aborta de inmediato -sin guardar
// nada- si la corriente supera CORRIENTE_ABSOLUTA_MAX_MA en cualquier
// momento de la prueba (limite duro de seguridad).
// Si se completan los NUM_PUNTOS_CALIBRACION_CORRIENTE puntos sin abortar,
// se copia la tabla temporal a corrienteNormalMA[] y se marca
// calibracionCorrienteValida = true; monitorearCorriente() usara esta tabla
// (mas MARGEN_CORRIENTE_PORCENTAJE) desde ese momento.
// Al terminar, el eje queda desplazado por la prueba: se recomienda enviar
// COMANDO_REGRESO ('r') antes de la primera prueba real de "vueltas tiempo".
// -----------------------------------------------------------------------
void calibrarCorrientePorVelocidad() {
  // La calibracion gira hacia adelante, pero NO es el modo funcionamiento
  // normal: el sensor IR (emisor + interrupcion) debe quedar apagado durante
  // toda la prueba, sin importar el estado en que haya quedado antes.
  desactivarSensorIR();

  enviarDebug("[CALIBRACION-CORRIENTE] Iniciando: " + String(NUM_PUNTOS_CALIBRACION_CORRIENTE) +
               " velocidades de prueba, con la carga real conectada. El eje avanzara durante la prueba.");
  sonarInicioMovimiento();

  float tablaTemporal[NUM_PUNTOS_CALIBRACION_CORRIENTE];
  bool  cancelado = false;

  iniciarConteoVueltas();

  for (int i = 0; i < NUM_PUNTOS_CALIBRACION_CORRIENTE && !cancelado; i++) {
    int16_t velocidadPrueba = VELOCIDADES_CALIBRACION_CORRIENTE[i];
    velocidadActual = velocidadPrueba;
    sms_sts.WriteSpe(SERVO_ID, velocidadPrueba, 0);

    enviarDebug("[CALIBRACION-CORRIENTE] Punto " + String(i + 1) + "/" +
                 String(NUM_PUNTOS_CALIBRACION_CORRIENTE) + ": velocidad " + String(velocidadPrueba) +
                 ". Asentando...");

    // Descartar el transitorio de arranque/cambio de velocidad.
    unsigned long tAsentar = millis();
    while (millis() - tAsentar < DURACION_ASENTAMIENTO_CALIBRACION_MS) {
      if (procesarInterrupciones()) { cancelado = true; break; }
      revisarRearmeFinesDeCarrera();
      actualizarConteoVueltas();

      float corrienteMA;
      if (leerCorrienteMA(corrienteMA) && corrienteMA > CORRIENTE_ABSOLUTA_MAX_MA) {
        enviarDebug("[CALIBRACION-CORRIENTE] Corriente supero el limite absoluto de seguridad (" +
                     String(CORRIENTE_ABSOLUTA_MAX_MA, 0) + " mA) durante el asentamiento. Cancelada.");
        cancelado = true;
        break;
      }
      delay(10);
    }
    if (cancelado) break;

    // Promediar la corriente ya en estado estable.
    double        sumaCorriente = 0.0;
    long          muestras      = 0;
    unsigned long tMedir        = millis();
    while (millis() - tMedir < DURACION_MEDICION_CALIBRACION_MS) {
      if (procesarInterrupciones()) { cancelado = true; break; }
      revisarRearmeFinesDeCarrera();
      actualizarConteoVueltas();

      float corrienteMA;
      if (leerCorrienteMA(corrienteMA)) {
        if (corrienteMA > CORRIENTE_ABSOLUTA_MAX_MA) {
          enviarDebug("[CALIBRACION-CORRIENTE] Corriente supero el limite absoluto de seguridad (" +
                       String(CORRIENTE_ABSOLUTA_MAX_MA, 0) + " mA). Cancelada.");
          cancelado = true;
          break;
        }
        sumaCorriente += corrienteMA;
        muestras++;
      }
      delay(50);
    }
    if (cancelado) break;

    if (muestras == 0) {
      enviarDebug("[CALIBRACION-CORRIENTE] No se pudo leer corriente en el punto " + String(i + 1) +
                   ". Cancelada.");
      cancelado = true;
      break;
    }

    tablaTemporal[i] = (float)(sumaCorriente / muestras);
    enviarDebug("[CALIBRACION-CORRIENTE] Punto " + String(i + 1) + ": corriente normal = " +
                 String(tablaTemporal[i], 1) + " mA (" + String(muestras) + " muestras).");
  }

  velocidadActual = 0;
  sms_sts.WriteSpe(SERVO_ID, 0, 0);
  delay(50);
  actualizarConteoVueltas();

  if (cancelado) {
    enviarDebug("[CALIBRACION-CORRIENTE] Cancelada. La tabla de corriente NO se modifico.");
    return;
  }

  for (int i = 0; i < NUM_PUNTOS_CALIBRACION_CORRIENTE; i++) corrienteNormalMA[i] = tablaTemporal[i];
  calibracionCorrienteValida = true;
  guardarCalibracionCorriente();

  enviarDebug("[CALIBRACION-CORRIENTE] Completa y guardada en memoria (no hace falta repetirla en el "
               "proximo arranque). Margen aplicado: el mayor entre +" + String(margenCorrientePorcentaje, 0) +
               "% y +" + String(MARGEN_CORRIENTE_MINIMO_MA, 1) + " mA fijos. Envia '" +
               String(COMANDO_REGRESO) + "' para regresar a home antes de iniciar una prueba de vueltas.");
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

  // El sensor IR (emisor GPIO26 + interrupcion de su receptor) solo se
  // enciende para el giro hacia adelante en modo funcionamiento normal
  // (n > 0). Con n < 0 (giro en reversa via "vueltas tiempo") permanece
  // apagado, igual que en calibracion y regreso a home.
  bool giroNormalAdelante = (n > 0.0f);
  if (giroNormalAdelante) activarSensorIR();

  // Aviso previo (informativo): la curva en S exige, cerca de la mitad del
  // tiempo, un PICO de velocidad 1.5x mayor que el promedio nominal (ver
  // derivadaCurvaS(): maximo 1.5 en u=0.5, promedio 1 en [0,1]). Si ese pico
  // ya supera SPEED_MAX, el controlador lo saturara en tiempo real (avisara
  // de nuevo alli la primera vez que ocurra) y es posible que no se
  // completen exactamente las vueltas pedidas en el tiempo pedido.
  float tSeg          = minutos * 60.0f;  // segundos
  float rpmPromedio   = (fabsf(n) / tSeg) * 60.0f;
  float rpmPico       = rpmPromedio * 1.5f;
  int16_t speedPico   = (int16_t)(rpmPico / RPM_POR_UNIDAD + 0.5f);

  if (speedPico > SPEED_MAX) {
    float rpmMax = SPEED_MAX * RPM_POR_UNIDAD;
    enviarDebug("[ADVERTENCIA] La curva en S pedira un pico de aprox. " + String(rpmPico, 1) +
                 " rpm (promedio solicitado: " + String(rpmPromedio, 1) +
                 " rpm), que supera la maxima alcanzable (" + String(rpmMax, 1) +
                 " rpm). El controlador saturara la velocidad cerca de la mitad del giro; "
                 "es posible que no se completen exactamente " + String(fabsf(n), 2) +
                 " vuelta(s) en " + String(minutos, 2) + " min.");
  }

  // Arranque siempre en velocidad 0: la curva en S se encarga de subir la
  // velocidad de forma gradual (ver controlarVelocidadObjetivo()) - no se
  // comanda ninguna velocidad "nominal" de entrada, precisamente para evitar
  // el arranque brusco que tenia el controlador de v9/v10.
  iniciarConteoVueltas();
  velocidadActual = 0;
  sms_sts.WriteSpe(SERVO_ID, 0, 0);
  sonarInicioMovimiento();

  unsigned long tStart = millis();
  unsigned long tTotal = (unsigned long)(minutos * 60000.0f);
  tUltimoDebugVueltas                 = tStart;
  tUltimoControlVelocidad             = tStart;
  tInicioAsentamientoCorriente        = tStart;
  advertenciaSaturacionControlEnviada = false;
  advertenciaReversaControlEnviada    = false;

  while (millis() - tStart < tTotal) {
    if (procesarInterrupciones()) {
      enviarDebug("Rotacion cancelada por interrupcion. Vueltas acumuladas: " +
                   String(vueltasActuales(), 3));
      if (giroNormalAdelante) desactivarSensorIR();
      return;
    }
    revisarRearmeFinesDeCarrera();
    revisarOcultarAlertaBurbuja();
    if (monitorearCorriente(dentroDeAsentamientoArranque())) {
      enviarDebug("[SOBRECORRIENTE] Movimiento pausado. Envia '" + String(COMANDO_REGRESO) +
                   "' para ir a home o '" + String(COMANDO_CONTINUAR) +
                   "' para continuar (se respetan el tiempo y las vueltas pendientes).");

      unsigned long tPausaInicio = millis();
      int decision = esperarComandoSobrecorriente();

      if (decision == 2) {
        enviarDebug("Rotacion cancelada por interrupcion durante la pausa de sobrecorriente. "
                     "Vueltas acumuladas: " + String(vueltasActuales(), 3));
        if (giroNormalAdelante) desactivarSensorIR();
        return;
      }
      if (decision == 1) {
        enviarDebug("Rotacion cancelada por sobrecorriente: yendo a home. Vueltas acumuladas: " +
                     String(vueltasActuales(), 3));
        if (giroNormalAdelante) desactivarSensorIR();
        secuenciaRegreso();
        return;
      }

      // decision == 0 (COMANDO_CONTINUAR): el tiempo en pausa NO cuenta
      // contra el presupuesto de tTotal (se corre tStart hacia adelante esa
      // misma duracion), para que la curva en S retome exactamente en el
      // punto "u" que le corresponde - no se comanda una velocidad fija de
      // arranque; se fuerza que controlarVelocidadObjetivo() recalcule de
      // inmediato (en vez de esperar hasta INTERVALO_CONTROL_VELOCIDAD_MS)
      // para que el motor no quede detenido de mas tras el "continuar".
      tStart += (millis() - tPausaInicio);
      tInicioAsentamientoCorriente = millis();
      sonarInicioMovimiento();
      enviarDebug("[SOBRECORRIENTE] Reanudando movimiento...");
      tUltimoControlVelocidad = 0;
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

  if (giroNormalAdelante) desactivarSensorIR();

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

  // Restaurar la calibracion de corriente guardada en un uso anterior (si
  // existe), para no tener que recalibrar en cada arranque.
  cargarCalibracionCorriente();

  // Restaurar el margen porcentual de sobrecorriente ajustado en un uso
  // anterior por Serial/Nextion (si existe; si no, queda el valor por
  // defecto MARGEN_CORRIENTE_PORCENTAJE_DEFECTO).
  cargarMargenCorriente();

  // Configurar pines de interrupcion con pull-up interno
  pinMode(PIN_EMERGENCIA,   INPUT_PULLUP);
  pinMode(PIN_FIN_CARRERA1, INPUT_PULLUP);
  pinMode(PIN_FIN_CARRERA2, INPUT_PULLUP);
  pinMode(PIN_IR_BURBUJA, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_EMERGENCIA),   isrEmergencia,  FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_FIN_CARRERA1), isrFinCarrera1, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_FIN_CARRERA2), isrFinCarrera2, FALLING);

  // Emisor (TX) del sensor IR de burbuja: apagado por defecto. Su
  // interrupcion (PIN_IR_BURBUJA) NO se activa aqui - solo se activa junto
  // con el emisor, dentro de rotarVueltas(), y unicamente durante el giro
  // hacia adelante en modo normal (n > 0). Ver activarSensorIR().
  pinMode(PIN_IR_TX, OUTPUT);
  digitalWrite(PIN_IR_TX, LOW);

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
               "O envia '" + String(COMANDO_CALIBRAR_CORRIENTE) + "' para calibrar el umbral de "
               "sobrecorriente adaptativo bajo la carga real (recomendado antes de la primera prueba).\r\n"
               "O envia '" + String(COMANDO_MARGEN_CORRIENTE) + "' + numero (ej. \"T69\") para ajustar "
               "el margen de sobrecorriente a 69% sin recargar el codigo. Margen actual: " +
               String(margenCorrientePorcentaje, 1) + "%.\r\n"
               "Si hay sobrecorriente durante un giro, queda en pausa: '" +
               String(COMANDO_REGRESO) + "' = ir a home, '" +
               String(COMANDO_CONTINUAR) + "' = continuar.");
}

void loop() {
  procesarInterrupciones();
  revisarRearmeFinesDeCarrera();
  revisarOcultarAlertaBurbuja();
  monitorearCorriente(false);

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

    // Comando de una letra ('i'/'I'): dispara la calibracion del umbral de
    // sobrecorriente adaptativo bajo la carga real ya conectada (ver
    // COMANDO_CALIBRAR_CORRIENTE).
    if (primerByte == 'i' || primerByte == 'I') {
      Serial.read();
      while (Serial.available()) Serial.read();  // limpiar resto de la linea
      calibrarCorrientePorVelocidad();
      return;
    }

    // Comando 'T'/'t' + numero (ej. "T69"): ajusta el margen porcentual de
    // sobrecorriente sin recompilar/recargar el codigo (ver
    // COMANDO_MARGEN_CORRIENTE / procesarComandoMargenCorriente()).
    if (primerByte == COMANDO_MARGEN_CORRIENTE || primerByte == 't') {
      procesarComandoMargenCorriente();
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
