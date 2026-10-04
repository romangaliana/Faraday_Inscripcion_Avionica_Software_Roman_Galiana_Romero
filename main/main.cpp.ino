#include <Wire.h>
#include <SPI.h>
#include <MS5611.h>
#include <Adafruit_BNO08x.h>
#include <SparkFun_LIS331.h>
#include <TinyGPS++.h>
#include <cmath> 

// Instancias de los sensores con sus librerías (creación de los objetos)
MS5611 baro1(0x77);
MS5611 baro2(0x76);
Adafruit_BNO08x imu;
LIS331 highG;
TinyGPSPlus gps;

// Instanciación del puerto USART1 del GPS(RX: PA10, TX: PA9)
Uart SerialGPS(PA10, PA9);

// Definición Máquina de estados 
// Tipo de dato 'enum' asigna nombres a números enteros (0, 1, 2...).
// En la memoria del microcontrolador ocupa solo 1 byte (uint8_t), lo que ahorra RAM
// y permite transmitir el estado en telemetría enviando solo un número.
enum FlightState {
    PAD,        // 0: En rampa de lanzamiento
    BOOST,      // 1: Fase propulsada (motor encendido)
    COAST,      // 2: Vuelo inercial (motor apagado, subiendo a apogeo)
    APOGEE,     // 3: Apogeo detectado (expulsión de paracaídas)
    DESCENT,    // 4: Caída bajo paracaídas
    LANDED      // 5: En el suelo
};
//iniciamos la variable que almacena el estado actual del cohete en reposo sobre la rampa (PAD=0)
FlightState currentState = PAD;

// Vector de estado: x = [z, v, b]^T
float z_est = 0.0f;
float v_est = 0.0f;
float b_est = 0.0f; //estimación incial del sesgo del sensor (por deriva termica)

// Matriz de Covarianza P (Incertidumbre)
float P_zz = 0.1f; // (m^2) incertudmbre de partida de altitud y velcidad muy poca, pues el cohete está en rampa
float P_vv = 0.1f; // ((m/s)^2) inicializo los dos con varianza de 0.1 (desvaicion tipica de 0.316m^2)
float P_bb = 5.0f; // ((m/s^2)^2)) se desconfia por completo de la estimación incial del sesgo del sensor (b_est)
float P_zv = 0.0f; // (m^2/s) al princpio los errores inciales de los 3 estados son independientes entre si
float P_zb = 0.0f; // (m^2/s^2) no hay correlación previa
float P_vb = 0.0f; // ((m/s) * (m/s^2) = m^2/s^3) covarianzas cruzadas empiezan en 0


// Varianzas de Ruido de Proceso (Q) y Medida (R)

//(matriz de Rudio del proceso Q)- Incertudimbre del modelo físico
const float Q_z = 0.001f; // (m^2) varianza de ruido inyectada en altitud (perturbaciones aerodinámicas y turbulencias)
const float Q_v = 0.01f; // (((m/s)^2)) varianza de ruido inyectada en la velocidad (vibraciones del motor, combustión, resonancias, etc)
const float Q_b = 0.0001f; // ((m/s^2)^2)) varianza del rudio del sesgo (asumiendo un paseo aleatorio (random walk) auqnue en verdad sea por la deriva térmica)
// defincion formal seria tasa de diseprsión de la incertidumbre por unidad de tiempo
//cuanto permites que el sesgo varie aleatoriamiente de un paso al siguiente 
// desviación típica de 0.01m/s^2 (se espera que el error interno del acelerometró cambie, como máximo 1cm/s^2 en cada paso) 

// Incertidumbre de medida
const float R_baro = 2.0f; // (m^2) varianza del rudio del barometro  (flujo turbulento, fluctuaciones de presión dinámicas, ruido del puerto estático, etc)
const float R_gps  = 9.0f; // (m^2) varianza del ruido vertical del gps 
 

// Umbrales de conmutación de sensores
const float HIGH_G_THRESHOLD = 7.0f * 9.80665f; // rango de +-8g segun datasheet de BNO085 (+1g de margen por seguridad)
const float SPACE_ALT_THRESHOLD = 30000.0f; // // Corte de barometro a 30 km (limite inferior de 10 mbar según datasheet (= 31056m segun ISA))
//margen de seguridad de 1km antes de alacanzar los 10mbar mínimos que soporta el chip

// Variables de control de tiempo y referencias (las inicializamos a 0)
unsigned long t_prev = 0; // Marca de tiempo del ciclo anterior para calcular el paso de integración (dt) en la predicción del filtro de Kalman
float z_ground = 0.0f;          // Altitud de incial en la rampa
int burnout_confirm_ticks = 0; // Contador para fin de empuje
int apogee_confirm_ticks = 0;  // "" para llegada al apogeo

// Variables de telemetría pasiva (Actitud y Coordenadas)
float quat_r = 1.0f, quat_i = 0.0f, quat_j = 0.0f, quat_k = 0.0f;
double gps_lat = 0.0, gps_lon = 0.0;

// Función de conversión altitud ISA multietapa (la creo pues no hay en la libreria MS5611)
float getAltitudeISA(MS5611 &baro, float seaLevelhPa = 1013.25f) { //si no le introducimos un segundo argumento, nos fijará la presión estandar a nivel del mar de 1013.25 hPa
  float p = baro.getPressure(); // Presión en hPa (mbar)

  // Presiones de transición según condiciones estándar ISA a nivel del mar
  float p1 = seaLevelhPa * 0.223361f; // ~226.32 hPa (Límite 11 km)
  float p2 = p1 * 0.241879f;          // ~54.748 hPa (Límite 20 km)

  // Capa 0: Troposfera (0 m a 11 000 m) Gradiente térmico negativo (grad = -6.5 K/km).
  if (p >= p1) {
    return 44330.77f * (1.0f - powf(p / seaLevelhPa, 0.190263f));
  }
  // Capa 1: Estratosfera Isoterma (11 000 m a 20 000 m) Capa isotérmica (T = 216.65 K cst)
  else if (p >= p2) {
    return 11000.0f - 6341.62f * logf(p / p1);
  }
  // Capa 2: Estratosfera Superior con gradiente positivo (20 000 m a 32 000 m) (grad = +1 K/km).
  else {
    return 20000.0f + 216650.0f * (1.0f - powf(p / p2, 0.029271f));
  }
}

//FILTRO DE KALMAN: función para la ETAPA DE CORECCIÓN 
void kalman_update_altitude(float z_meas, float R_sensor) {
    float y = z_meas - z_est; // Innovación (lectura del sensor - posición estimada)
    float S = P_zz + R_sensor; // Varianza de la lectura

    // Rechazo de "outlier" (valor atípico)
    if ((y * y) > (16.0f * S)) return; 
    // si la lectura está a mas de 4 desviaciones típicas de la estimación lo damos como una lectura invalida

    // Ganancia de Kalman
    float K_z = P_zz / S;
    float K_v = P_zv / S;
    float K_b = P_zb / S;

    // Corrección del estado
    z_est += K_z * y;
    v_est += K_v * y;
    b_est += K_b * y;

    // Actualización de la covarianza

    // se guardan las covarianzas previas para operar con ellos en varias lineas
    float P_zv_old = P_zv;
    float P_zb_old = P_zb;

    P_zz = (1.0f - K_z) * P_zz;
    P_zv = (1.0f - K_z) * P_zv_old;
    P_zb = (1.0f - K_z) * P_zb_old;
    P_vv = P_vv - K_v * P_zv_old;
    P_vb = P_vb - K_v * P_zb_old;
    P_bb = P_bb - K_b * P_zb_old;
}

void setup() {
    // 1. Iniciamos el puerto serie para ver los datos en el PC
    Serial.begin(115200); //inicio a 115200 baudios (bit/s)
    Serial.println("Sistema de navegación iniciado");

    // 2. Asignacion de los buses del STM32F405
    // Bus I2C1 para los 2 barometros
    Wire.setSCL(PB6);
    Wire.setSDA(PB7);
    Wire.begin(); //
    Wire.setClock(400000); // cambiamos señal de reloj a 400kHz 

     // Bus SPI1 para la IMU y Acelerometro High-G
    SPI.setSCLK(PA5);
    SPI.setMISO(PA6);
    SPI.setMOSI(PA7);
    SPI.begin();          

    // Bus UART para GPS
    SerialGPS.begin(115200); // El NEO-M9N transmite a 115200 bits/s
    Serial.println("GPS configurado (SerialGPS). Esperando satélites...");

    // 3. Configuración de los pines Chip Select (CS) del SPI manteniéndolos en nivel alto por defecto
    pinMode(PB0, OUTPUT); //CS_IMU
    digitalWrite(PB0, HIGH);
    pinMode(PB1, OUTPUT); //CS_HIGHG
    digitalWrite(PB1, HIGH);

   // 4. Inicializacion Barometros (I2C) (Librería MS5611)
   //la libreria selecciona por defecto máxima resolución (OSR (OverSampling Ratio)= 4096 con resolución de 0.012mbar (10cm))
    if (!baro1.begin()) {
        Serial.println("Fallo Barometro 1 (0x77)");
        while(1);
    }
    Serial.println("Barometro 1 OK");

    if (!baro2.begin()) {
        Serial.println("Fallo Barometro 2 (0x76)");
        while(1);
    }
    Serial.println("Barometro 2 OK");
    
    // 5. Inicializacion IMU BNO085 (SPI) con interrupción en PC4 (CS en PB0)
    if (!imu.begin_SPI(PB0, PC4)) {  //(CS, INT) PC4 = INT_IMU
        Serial.println("Error: BNO085");
        while (1);
    }
    Serial.println("IMU BNO085 OK");

    //modo para que entregue la aceleración total incluyendo la gravedad (la única que puede dar el otro sensor high-G)
    // Perido de muestreo de 10000 μs = 100 Hz de frecuencia de muestreo 
    imu.enableReport(SH2_ACCELEROMETER, 10000); 
    imu.enableReport(SH2_ROTATION_VECTOR, 10000); 

    // 6. Inicializacion Acelerometro High-G (SPI) (libreria SparkFun_LIS331)
    highG.setSPICSPin(PB1);
    highG.begin(LIS331::USE_SPI);
    //enciendo el sensor (por defecto empieza apagdo)
    highG.setPowerMode(LIS331::NORMAL); 
    //frecuencia de muestreo a 100Hz
    highG.setODR(LIS331::DR_100HZ); 
    // fijamos rango de +-100g con sensibilidad de 49mg/digit (máxima sensibilidad posible)
    highG.setFullScale(LIS331::LOW_RANGE); 
    Serial.println("Acelerometro High-G OK");

    // 7. Fijamos altitud en rampa promediando ambos sensores
    // Filosofía No-Go: el sistema no avanza si no se obtiene una lectura válida de ambos barómetros
    int s1 = baro1.read();
    int s2 = baro2.read();

    while (s1 != MS5611_READ_OK || s2 != MS5611_READ_OK) {
        //Margen de 10ms al sensor para asegurarse de que tenga la lectura lista
        // Oversampling ratio (OSR)=4096 para máxima resolución de 0.012mbar(10cm) - Conversion time (tc)= 8.22ms (máximo 9.04ms)
        delay(10); 
        s1 = baro1.read();
        s2 = baro2.read();
    } 

    // Llamamos a nuestra función ISA
    float z1_init = getAltitudeISA(baro1);
    float z2_init = getAltitudeISA(baro2);
    
    z_ground = 0.5f * (z1_init + z2_init); // Hacemos la media 
    z_est = z_ground; // Inicializamos el estado del filtro con la altitud en rampa
    Serial.print("Altitud de rampa fijada (z_ground): "); 
    Serial.print(z_ground, 2); 
    Serial.println(" m");
    
    Serial.println("--- SISTEMA LISTO. ESPERANDO DESPEGUE ---");
    t_prev = micros();
}

void loop() {
    // =========================================================================
    // 1. CÁLCULO DEL PASO TEMPORAL dt (Protegido contra overflow)
    // =========================================================================
    unsigned long t_now = micros(); //micros da el tiempo en μs 
    unsigned long delta_micros = t_now - t_prev; //calculamos la diferencia de tiempo de cada iteración manteniendo la precisión  de 1μs para integrar de manera precisa
    float dt = delta_micros / 1000000.0f; //convertimos a segundos en coma flotante (por la FPU que lleva el micro)
    t_prev = t_now; //actualizamos la marca de tiempo anterior para el siguiente ciclo  

    // Protección contra pasos de tiempo anómalos
    // Valor de rescate de 0.01s = 10ms: paso de tiempo coherente con el ODR (100 Hz) de los acelerómetros (IMU y High-G) 
    if (dt <= 0.0f || dt > 0.05f) dt = 0.01f; 
    // Protegemos si el bucle tarda menos de 1  μs (el micro va a 168 MHz, ejecuta una instrucción cada ~5.95 ns y podria tardar menos de 1  μs que daría dt=0)
    // protegenmos ante anomilas númericas como -Inf, Inf y NaN y de que diverja el filtro de Kalman si hay un bloqueo largo en el bus (>50ms)

    float a_raw = 0.0f; //Aceleración total (IMU o High-G) antes de restar gravedad y sesgo

    // =========================================================================
    // 2. LECTURA DE SENSORES INERCIALES (High-G y BNO085)
    // =========================================================================

    // LECTURA HIGH-G (H3LIS331DL)
    int16_t x_raw, y_raw, z_raw_highg; // Registros enteros crudos de 16 bits (complemento a 2). El ADC interno es de 12 bits justificado a la izquierda (con 4 ceros en los bits menos significativos)
    highG.readAxes(x_raw, y_raw, z_raw_highg); // El sensor envía por SPI 6 bytes (ejes X, Y, Z) y la función junta el byte alto con el bajo: (OUT_H << 8) | OUT_L
    // Conversión del eje vertical Z: de cuentas ADC a "g" configurado en rango +/-100g (sensibilidad ~49 mg/digit a 12 bits) y luego a m/s^2 (SI)
    float a_highg_z = highG.convertToG(100, z_raw_highg) * 9.80665f;  

    // LECTURA IMU BNO085 (Vaciado de buffer SHTP con librería Adafruit_BNO08x)
    // Variable estática: conserva la última aceleración válida entre ciclos si en esta iteración no hay paquete nuevo
    static float a_bno_z = 0.0f; 
    sh2_SensorValue_t sensorValue; // Estructura local en RAM para desrealizar las tramas del protocolo de transporte SHTP
    // Bucle while: extrae y vacía todo los reportes acumulados en el bufer del sensor por el bus SPI,
    // escribiendo los datos directamente en la dirección de memoria (&) de sensorValue para minimizar latencia
    while (imu.getSensorEvent(&sensorValue)) { //devuelve true si ha logrado extraer un paquete del buffer y false cuando está vacio
        // identificamos si es un paquete de aceleración  (ya entregada directamente en m/s^2 por el procesador interno)
        if (sensorValue.sensorId == SH2_ACCELEROMETER) { 
            // interpretamos el contenido del registro bajo el formato de aceleración lineal y extraemos la aceleración en el eje z
            a_bno_z = sensorValue.un.accelerometer.z; 
        }
        // identificamos si es un paquete de orientación (fusión EKF interna en cuaterniones)
        else if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {  
             //lo interpretamos como orientación y extraemos cada componente del cuaternión
             quat_r = sensorValue.un.rotationVector.real; // Parte escalar: cos(theta/2)
             quat_i = sensorValue.un.rotationVector.i;    // Parte vectorial eje X: ux * sin(theta/2)
             quat_j = sensorValue.un.rotationVector.j;    // Parte vectorial eje Y: uy * sin(theta/2)
             quat_k = sensorValue.un.rotationVector.k;    // Parte vectorial eje Z: uz * sin(theta/2)
        }
    }

    // Selección de acelerómetro según magnitud de aceleración
    // Si la aceleración supera el límite (en valor absoluto, sea empuje o frenada fuerte),
    // la IMU normal se satura, así que pasamos a leer del acelerómetro de alta escala (High-G)
    if (fabs(a_highg_z) > HIGH_G_THRESHOLD) {
        a_raw = a_highg_z; 
    } 
    // Si la aceleración es normal y no satura, usamos la IMU (BNO085) que es mucho más precisa y tiene menos ruido
    else {
        a_raw = a_bno_z; 
    }

    // =========================================================================
    // 3. FILTRO DE KALMAN: ETAPA DE PREDICCIÓN (Propagación del vector estado)
    // =========================================================================
    // Aceleración neta vertical real: restamos la gravedad terrestre y el sesgo estimado del sensor
    float a_net = (a_raw - 9.80665f) - b_est;

    // Integración de estados (integramos en el paso temporal dt)
    z_est += v_est * dt + 0.5f * a_net * dt * dt; // z = z + v*dt + 0.5*a*dt^2
    v_est += a_net * dt;                          // v = v + a*dt
    // b_est se mantiene constante en la predicción (su derivada temporal nominal es cero)

    // Propagación matricial de la incertidumbre (covarianza) P_k = F*P*F^T + Q
    // Se desprecian términos de orden dt^3 hacia arriba (a 100 Hz, dt=0.01s -> dt^3 = 10^-6, es despreciable)
    // Se guardan las varuiables previas para no pisar valores en los cálculos cruzados
    float P_vv_old = P_vv;
    float P_vb_old = P_vb;
    float P_zb_old = P_zb;
    float P_bb_old = P_bb;

    P_zz += dt * (2.0f * P_zv + dt * (P_vv_old - P_zb_old)) + Q_z;
    P_zv += dt * (P_vv_old - P_zb_old) - 1.5f * dt * dt * P_vb_old;
    P_zb += -dt * P_vb_old - 0.5f * dt * dt * P_bb_old;
    P_vv += -2.0f * dt * P_vb_old + dt * dt * P_bb_old + Q_v;
    P_vb += -dt * P_bb_old;
    P_bb += Q_b;

    // =========================================================================
    // 4. FILTRO DE KALMAN: CORRECCIÓN BAROMÉTRICA (Atmósfera densa)
    // =========================================================================
    // (Redundancia con 2 sensores MS5611)
    // Desconectamos el barómetro por encima de su limite de medición
    if (z_est < SPACE_ALT_THRESHOLD) {
        int s1 = baro1.read(); //tomamos meidas y calula presión y temperatura
        int s2 = baro2.read();
        
        // Comprobamos que la lectura de presión/temperatura no esté corrupta
        bool ok1 = (s1 == MS5611_READ_OK);
        bool ok2 = (s2 == MS5611_READ_OK);

        // Caso 1: Ambos barómetros funcionan correctamente
        if (ok1 && ok2) {
            // Llamada a nuestra función ISA para obtener la altitud
            float alt1 = getAltitudeISA(baro1);
            float alt2 = getAltitudeISA(baro2);
            float alt_promedio = 0.5f * (alt1 + alt2);
            // Al promediar dos sensores independientes con varianza R, la varianza del promedio es R/2 (0.5f * R_baro)
            kalman_update_altitude(alt_promedio, 0.5f * R_baro); 
        //utilizamos la medida de un solo sensor si uno de los 2 barometros falla (por avería o devuelve una lectura corrupta)
        // Caso 2: Falla el barómetro 2 -> usamos solo barómetro 1 con su varianza nominal (R_baro)
        } else if (ok1) {
            float alt1 = getAltitudeISA(baro1);
            kalman_update_altitude(alt1, R_baro);
        // Caso 3: Falla el barómetro 1 -> usamos solo barómetro 2 con varianza nominal (R_baro)
        } else if (ok2) {
            float alt2 = getAltitudeISA(baro2);
            kalman_update_altitude(alt2, R_baro); 
        }
        // Si fallan ambos, no se llama a update y el filtro navega con la altitud dada por el gps
    }

    // =========================================================================
    // 5. FILTRO DE KALMAN: CORRECCIÓN GPS (Asíncrona, 5-10 Hz)
    // =========================================================================
    // Vaciamos el búfer UART del GPS en cada ciclo para no acumular latencia
    while (SerialGPS.available()) {
        if (gps.encode(SerialGPS.read())) {
            // Solo entra aquí cuando se ha completado una frase NMEA válida con checksum correcto (.encode devuelve true)
            // Verificamos de forma combinada que tenemos una solución 3D completa
            // (latitud, longitud Y altitud recién salidas del módulo)
            if (gps.location.isUpdated() && gps.altitude.isUpdated()) {
                
                // Extraemos las coordenadas
                gps_lat = gps.location.lat();
                gps_lon = gps.location.lng();
                
                // Extraemos la altitud
                float alt_gps = (float)gps.altitude.meters();
                
                // Corrección del Filtro de Kalman.
                // R_gps es mucho mayor que R_baro (~9 m^2 vs ~2 m^2),
                // por lo que el filtro toma el GPS como una referencia suave a largo plazo,
                // impidiendo la deriva térmica del barómetro.
                kalman_update_altitude(alt_gps, R_gps);
            }
        }
    }

    // =========================================================================
    // 6. MÁQUINA DE ESTADOS: GESTIÓN DE LAS FASES DE VUELO
    // Evalúa las transiciones de vuelo exigiendo que la condición se cumpla nas cuantas veces seguidas
    // para evitar falsos positivos provocados por picos de ruido o vibraciones aerodinámicas.
    // =========================================================================
    
    switch (currentState) {
        
        // ---------------------------------------------------------------------
        // ESTADO 0: RAMPA DE LANZAMIENTO (PAD)
        // ---------------------------------------------------------------------
        case PAD:
            // Transición a BOOST: Condición doble de seguridad para confirmar el despegue.
            // Requiere superar simultáneamente 10 m/s de velocidad vertical estimada y ganar 15 m 
            // sobre la rampa para evitar falsos despegues por ráfagas de viento.
            if (v_est > 10.0f && (z_est - z_ground) > 15.0f) {
                currentState = BOOST;
                Serial.println("EVENTO: Despegue detectado -> BOOST");
            }
            break;

        // ---------------------------------------------------------------------
        // ESTADO 1: FASE PROPULSADA (BOOST)
        // ---------------------------------------------------------------------
        case BOOST:
            // Transición a COAST: Detección del fin de combustión del motor (Burnout).
            // Cuando el motor se apaga, la aceleración neta vertical cae por debajo de cero (a_net < 0),
            // indicando que la resistencia aerodinámica y la gravedad dominan la dinámica del vehículo.
            if (a_net < 0.0f) {
                burnout_confirm_ticks++; 
                //Exigimos que a_net sea negativa durante 3 ciclos consecutivos
                // (a 100 Hz = 30 ms) para descartar cualqueir tipo de oscilación 
                if (burnout_confirm_ticks >= 3) {
                    currentState = COAST;
                    Serial.println("EVENTO: Fin de empuje (Burnout) -> COAST");
                }
            } else {
                // Si la aceleración vuelve a ser positiva antes de 3 ciclos, reiniciamos el contador
                burnout_confirm_ticks = 0;
            }
            break;

        // ---------------------------------------------------------------------
        // ESTADO 2: VUELO INERCIAL (COAST)
        // ---------------------------------------------------------------------
        case COAST:
            // Transición a APOGEE: Detección del punto mas alto de la trayectoria.
            // Ocurre cuando la velocidad vertical estimada cruza por cero (v_est <= 0).
            // La condición (z_est - z_ground > 100.0f) impide una eyección accidental del paracaídas a baja altitud.
            if (v_est <= 0.0f && (z_est - z_ground) > 100.0f) {
                apogee_confirm_ticks++;
                // Exigimos que hayan 3 ciclos consecutivos con v <= 0 (30 ms a 100 Hz)
                // para evitar el disparo del pàracaidas por fluctuaciones de presión o ruido
                if (apogee_confirm_ticks >= 3) { 
                    currentState = APOGEE;
                    Serial.println("EVENTO: Apogeo detectado -> APOGEE");
                }
            } else {
                // Si la velocidad vuelve a ser positiva, reiniciamos el contador
                apogee_confirm_ticks = 0;
            }
            break;

        // ---------------------------------------------------------------------
        // ESTADO 3: APOGEO (APOGEE)
        // ---------------------------------------------------------------------
        case APOGEE:
            // Estado de acción instantánea: aquí se activa físicamente el canal pirotécnico
            // o servomotor que expulsa el paracaídas.
            Serial.println("EVENTO: Apogeo confirmado -> DESCENT");
            Serial.println("ACCION: Desplegando el paracaidas...");
            currentState = DESCENT; // Transición inmediata al descenso en el siguiente ciclo
            break;

        // ---------------------------------------------------------------------
        // ESTADO 4: DESCENSO (DESCENT)
        // ---------------------------------------------------------------------
        case DESCENT:
            // Fase de caída bajo paracaídas: monitorización de la posición y velocidad, y descenso hacia tierra.
            // Aquí se ubicaría la lógica para abrir el paracaídas principal a menor altitud (ej. 300 m)
            // o detectar el impacto final en suelo para pasar al estado LANDED.
            break;
    }

    // =========================================================================
    // 7. VOLCADO DE TELEMETRÍA (Imprimos solo a 10 Hz mediante temporizador)
    // Se limita a 10 Hz porque enviar cadenas de texto por puerto serie consume tiempo de CPU
    // convirtiendo números a caracteres ASCII y saturaría el búfer UART a 100 Hz.
    // =========================================================================
    static unsigned long t_telemetria = 0; //Marca de tiempo del último envío (persiste en RAM entre iteraciones)
    // Comprobamos si han transcurrido al menos 100 ms (frecuencia = 1 / 0.1 s = 10 Hz)
    if (millis() - t_telemetria >= 100) {
        t_telemetria = millis(); // Reiniciamos el cronómetro para contar otros 100ms
        // Estado operativo del vuelo (0: Pad/Rampa, 1: Ascenso motor, 2: Planeo inercial, 3: Descenso)
        Serial.print("St:"); Serial.print(currentState); 
        
        // Altitud estimada por Kalman en metros (1 decimal: resolución de 0.1 m suficiente para telemetría)
        Serial.print(" Z:"); Serial.print(z_est, 1); 
        
        // Velocidad vertical estimada en m/s (1 decimal: permite detectar apogeo cuando cruza v=0)
        Serial.print(" V:"); Serial.print(v_est, 1); 
        
        // Sesgo estimado del acelerómetro en m/s^2 (3 decimales: necesario para monitorizar convergencia del filtro)
        Serial.print(" b:"); Serial.print(b_est, 3); 
        
        // Parte escalar (real) del cuaternión de actitud (2 decimales: suficiente para la orientación)
        Serial.print(" Qr:"); Serial.print(quat_r, 2); //Qr-Parte escalar: cos(theta/2)
        
        // Latitud geográfica del GPS en grados decimales (6 decimales: precisión métrica necesaria para el rescate)
        Serial.print(" Lat:"); Serial.print(gps_lat, 6); 
        
        // Longitud geográfica del GPS con fin de línea (println cierra la trama y salta a la siguiente fila)
        Serial.print(" Lon:"); Serial.println(gps_lon, 6); 
    }
    
}