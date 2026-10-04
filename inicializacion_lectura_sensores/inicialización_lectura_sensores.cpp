#include <Wire.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <MS5611.h>
#include <Adafruit_BNO08x.h>
#include <SparkFun_LIS331.h>
#include <TinyGPS++.h>

// Instancias de los sensores con sus librerías (creación de los objetos)
MS5611 baro1(0x77);
MS5611 baro2(0x76);
Adafruit_BNO08x imu;
sh2_SensorValue_t sensorValue;
LIS331 highG;
TinyGPSPlus gps;

// Instanciación del puerto USART1 del GPS(RX: PA10, TX: PA9)
Uart SerialGPS(PA10, PA9);

void setup() {
  // 1. Iniciamos el puerto serie para ver los datos en el PC
  Serial.begin(115200); //inicio a 115200 baudios (bit/s)
  Serial.println("--- Iniciando Test de Sensores ---");

  // 2. Asignacion de los buses del STM32F405
  // Bus I2C1 para los 2 barometros
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  //activamos señal de reloj a 100KHz por defecto, configura PB6 y PB7 en modo open-drain y deja a SCL y SDA en 3.3V lista para transmitir
  Wire.begin(); 
  //Podriamos incrementar la velocidad de la comunicación elevando la velocidad del reloj 
  //con Wire.setClock(400000); (para 400Khz por ejemplo) 

  // Bus SPI1 para la IMU y Acelerometro High-G
  SPI.setSCLK(PA5);
  SPI.setMISO(PA6);
  SPI.setMOSI(PA7);
  SPI.begin();

 // Bus UART para el GPS
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
  if (!imu.begin_SPI(PB0, PC4)) { //(CS, INT) PC4 = INT_IMU
    Serial.println("Fallo IMU BNO085");
    while(1);
  }
  Serial.println("IMU BNO085 OK");
  
  // Pedimos al coprocesador interno de la IMU que empieze a tomar muestras y calcular 
  //y enviar la aceleracion total y orientacion (cuaterniones) con una frecuencia de muestreo de 100Hz
   // Perido de muestreo de 10000 μs = 100 Hz de frecuencia de muestreo 
  imu.enableReport(SH2_ACCELEROMETER, 10000); 
  imu.enableReport(SH2_ROTATION_VECTOR, 10000);

  // 6. Inicializacion Acelerometro High-G (SPI) (libreria SparkFun_LIS331)
  //PB1 pin CS_HIGHG
  highG.setSPICSPin(PB1); 
  //le indciamos que nos comunicamos a través del bus SPI
  highG.begin(LIS331::USE_SPI); 
  //por defecto el highG arranca en modo de apagado (lo encendemos)
  highG.setPowerMode(LIS331::NORMAL); 
  //le indicamos la frecuencia de muestreo de 100Hz
  highG.setODR(LIS331::DR_100HZ); 
  // pone los bits FS del sensor a 00 para su rango de +-100g con sensibilidad de 49mg/digit (máxima sensibilidad posible)
  highG.setFullScale(LIS331::LOW_RANGE); 

  Serial.println("--- TODO OK. LEYENDO DATOS ---");
}


void loop() {
  // =========================================================================
  // 1. TAREAS ASÍNCRONAS CONTINUAS (Vaciado de Búferes)
  // Corren a la máxima velocidad del microcontrolador (decenas de miles de Hz)
  // =========================================================================

  // A. Vaciado del UART del GPS
  // leemos GPS siempre que hayan datos disponibles en el buffer del UART
  // available() devuelve cuantos bytes han entrado por el pin Rx y estan acumulados en la memoria RAM esperando a que los leamos
  while (SerialGPS.available() > 0) { 
  //encode es la función creada en la libreria TinyGPSPlus 
  //va leyendo letras/carácteres (un byte) hasta llegar al final de la sentencia NMEA
  //verifica el checksum al final de la sentencia y si la frase es valida, 
  //traduce el texto ASCII a numeros decimales reales  y devuelve true 
    gps.encode(SerialGPS.read()); 
  }                           
  
 // B. Vaciado de buffer SHTP con librería Adafruit_BNO08x
 // Variable estática: conserva la última aceleración válida entre ciclos si en esta iteración no hay paquete nuevo
    static float a_bno_z = 0.0f;
    static float quat_r = 1.0f, quat_i = 0.0f, quat_j = 0.0f, quat_k = 0.0f;
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

  // =========================================================================
  // 2. TAREA TEMPORIZADA: VOLCADO DE TELEMETRÍA (4 Hz)
  // Imprime el estado actual de todas las variables sin bloquear la CPU
  // =========================================================================
  static unsigned long ultimoTiempo = 0;
  //cada 250ms (4Hz) leo e imprimo la telemetria evitando poner un delay (en el que el micro se queda congelado)
  //así, mientrás, el bucle while de arriba sigue vaciando los bytes del GPS
  if (millis() - ultimoTiempo >= 250) {  
    // al ser un tipo de dato unsigned long (un entero sin signo de 32 bits) tanto lo que devuelve la función millis y la variable declarada por mi,
    // la resta no dará negativo cuando se desborde al llegar a 2^32 
    // dará la diferencia de tiempo real  
    ultimoTiempo = millis();   

    // LECTURA DE BAROMETROS
    //toma las medidas y hace los calculos para presión y la temperatura
    int baro1_status = baro1.read();
    int baro2_status = baro2.read(); 
    // MS5611_READ_OK es un valor de la liberia (0) que indica que se ha verificado la lectura por el bus I2C
    if (baro1_status == MS5611_READ_OK && baro2_status == MS5611_READ_OK) { 
      float p1 = baro1.getPressure();
      float p2 = baro2.getPressure();
      //calculamos la altitud con el modelo ISA pues esta libreria no incluye el calulo de la altitud
      //formula ISA en la troposfera (hasta 11km)
      float alt1 = 44330.77 * (1.0 - pow(p1 / 1013.25, 0.1902949)); //estamos en la troposfera (hasta 11km)
      float alt2 = 44330.77 * (1.0 - pow(p2 / 1013.25, 0.1902949));
      
      Serial.print("[BARO1] Alt: "); Serial.print(alt1); 
      Serial.print(" m | Temp: "); Serial.print(baro1.getTemperature()); Serial.println(" C");
      
      Serial.print("[BARO2] Alt: "); Serial.print(alt2); 
      Serial.print(" m | Temp: "); Serial.print(baro2.getTemperature()); Serial.println(" C");
    }

    // LECTURA HIGH-G (H3LIS331DL)
    int16_t x_raw, y_raw, z_raw_highg; // Registros enteros crudos de 16 bits (complemento a 2). El ADC interno es de 12 bits justificado a la izquierda (con 4 ceros en los bits menos significativos)
    highG.readAxes(x_raw, y_raw, z_raw_highg); // El sensor envía por SPI 6 bytes (ejes X, Y, Z) y la función junta el byte alto con el bajo: (OUT_H << 8) | OUT_L
    // Conversión del eje vertical Z: de cuentas ADC a "g" configurado en rango +/-100g (sensibilidad ~49 mg/digit a 12 bits) y luego a m/s^2 (SI)
    float a_highg_z = highG.convertToG(100, z_raw_highg) * 9.80665f;
    Serial.print("[HIGH-G] Z: "); 
    Serial.print(a_highg_z); Serial.println("m/s^2");

    
    // IMU BNO085 (Mostrando las variables guardadas en el vaciado asíncrono)
    Serial.print("[IMU] Acel Z: ");
    Serial.print(a_bno_z, 2); Serial.println(" m/s^2");
    
    Serial.print("[IMU] Cuat(r,i,j,k): ");
    Serial.print(quat_r, 3); Serial.print(", ");
    Serial.print(quat_i, 3); Serial.print(", ");
    Serial.print(quat_j, 3); Serial.print(", ");
    Serial.println(quat_k, 3);

    // IMPRESIÓN GPS
    // isValid() es true siempre que encode haya descifrado ya tanto la longitud y latitud como la altura 
    if (gps.location.isValid()) {  // cuando encode ha descifrado ya tanto la longitud como latitud esto devuelve true 
      Serial.print("[GPS] Sats: "); Serial.print(gps.satellites.value()); //número de satélites activos que el módulo tiene enganchados
      Serial.print(" | Lat: "); Serial.print(gps.location.lat(), 6); //resolucion de 6 decimales (del orden de 10cm  en coordenas terrestes)
      Serial.print(" | Lon: "); Serial.print(gps.location.lng(), 6);
      Serial.print(" | Alt: "); Serial.print(gps.altitude.meters()); Serial.println(" m");
    } else {
      // Da feedback visual constante en el monitor serie mientras pilla cobertura
      Serial.print("[GPS] Esperando Fix... (Sats visibles: "); 
      Serial.print(gps.satellites.value()); Serial.println(")");
    }
    
    Serial.println("-----------------------------------------");
  }
}