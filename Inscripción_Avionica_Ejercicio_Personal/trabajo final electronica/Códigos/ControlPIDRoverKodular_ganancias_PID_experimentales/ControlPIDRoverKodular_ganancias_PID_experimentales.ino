#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <BluetoothSerial.h>

// ---------------- IMU (BNO055) ----------------
Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensors_event_t event;  // Para guardar los ángulos de Euler

// ---------------- ROVER: Pines motores ----------------
// Definiciones de pines para el Motor A (Izquierdo)
#define ENA 12
#define IN1 14
#define IN2 27

// Definiciones de pines para el Motor B (Derecho)
#define ENB 33
#define IN3 26
#define IN4 25

// PWM
#define CHANNEL1 0         // PWM motor izquierdo
#define CHANNEL2 1         // PWM motor derecho
#define FREQ     1000      // PMW kHz
#define RES      15        // 15 bits → 0..32767

BluetoothSerial SerialBT;

// ---------------- Variables PID ----------------
// Ganancias (inicializamos a 0, las cambiaremos desde Kodular)
float Kp = 0.0;   // proporcional
float Ki = 0.0;  // integral
float Kd = 0.0;   // derivativa

// Variables del PID
float ref = 0.0;      // ángulo de refierencia Z = 0º
float X;              // Variable a controlar (ángulo de Euler Z)
float error;          // e(k)
float prev_error = 0; // e(k-1)
long  T;              // tiempo actual (micros)
long  prev_T;         // tiempo anterior (micros)
float delta_T;        // Ts = T - prev_T (en segundos)
float integral = 0;   // suma de errores (parte I)
float derivator = 0;  // derivada del error (parte D)
float u = 0;          // salida del PID (acción de control)

// Límites 
const float INTEGRAL_MAX = 100.0;  // límite de la integral
const float U_MAX        = 100.0;  // límite de la salida u (como “porcentaje extra”)

const float BASE_SPEED = 100.0;    // % de duty base hacia adelante

// ---------------- FUNCIONES AUXILIARES ----------------

// Convierte duty (0-100) a PWM (0-32767)
unsigned int dutyToPWM(float dutyPercent) {
  if (dutyPercent < 0)   dutyPercent = 0;
  if (dutyPercent > 100) dutyPercent = 100;
  return (unsigned int)(dutyPercent * 32767.0 / 100.0);
}

// Lee las ganancias PID que llegan desde Kodular (3 sliders: Kp, Ki, Kd)
void leerGananciasDesdeBT() {
  // esperamos que hayan 3 bytes en el buffer (recibidos de Kodular)
  if (SerialBT.available() >= 3) {
    uint8_t kpByte = SerialBT.read();  // 0-100
    uint8_t kiByte = SerialBT.read();  // 0-100
    uint8_t kdByte = SerialBT.read();  // 0-100

    Kp = kpByte / 10.0;
    Ki = kiByte / 10.0;
    Kd = kdByte / 10.0;
 
    Serial.print(" Kp = "); Serial.print(Kp);
    Serial.print(" Ki = "); Serial.print(Ki);
    Serial.print(" Kd = "); Serial.println(Kd);
  }
}

// ---------------- SETUP ----------------
void setup() {
  
  Serial.begin(115200);

  // --- IMU ---
  Wire.begin();
  Wire.setClock(400000);

  if (!bno.begin()) {
    Serial.println("BNO055 no detectado. Comprobar conexiones o dirección I2C");
    while (1); // bloqueado aquí si no hay IMU
  }
  bno.setExtCrystalUse(true);
  delay(1000);

  SerialBT.begin("ESP32BT_Rover4"); 

  // --- Motores (PWM + sentido) ---
  ledcAttachChannel(ENA, FREQ, RES, CHANNEL1); // motor izquierdo
  ledcAttachChannel(ENB, FREQ, RES, CHANNEL2); // motor derecho

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Ambos motores parados al inicio
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);



  // --- PID ---
  prev_T = micros();
  integral = 0;
  prev_error = 0;
}

// ---------------- LOOP ----------------
void loop() {

  // 1) Leer (si llegan) nuevas ganancias Kp, Ki, Kd desde Kodular
  leerGananciasDesdeBT();

  // 2) Leer IMU → ángulos de Euler
  bno.getEvent(&event);
  X = event.orientation.x;   // eje x = yaw (giro derecha/izquierda)

  // Convertir 0–360 a -180..+180
  if (X > 180) {
  X = X - 360;
  }

  // calculamos error
  ref   = 0.0;
  error = ref - X;           // e = 0 - X

  // 4) Calcular delta_T (en segundos)
  T = micros();
  delta_T = (float)(T - prev_T) / 1.0e6;   // micros → segundos

  // Parte integradora del PID
  integral += error * delta_T;

  // Saturación anti-windup de la integral
  if (integral > INTEGRAL_MAX)  integral = INTEGRAL_MAX;
  if (integral < -INTEGRAL_MAX) integral = -INTEGRAL_MAX;

  // Parte derivativa del PID
  derivator = (error - prev_error) / delta_T;

  // Actualizar valores para la próxima iteración
  prev_T     = T;
  prev_error = error;

  // Salida PID: u = Kp·e + Ki·∫e + Kd·de/dt
  u = Kp * error + Ki * integral + Kd * derivator;

  // Saturar la salida u 
  if (u > U_MAX)  u = U_MAX;
  if (u < -U_MAX) u = -U_MAX;


  //  - Tomamos una velocidad base (BASE_SPEED) hacia delante en las dos ruedas.
  //  - Añadimos +u a la izquierda y -u a la derecha segun nuestro control PID:
  
  //  EJ: Si el rover gira hacia la derecha (X>0 - error(0-X)<0 - u<0):
  // leftSpeed  < base, rightSpeed > base → gira hacia la izquierda para corregir.
 
  float leftSpeed  = BASE_SPEED + u;
  float rightSpeed = BASE_SPEED - u;

  // Limitamos a [0,100]% 
  if (leftSpeed  < 0)   leftSpeed  = 0;
  if (leftSpeed  > 100) leftSpeed  = 100;
  if (rightSpeed < 0)   rightSpeed = 0;
  if (rightSpeed > 100) rightSpeed = 100;

  // sentido siempre hacia adelante
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  // motor izquierdo hacia delante
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);   // motor derecho hacia delante

  // Aplicar PWM (0-32767) según las velocidades calculadas (aseguramos que son positivas)
  unsigned int cTonLeft  = dutyToPWM(leftSpeed);
  unsigned int cTonRight = dutyToPWM(rightSpeed);

  ledcWrite(ENA, cTonLeft);
  ledcWrite(ENB, cTonRight);

  Serial.print("X = "); Serial.print(X);
  Serial.print("  error = "); Serial.print(error);
  Serial.print("  u = "); Serial.print(u);
  Serial.print("  L% = "); Serial.print(leftSpeed);
  Serial.print("  R% = "); Serial.println(rightSpeed);

  // Tiempo de muestreo 
  delay(50);   // 50 ms (20 Hz)
}
