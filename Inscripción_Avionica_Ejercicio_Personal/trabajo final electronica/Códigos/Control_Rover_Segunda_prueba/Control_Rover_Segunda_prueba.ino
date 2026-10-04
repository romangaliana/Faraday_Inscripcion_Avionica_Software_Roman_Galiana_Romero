#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <BluetoothSerial.h>

// ---------------- IMU (BNO055) ----------------
Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensors_event_t event;

// ---------------- Pines del ROVER ----------------
#define ENA 12
#define IN1 14
#define IN2 27

#define ENB 33
#define IN3 26
#define IN4 25

// PWM
#define CHANNEL1 0
#define CHANNEL2 1
#define FREQ     1000
#define RES      15

BluetoothSerial SerialBT;

// ---------------- PID (CONSTANTES FINALES) ----------------
float Kp = 7.5;   // Proporcional
float Ki = 6.0;   // Integral
float Kd = 0.0;   // Derivativa

// ---------------- Variables PID ----------------
float ref = 0.0; // ponemos la referencia iniclal 0 (la iremos cambiando con el boton de reset de Kodular)
float X;
float error;
float prev_error = 0;

long  T;
long  prev_T;
float delta_T;

float integral = 0;
float derivator = 0;
float u = 0;

bool started = false;   // estado del START/STOP

// Límites
const float INTEGRAL_MAX = 100.0;
const float U_MAX        = 100.0;

const float BASE_SPEED = 100.0;

// ---------------- Función duty→PWM ----------------
unsigned int dutyToPWM(float dutyPercent) {
  if (dutyPercent < 0)   dutyPercent = 0;
  if (dutyPercent > 100) dutyPercent = 100;
  return (unsigned int)(dutyPercent * 32767.0 / 100.0);
}

// ---------------- PROCESAR COMANDOS BT ----------------
void leerComandosBT() {
  while (SerialBT.available() > 0) {

    int comando = SerialBT.read();  // <-- recibe 1 o 2

    if (comando == 1) {
      bno.getEvent(&event);
      X = event.orientation.x;
      // RESET
      ref = X;
      integral = 0;
      prev_error = 0;

      Serial.println(">> RESET (1) recibido");
    }

    else if (comando == 2) {
      // START/STOP
      started = !started;

      if (!started) {
        // Apagar motores
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        ledcWrite(ENA, 0);
        ledcWrite(ENB, 0);

        Serial.println(">> STOP (2) recibido");
      } else {
        Serial.println(">> START (2) recibido");
      }
    }
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);

  // IMU
  Wire.begin();
  Wire.setClock(400000);

  if (!bno.begin()) {
    Serial.println("BNO055 no detectado. Revisar conexiones.");
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(1000);

  // Motores
  ledcAttachChannel(ENA, FREQ, RES, CHANNEL1);
  ledcAttachChannel(ENB, FREQ, RES, CHANNEL2);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);

  // Bluetooth
  SerialBT.begin("ESP32BT_Rover4");

  // PID
  prev_T = micros();
  integral = 0;
  prev_error = 0;
}

// ---------------- LOOP ----------------
void loop() {

  // 1) Leer comandos: RESET (1) / START-STOP (2)
  leerComandosBT();

  // Si no está iniciado → motores off
  if (!started) {
    delay(20);
    return;
  }

  // 2) Leer IMU
  bno.getEvent(&event);
  X = event.orientation.x;

  
  float diff = X - ref; // nuestro angulo va a ser este 
  // pues cuando le damos al boton de reset el angulo que lee en esos momentos va ser su referencia 
  //(nuestro angulo 0)
  if (diff > 180) diff = diff - 360;
  if (diff < -180) diff = diff + 360; 


  // 3) PID x
  error = -diff; // ref - x (pero de -180 a 180)

  T = micros();
  delta_T = (float)(T - prev_T) / 1e6;

  integral += error * delta_T;
  if (integral > INTEGRAL_MAX)  integral = INTEGRAL_MAX;
  if (integral < -INTEGRAL_MAX) integral = -INTEGRAL_MAX;

  derivator = (error - prev_error) / delta_T;

  prev_T = T;
  prev_error = error;

  u = Kp * error + Ki * integral + Kd * derivator;

  if (u > U_MAX)  u = U_MAX;
  if (u < -U_MAX) u = -U_MAX;

  // 4) CONTROL DE MOTORES
  float leftSpeed  = BASE_SPEED + u;
  float rightSpeed = BASE_SPEED - u;

  if (leftSpeed  < 0) leftSpeed = 0;
  if (leftSpeed  > 100) leftSpeed = 100;
  if (rightSpeed < 0) rightSpeed = 0;
  if (rightSpeed > 100) rightSpeed = 100;

  // Siempre adelante
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  ledcWrite(ENA, dutyToPWM(leftSpeed));
  ledcWrite(ENB, dutyToPWM(rightSpeed));

  // Debug
  Serial.print(" X absoluta = "); Serial.print(X);
  Serial.print(" X = "); Serial.print(diff); // la X de verdad en el sistema de corrdenadas del Rover con la refrencia
  Serial.print(" e = "); Serial.print(error);
  Serial.print(" u = "); Serial.print(u);
  Serial.print(" L = "); Serial.print(leftSpeed);
  Serial.print(" R = "); Serial.println(rightSpeed);

  delay(50);
}
