#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <BluetoothSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define vel_com 115200
#define long_array 160

byte datos[160];

BluetoothSerial SerialBT;

// ---------------- ToF ----------------
SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData;
SF_VL53L5CX_RANGING_MODE ranging_mode;

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

// ---------------- PID ----------------
float Kp = 7.5;
float Ki = 6.0;
float Kd = 0.0;

float X = 0.0;          // yaw actual 0..360
float ref_yaw = 0.0;    // rumbo objetivo absoluto 0..360
float ref_yaw_init = 0.0; //refrencia inical fija

float error = 0.0;
float prev_error = 0.0;

long  T;
long  prev_T;
float delta_T;

float integral = 0.0;
float derivator = 0.0;
float u = 0.0;

bool started = false;   // estado del START/STOP

const float INTEGRAL_MAX = 100.0;
const float U_MAX        = 100.0;
float BASE_SPEED   = 100.0;

// ---------------- Referencia desde Matlab ----------------
float ref_angle_rel = 0.0;   // relativo (puede ser negativo)
uint8_t ref_col = 0;

const float angles_rel[8] = {
  -24.0625, -17.1875, -10.3125, -3.4375,
   3.4375,  10.3125,  17.1875,  24.0625
};  // angulos de yaw respecto a cada columna del sensor

// ---------------- funciones ángulos ----------------
float wrap360(float a) {
  while (a >= 360.0) a -= 360.0;
  while (a < 0.0)    a += 360.0;
  return a;
}

float wrap180(float a) {
  while (a > 180.0)  a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

// ---------------- PWM duty ----------------
unsigned int dutyToPWM(float dutyPercent) {
  if (dutyPercent < 0)   dutyPercent = 0;
  if (dutyPercent > 100) dutyPercent = 100;
  return (unsigned int)(dutyPercent * 32767.0 / 100.0);
}

// ---------------- ToF: enviar 5x8 a Matlab ----------------
void data_acquisition(){
  if (myImager.getRangingData(&measurementData)){
    int j = 0;
    for (int i = 8; i <= 47; i++) {
      int d = (int)measurementData.distance_mm[i];
      datos[2*j]     = (byte)(d / 256);
      datos[2*j + 1] = (byte)(d % 256);

      int st = (int)measurementData.target_status[i];
      datos[80 + 2*j]     = (byte)(st / 256);
      datos[80 + 2*j + 1] = (byte)(st % 256);

      j++;
    }
    SerialBT.write(datos,160);
  }
}

// ---------------- BT: leer comandos ----------------
void leerBT() {

  while (SerialBT.available() >= 2) {   // SIEMPRE llegan 2 bytes

    uint8_t cmd  = (uint8_t)SerialBT.read();  // byte 1
    uint8_t data = (uint8_t)SerialBT.read();  // byte 2

   if (cmd == 1) {
     // START + fijar referencia inicial
      bno.getEvent(&event);
      X = wrap360(event.orientation.x);

      ref_yaw_init = X; //refencia inical
      ref_yaw = X;        // 
      integral = 0.0;
      prev_error = 0.0;

      started = true;

      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }

     // ---------- STOP ----------
   else if (cmd == 2) {
      started = false;

      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      ledcWrite(ENA, 0);
      ledcWrite(ENB, 0);

      integral = 0.0;
      prev_error = 0.0;

    }


   else if (cmd == 3) {
      // Pedir datos ToF   ) ))
      data_acquisition();
      delay(10);     // ⏱️ espera 10 ms
      break;
    }

    else if (cmd == 4) {
      uint8_t idx = data;   // columna

      if (idx >= 1 && idx <= 8) {
        ref_col = idx;
        ref_angle_rel = angles_rel[idx - 1];

        bno.getEvent(&event);
        X = wrap360(event.orientation.x);
        ref_yaw = wrap360(X + ref_angle_rel);
        integral = 0.0;
        prev_error = 0.0;

        Serial.print("Nueva ref_yaw = ");
        Serial.println(ref_yaw);
      }

            
    }

    else if (cmd == 6) {
  // Volver a la referencia inicial
  ref_yaw = ref_yaw_init;

  integral = 0.0;
  prev_error = 0.0;

  Serial.println("Volviendo a referencia inicial");
}


   else if (cmd == 5) {
      // Cambiar velocidad base desde MATLAB (0–100 %)
      BASE_SPEED = constrain(data, 0, 100);
   }
  
  }
}



// ---------------- SETUP ----------------
void setup() {
  Serial.begin(vel_com);

  SerialBT.begin("ESP32BT_Rover4");

  Wire.begin();
  Wire.setClock(400000);

  if (!myImager.begin()) {
    Serial.println(F("Sensor ToF not found"));
    while (1);
  }

  myImager.setResolution(8*8);
  myImager.setRangingFrequency(10);
  ranging_mode = SF_VL53L5CX_RANGING_MODE::CONTINUOUS;
  myImager.setRangingMode(ranging_mode);
  myImager.setSharpenerPercent(10);
  myImager.startRanging();

  if (!bno.begin()) {
    Serial.println("BNO055 no detectado");
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(300);

  ledcAttachChannel(ENA, FREQ, RES, CHANNEL1);
  ledcAttachChannel(ENB, FREQ, RES, CHANNEL2);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  prev_T = micros();
  integral = 0;
  prev_error = 0;

  // primera referencia: ir recto (ref = yaw actual)
  bno.getEvent(&event);
  X = wrap360(event.orientation.x);
  ref_yaw = X;
}

// ---------------- LOOP ----------------
void loop() {

  // 1) Leer BT (Matlab pide datos o manda ref)
  leerBT();

  if (!started) {
   delay(5);
   return;
  }

  // 2) Leer IMU
  bno.getEvent(&event);
  X = wrap360(event.orientation.x);

  // 3) Error respecto a la referencia absoluta
  error = wrap180(ref_yaw - X);

  // 4) PID
  T = micros();
  delta_T = (float)(T - prev_T) / 1e6;
  prev_T = T;

  integral += error * delta_T;
  if (integral > INTEGRAL_MAX)  integral = INTEGRAL_MAX;
  if (integral < -INTEGRAL_MAX) integral = -INTEGRAL_MAX;

  derivator = (error - prev_error) / delta_T;
  prev_error = error;

  u = Kp * error + Ki * integral + Kd * derivator;
  if (u > U_MAX)  u = U_MAX;
  if (u < -U_MAX) u = -U_MAX;

  // 5) Motores
  float leftSpeed  = BASE_SPEED + u;
  float rightSpeed = BASE_SPEED - u;

  if (leftSpeed  < 0) leftSpeed = 0;
  if (leftSpeed  > 100) leftSpeed = 100;
  if (rightSpeed < 0) rightSpeed = 0;
  if (rightSpeed > 100) rightSpeed = 100;

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  ledcWrite(ENA, dutyToPWM(leftSpeed));
  ledcWrite(ENB, dutyToPWM(rightSpeed));

  delay(5);
}
