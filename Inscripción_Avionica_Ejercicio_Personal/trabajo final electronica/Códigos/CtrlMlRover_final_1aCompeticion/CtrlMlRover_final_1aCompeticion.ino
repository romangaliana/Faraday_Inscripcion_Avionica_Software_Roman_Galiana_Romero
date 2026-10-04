#include <BluetoothSerial.h> // Librebria Bluetooth Serial del ESP32

// Definiciones de pines para el Motor A (Izquierdo)
#define ENA 12 // entrada de velocidadd (PWM izquierdo)
#define IN1 14 // (control de sentido atrás)
#define IN2 27 // (control de sentido adelante)

// Definiciones de pines para el Motor B (Derecho)
#define ENB 33 // entrada de velocidadd (PWM derecho)
#define IN3 26 // (control de sentido adelante)
#define IN4 25 // (control de sentido atrás)

// Configuración del PWM 
#define CHANNEL1 0 // izquierdo
#define CHANNEL2 1 // derecho
#define FREQ 1000 // PWM de 1kHz
#define RES 15 // 15 bits (de 0 a 32767)

BluetoothSerial SerialBT;

void setup() {
 
  SerialBT.begin("ESP32BT_Rover4"); 

  // control de velocidad independiente para cada motor
  ledcAttachChannel(ENA, FREQ, RES, CHANNEL1); // pin ENA (PWM izquierdo)
  ledcAttachChannel(ENB, FREQ, RES, CHANNEL2); // pin ENB (PWM derecho)

  // pines que controlan el sentido de velocidad configurados como salida
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // motores detenidos al inicio
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // motores empiezan apagados
  ledcWrite(ENA, 0);   
  ledcWrite(ENB, 0);       

  delay(1000); // esperar 1s 
}

void loop() {
  
  int kodularPower, kodularTurn; // Kodular envia 2 valores: POWER (0-200) - adelante/atrás ; TURN (0-200)- izquierda/dreceha
  
  // Definimos variables (tipo float para precisión)
  float powerY, turnX;
  float leftSpeed, rightSpeed;
  
  unsigned int cTonLeft, cTonRight; // PWM final de cada motor (0-32767)

  
  if (SerialBT.available() >= 2) { // Espera a recibir los 2 bytes (POWER y TURN) del Kodular
    
    kodularPower = SerialBT.read(); // Valor 0-200 de Kodular
    kodularTurn  = SerialBT.read(); // Valor 0-200 de Kodular

    // Ponemos la Potencia de (-100 a +100)
    // 0 -> -100 (Atrás)
    // 100 -> 0 (Parado)
    // 200 -> +100 (Adelante)
    powerY = (float)kodularPower - 100.0;

    // Ponemos el Turn de (-100 a +100)
    // 0 -> -100 (Izquierda)
    // 100 -> 0 (Recto)
    // 200 -> +100 (Derecha)
    turnX = (float)kodularTurn - 100.0;

    
    // Si TURN = 0 - ambos motores reciben misma potencia (sentido hacia adelante)
    // Si TURN > 0 - motor izquierdo mas rapido (giro a la derecha
    // Si TURN < 0 - motor derecho mmas rapido (giro a la izquierda)
    // Si POWER = 0 && TURN ≠ 0 - una oruga va hacia delante y la otra hacia atrás
    //                          → el rover gira sobre su propio eje (giro en el sitio)
    leftSpeed  = powerY + turnX; 
    rightSpeed = powerY - turnX;

    // limitamos de (-100 a +100) - positivos hacia adelante y negativos hacia atras
    leftSpeed  = constrain(leftSpeed, -100.0, 100.0); 
    rightSpeed = constrain(rightSpeed, -100.0, 100.0);

    
    // Motor Izquierdo (ENA)
    if (leftSpeed > 0) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH); // hacia adelante
    } else if (leftSpeed < 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW); // hacia atrás
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW); // motor apagado
    }

    // Motor Derecho (ENB)
    if (rightSpeed > 0) {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW); // hacia adelante
    } else if (rightSpeed < 0) {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH); // hacia atrás
    } else {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW); // motor apagado
    }


    // conversion a PWM (0-32767) (positivo siempre)
    cTonLeft  = (unsigned int)(abs(leftSpeed) * 32767.0 / 100.0); 
    cTonRight = (unsigned int)(abs(rightSpeed) * 32767.0 / 100.0);

    // Enviamos PWM a cada motor
    ledcWrite(ENA, cTonLeft);
    ledcWrite(ENB, cTonRight);
  }
}
