#include <BluetoothSerial.h>

#define ENA 12
#define IN1 14
#define IN2 27

#define ENB 33
#define IN3 26
#define IN4 25

#define CHANNEL1 0
#define CHANNEL2 1
#define FREQ 1000
#define RES 15

BluetoothSerial SerialBT;

void setup() {
  SerialBT.begin("ESP32BT_Rover4");

  
  ledcAttachChannel(ENA, FREQ, RES, CHANNEL1);
  ledcAttachChannel(ENB, FREQ, RES, CHANNEL2);

  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  ledcWrite(ENA, 0);   
  ledcWrite(ENB, 0);       

  delay(1000);
}


void loop() {
  int DutyLeft, DutyRight;          
  unsigned int cTonLeft, cTonRight; 

 
  
  if (SerialBT.available() >= 2) {
    
    DutyLeft = SerialBT.read() - 100;
    
    DutyRight = SerialBT.read() - 100;

   
    
    if (DutyLeft > 0) {             
      digitalWrite(IN1, LOW);       
      digitalWrite(IN2, HIGH);      
    } else if (DutyLeft < 0) {      
      digitalWrite(IN1, HIGH);      
      digitalWrite(IN2, LOW);       
    } else {                       
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }

   
    if (DutyRight > 0) {            
      digitalWrite(IN3, HIGH);      
      digitalWrite(IN4, LOW);      
    } else if (DutyRight < 0) {     
      digitalWrite(IN3, LOW);       
      digitalWrite(IN4, HIGH);      
    } else {                        
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }

  
    cTonLeft  = abs(DutyLeft) * 32767 / 100;
    cTonRight = abs(DutyRight) * 32767 / 100;

    ledcWrite(ENA, cTonLeft);
    ledcWrite(ENB, cTonRight);
  }
}
