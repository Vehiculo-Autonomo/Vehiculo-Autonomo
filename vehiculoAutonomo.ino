#include "HUSKYLENS.h"
#include "PIDLoop.h"
#include <Servo.h>

/*
-------------------------------------------------
             DELCARACIÓN VARIABLES
-------------------------------------------------
*/

// ----------------- MOTORES --------------------

// Motor Principal
#define pinMotor1 5  // Determina la dirección del motor
#define pinMotor2 4  // Determina la dirección del motor
#define enMotor 6    // Determina la fuerza del motor. Usa pwm 

//PIDLoop controlMain(5, 0.0001, 0, false)  // Controlador PID para el motor principal. Los valores son sin carga

#define velMin 50       // La velocidad mínima que se necesita para tener suficiente torque inicial

int velocidad = 0;      // Un valor entre 0-255 que se manda como pwm al puente H
signed int dvel = 0;    // La cantidad que la velocidad aumenta o disminuye en cada paso

// Servo
#define pinNicla1 A2
#define pinServo 10

Servo servo;
PIDLoop controlServo(1, 0, 0, true);      // Controlador PID el servomotor (por el momento lo dejaremos como P)

#define anguloMax 105
#define anguloMed 84
#define anguloMin 60

int angulo = anguloMed;         // El ángulo al que se encuentra el vehiculo, se recibe de la nicla
signed int dang = 0;

double anguloPid = angulo;      // El ángulo de la línea de seguimiento. Se usa en trackLine()
double anguloPidFil = anguloPid;

// ----------------- ENCODER --------------------

#define pinEncoderA 2
#define pinEncoderB 3

short valorEncoderA = 0;
short valorEncoderB = 0;  
signed long int encoderPosition = 0;

// ---------------- HUSKYLENS --------------------

#define crossID 1   // Ids que diferencian los diferentes objetos aprendidos por la huskylens
#define aprilID 2

HUSKYLENS huskylens;  // Crea un objeto con el cual reconoceremos a la husky

// -------------- INTERRUPCIONES -----------------

// Variable timers 
volatile bool banderaTimer = false;   // Al activarse la bandera se ejecutará una ronda de detección. Se activa con timer1

volatile int counterHusky = 0;        // Se contará cada que el timer interrupta
volatile int counterStop = 0;         //  ''

const int comparadorHusky = 100 / 16; // La cantidad de cuentas hasta que se active la detección de la husky. Cada 100ms
const int comparadorStop = 1500 / 16; // La cantidad de cuentas que dura una parada. Cada 3s

// Banderas 
bool banderaCross = false;    // Se activa cuando se detecta un paso peatonal, se desactiva después de la intersección. Se usa en trackCross()
signed int banderaApril = 0;  // Su valor depende de que tag sea leída. Afectará la decisión de movimiento despues de un paso peatonal. Su valor se determina en trackApril() y se usa en timer4


/*
-------------------------------------------------
             SETUP E INTERRUPCIONES
-------------------------------------------------
*/

void setup() {

  // Huskylens
  Serial.begin(115200);                                         //Start serial communication

  Wire.begin();                                                 //Begin communication with the Huskeylens
  while (!huskylens.begin(Wire)) {
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protol Type>>I2C)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }
  Serial.println("Conectado");

  // Motores
  pinMode(pinMotor1, OUTPUT);
  pinMode(pinMotor2, OUTPUT);
  servo.attach(pinServo);
 
  // Encoder
  attachInterrupt(digitalPinToInterrupt(pinEncoderA), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncoderB), readEncoderB, CHANGE);

  // Timers
  cli();

    // Timer programa
    TCCR2A = 0;
    TCCR2A |= (1 << WGM01);
    TCCR2B = 0;
    TCCR2B |= (1 << WGM01);
    TCCR2B |= (1 << CS12) | (1 << CS10);
    
    OCR2A = 249; // Preescaler 1024 | frecuencia 16ms | 62.5Hz
    TIMSK2 |= (1 << OCIE2A);
    TCNT2  = 0;

  sei();

  delay(1000);
  stopMove();
  moveForward();
  servo.write(anguloMed);
}

ISR(TIMER2_COMPA_vect) {
  banderaTimer = true;
}

void loop() {
  
  // 62.5 Hz | 16 ms
  if (banderaTimer) {

    banderaTimer = false;
    counterHusky += 1;
    counterStop += 1;
    
    // 100 ms
    if (counterHusky > comparadorHusky) {
      counterHusky = 0;

      if (!banderaCross) {
        // trackLine();
        // trackCross();
        false;
      }
      
      updateVel();
      // updateAng();
    }

    // RUTINA DE PRUEBA
    // if (counterStop > comparadorStop) {
    //   counterStop = 0;
    // //   angulo = anguloMax;
    //   if (angulo == 85){
    //     angulo = anguloMax;
    //   }
    //   else if (angulo == anguloMax) {
    //     angulo = anguloMin;
    //   }
    //   else {
    //     angulo = 85;
    //   }
    // }

    // 1.5 seg
    if (counterStop > comparadorStop) {
      counterStop = 0;
      trackApril();
      switch (banderaApril) {
        case -1:
          break;
        case 1:
          stopMove();
        case 2:
          stopMove();
          break;
        case 3:
          turnRight();
          break;
        case 4:
          turnLeft();
          break;
        case 5: 
          moveForward();
      }
      banderaApril = -1;
    }

  }
}

/*
-------------------------------------------------
              MOVIMIENTOS VEHÍCULO
-------------------------------------------------
*/

void updateVel() {
  // Actualiza la velocidad del motor main y lo manda al pin enMotor
  velocidad += dvel;
  velocidad > 255 ? velocidad = 255 : false;
  velocidad < velMin ? velocidad = 0 : false;
  analogWrite(enMotor, velocidad);
}

void updateAng() {
  angulo += dang;
  angulo > anguloMax ? angulo = anguloMax : false;
  angulo < anguloMin ? angulo = anguloMin : false;
  servo.write(angulo);
}

void moveForward() {
  digitalWrite(pinMotor1, HIGH);
  digitalWrite(pinMotor2, LOW);
  velocidad = velMin;
  dvel = 5;
}

void stopMove() {
  digitalWrite(pinMotor1, LOW);
  digitalWrite(pinMotor2, LOW);
  dvel = -10;
}

void turnRight() {

  stopMove();
  delay(3000);

  velocidad = 255;
  digitalWrite(pinMotor1, HIGH);
  digitalWrite(pinMotor2, LOW);
  analogWrite(enMotor, velocidad);

  while (angulo < anguloMax) {
    angulo += 2;
    servo.write(angulo);
    delay(200);
  }
  delay(6800);
  while (angulo > anguloMed) {
    angulo -= 2;
    servo.write(angulo);
    delay(200);
  }
  angulo = anguloMed;
  banderaCross = false;

}

void turnLeft() {

  stopMove();
  delay(3000);

  velocidad = 255;
  digitalWrite(pinMotor1, HIGH);
  digitalWrite(pinMotor2, LOW);
  analogWrite(enMotor, velocidad);

  while (angulo > anguloMin) {
    angulo -= 2;
    servo.write(angulo);
    delay(200);
  }
  delay(6800);
  while (angulo < anguloMed) {
    angulo += 2;
    servo.write(angulo);
    delay(200);
  }
  angulo = anguloMed;
  banderaCross = false;

}

/*
-------------------------------------------------
              FUNCIONES ENCODER
-------------------------------------------------
*/

void readEncoderA() {

  valorEncoderA = digitalRead(pinEncoderA);
   
  if (valorEncoderA == 1 && valorEncoderB == 1) {
    encoderPosition++;
  }
  else if (valorEncoderA == 1 && valorEncoderB == 0) {
    encoderPosition--;
  }
  else if (valorEncoderA == 0 && valorEncoderB == 1) {
    encoderPosition--;
  }
  else if (valorEncoderA == 0 && valorEncoderB == 0) {
    encoderPosition++;
  }
}

void readEncoderB() {

  valorEncoderB = digitalRead(pinEncoderB);
   
  if (valorEncoderB == 1 && valorEncoderA == 0) {
    encoderPosition++;
  }
  else if (valorEncoderB == 1 && valorEncoderA == 1) {
    encoderPosition--;
  }
  else if (valorEncoderB == 0 && valorEncoderA == 0) {
    encoderPosition--;
  }
  else if (valorEncoderB == 0 && valorEncoderA == 1) {
    encoderPosition++;
  }
}

/*
-------------------------------------------------
              FUNCIONES DETECCIÓN
-------------------------------------------------
*/

void trackLine() {

  anguloPid = analogRead(pinNicla1);
  Serial.println(anguloPid);
  anguloPid = anguloPid * (double)(5.0/1023.0) * (double)(180.0/3.3);
  // anguloPidFil = 0.01*anguloPid + (1-0.01)*anguloPidFil;
  Serial.println(anguloPid);

  int32_t error;
  error = (int32_t)anguloPidFil - (int32_t)anguloMed;
  controlServo.update(error);

  angulo = controlServo.m_command;
  angulo > anguloMax ? angulo = anguloMax : false;
  angulo < anguloMin ? angulo = anguloMin : false;

  // Serial.println(angulo);
  servo.write(angulo);
  
}

void trackCross() {

  huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING);

  if (huskylens.request(crossID) && huskylens.available()) {
  
    HUSKYLENSResult result = huskylens.get(crossID);

    // Utiliza el área en pixeles del objeto para determinar que tan cerca está
    if (result.width*result.height >= 1000) { // AJUSTAR valor de área del cross
      if (!banderaCross) {
        banderaCross = true;
        stopMove();
        trackApril();
      }
      else {
        banderaCross = false;
        // AGREGAR tiempo para que pase el crossg
      }
    }
  }
}

void trackApril() {
  
  huskylens.writeAlgorithm(ALGORITHM_TAG_RECOGNITION);

  if (huskylens.request() && huskylens.available()) {
    
    Serial.println(huskylens.available());
    
    int width = 0;
    int id = 0;

    for (int i=0; i < huskylens.available(); i++) {
      HUSKYLENSResult result = huskylens.getBlock(i);
      if (result.width > width){
        width = result.width;
        id = result.ID;
      }
    }
    // printResult(result);
    // AGREGAR condición por si detecta más de una tag
    banderaApril = id;
    if (banderaApril != -1) {
      velocidad = 0;
      stopMove();
    }
    // AGREGAR loop por si le toma tiempo leer la tag
    // AGREAGAR condición de si no encuentra tag sea el valor de forward
  }
  
}

void printResult(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
   Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
    }
    else if (result.command == COMMAND_RETURN_ARROW){
        Serial.println(String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID);
    }
    else{
        Serial.println("Object unknown!");
    }
}