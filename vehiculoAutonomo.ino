#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include "PIDLoop.h"

/*
-------------------------------------------------
             DELCARACIÓN VARIABLES
-------------------------------------------------
*/

// ------------- Motores ---------------

#define pinMotor1 4  // Determina la dirección del motor
#define pinMotor2 5  // Determina la dirección del motor
#define enMotor 6    // Determina la fuerza del motor. Usa pwm 
#define pinServo 8

//PIDLoop controlMain(5, 0.0001, 0, false)  // Controlador PID para el motor principal. Los valores son sin carga
PIDLoop controlServo(1, 0, 0, true);      // Controlador PID el servomotor (por el momento lo dejaremos como P)

int angle_pid = 0;      // El ángulo de la línea de seguimiento. Se usa en trackLine()

#define velMin 50       // La velocidad mínima que se necesita para tener suficiente torque inicial

int velocidad = velMin; // Un valor entre 0-255 que se manda como pwm al puente H
signed int dvel = 5;    // La cantidad que la velocidad aumenta o disminuye en cada paso

// ------------ HUSKYLENS ----------------

#define crossID 1   // Ids que diferencian los diferentes objetos aprendidos por la huskylens
#define aprilID 2

HUSKYLENS huskylens;  // Crea un objeto con el cual reconoceremos a la husky

// ----------- Interrupciones -------------

// Variable timers 
volatile bool banderaTimer = false;   // Al activarse la bandera se ejecutará una ronda de detección. Se activa con timer1

// Banderas 
bool banderaCross = false;    // Se activa cuando se detecta un paso peatonal, se desactiva después de la intersección. Se usa en trackCross()
int banderaApril = 0;         // Su valor depende de que tag sea leída. Afectará la decisión de movimiento despues de un paso peatonal. Su valor se determina en trackApril() y se usa en timer4

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
  huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING);

  // Motores
  pinMode(pinMotor1, OUTPUT);
  pinMode(pinMotor2, OUTPUT);

  // PWM servo
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3  = 0;
  OCR3A = 1249; // Preescaler 256 | frecuencia 50 hz
  TCCR3B |= (1 << WGM10);
  TCCR3B |= (1 << CS12);   

  // Timers
  cli();

    // Timer programa
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    OCR1A = 6249; // Preescaler 256 | frecuencia 10 Hz
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS12);
    TIMSK1 |= (1 << OCIE1A);

    // Timer parada
    TCCR4A = 0;
    TCCR4B = 0;
    TCNT4  = 0;
    OCR4A = 46874; // Preescaler 1024 | frecuencia 1/3 Hz
    TCCR4B |= (1 << WGM12);
    TCCR4B |= (1 << CS12) | (1 << CS10);
    TIMSK4 |= (1 << OCIE4A);
  
  sei();

}

ISR(TIMER1_COMPA_vect) {
  banderaTimer = true;
}

ISR(TIMER4_COMPA_vect) {
  switch (banderaApril) {
    case 0:
      break;
    case 1:
      moveForward();
      break;
    case 2:
      turnRight();
      break;
    case 3:
      turnLeft();
      break;
  }
  banderaApril = 0;
}

void loop() {

  // 10 Hz | 100 ms
  if (banderaTimer) {
    banderaTimer = false;

    if (!banderaCross) {
      trackLine();
      trackCross();
    }

    updateVel();
  }
}

/*
-------------------------------------------------
              MOVIMIENTOS VEHÍCULO
-------------------------------------------------
*/

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
  TCNT4 = 0;  // Reinicia el timer4
}

void turnRight() {
  servo(45);
  digitalWrite(pinMotor, HIGH);
  // AGREGAR tiempo 
  servo(0);
  banderaCross = false;
}

void turnLeft() {
  servo(-45);
  digitalWrite(pinMotor, HIGH);
  // AGREGAR tiempo 
  servo(0);
  banderaCross = false;
}

void servo(int angle) {
  // Calcula el duty cicle dependiendo de la entrada angle y manda el pwm
  analogWrite(pinServo, (int)map(angle,-90,90,12,26));
}

void updateVel() {
  // Actualiza la velocidad del motor main y lo manda al pin enMotor
  velocidad += dvel;
  velocidad > 255 ? velocidad = 255;
  velocidad < velMin ? velocidad = 0;
  analogWrite(enMotor, velocidad);
}

/*
-------------------------------------------------
              FUNCIONES DETECCIÓN
-------------------------------------------------
*/

void trackLine() {

  huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING);

  if (huskylens.requestArrows()) {
  
    HUSKYLENSResult result = huskylens.read();

    int32_t error;
    float angle = (180 / PI) * atan(((float)result.xTarget - (float)result.xOrigin) / ((float)result.yOrigin - (float)result.yTarget));

    //error = (int32_t)angle - (int32_t)160;
    error = (int32_t)angle;
    controlServo.update(error);

    //Serial.println(angle);
    angle_pid = controlServo.m_command;
    servo(angle_pid);
  }
}

void trackCross() {

  huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING);

  if (huskylens.request(crossID)) {
  
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
        // AGREGAR tiempo para que pase el cross
      }
    }
  }
}

void trackApril() {
  
  huskylens.writeAlgorithm(ALGORITHM_TAG_RECOGNITION);
  HUSKYLENSResult result = huskylens.read();

  // AGREGAR condición por si detecta más de una tag
  banderaApril = result.ID;
  // AGREGAR loop por si le toma tiempo leer la tag
  // AGREAGAR condición de si no encuentra tag sea el valor de forward
  
}
