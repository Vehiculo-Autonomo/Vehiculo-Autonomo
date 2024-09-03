#include "HUSKYLENS.h"                         
#include "SoftwareSerial.h"
#include "PIDLoop.h"
#include <Servo.h>

#define pinMotor 10
#define pinDireccion 11

int angle_pid = 0;
Servo servo;

PIDLoop headingLoop(1, 0, 0, true);      // Controlador PID para el controlador (por el momento lo dejaremos como P)          
HUSKYLENS huskylens;    // Crea un objeto con el cual reconoceremos a la husky                     
int ID1 = 1;            // El id de línea. Este se determina al momento de enseñar a la husky

void printResult(HUSKYLENSResult result);

long tiempo = 0;

void setup() {
  Serial.begin(115200);                                         //Start serial communication
  Wire.begin();                                                 //Begin communication with the Huskeylens
  while (!huskylens.begin(Wire)) {
      Serial.println(F("Begin failed!"));
      Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protol Type>>I2C)"));
      Serial.println(F("2.Please recheck the connection."));
      delay(100);
  }

  pinMode(pinMotor, OUTPUT);
  servo.attach(pinDireccion);
  
  huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING);            //Switch the algorithm to line tracking.
  moveForward();                                                //Start the motors
}

void loop() {

  tiempo = micros();
  if (!huskylens.request(ID1)) {                                 //If a connection is not established
    Serial.println(F("Check connection to Huskeylens"));
    //leftSpeed = 0; 
    //rightSpeed = 0;
  }
  else if(!huskylens.isLearned()) {                              //If an object has not been learned
    Serial.println(F("No object has been learned"));
    //leftSpeed = 0; 
    //rightSpeed = 0;
  }
  else if(!huskylens.available()) {                             //If there is no arrow being shown on the screen yet
    Serial.println(F("No arrow on the screen yet"));
  }

  // Si no hay error y se detecta una línea se mostrarán los resultados y ajustará la dirección
  else {                                                        //Once a line is detected and an arrow shown
    HUSKYLENSResult result = huskylens.read();                  //Read and display the result
    printResult(result);

    turn(result);
    
  }
  //Serial.print("Count learned: ");
  //Serial.println(huskylens.countLearnedIDs());
  //Serial.print("Count detected: ");
  //Serial.println(huskylens.countArrows());

  Serial.print("tiempo: ");
  Serial.println(micros() - tiempo);
  
}

// Por el momento estas funciones las mantendremos lo más genérico posible
void moveForward() {
  digitalWrite(pinMotor, HIGH);
}

void stopMove() {
  digitalWrite(pinMotor, LOW);
}

// Gira el servo lo suficiente para mantener el seguimiento de línea
void turn(HUSKYLENSResult result) {

  int32_t error;
  float angle = (180/PI)*atan(((float)result.xTarget - (float)result.xOrigin) / ((float)result.yOrigin - (float)result.yTarget));

  //error = (int32_t)angle - (int32_t)160;            
  error = (int32_t)angle;
  headingLoop.update(error);                               

  Serial.println(angle);
  angle_pid = headingLoop.m_command;
  servo.write(angle_pid);

}

// Imprime los objetos encontrados y los imprime en el serial
void printResult(HUSKYLENSResult result) {
    if (result.command == COMMAND_RETURN_BLOCK) {
        Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
    }
    else if (result.command == COMMAND_RETURN_ARROW) {
        Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
    }
    else {
        Serial.println("Object unknown!");
    }
}
