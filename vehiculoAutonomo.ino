volatile bool banderaTimer = false;   // La bandera del timer, se activará cada 16ms
volatile int counterHusky = 0;        // Se contará cada que el timer interrupta
volatile int counterStop = 0;         //  ''

const int comparadorHusky = 100 / 16; // La cantidad de cuentas hasta que se active la detección de la husky. Cada 100ms
const int comparadorStop = 3000 / 16; // La cantidad de cuentas que dura una parada. Cada 3s

volatile bool banderaHusky = false;     // Va a cambiar cada que se active la detección. Debe tener periodo de 100ms
volatile bool banderaStop = false;      // Va a cambiar cada que se complete una parada. Debe tener periodo de 6s
volatile bool banderaPrograma = false;  // Muestra el tiempo que toma en ejecutarce el programa

void setup() {
  
  pinMode(5, OUTPUT);   // banderaPrograma  
  pinMode(4, OUTPUT);   // varios

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
}

ISR(TIMER2_COMPA_vect) {
  banderaTimer = true;
}

void loop() {
  
  if (banderaTimer) {
    
    banderaPrograma = !banderaPrograma;
    digitalWrite(5, banderaPrograma);
    // digitalWrite(4, HIGH);

    banderaTimer = false;
    counterHusky += 1;
    counterStop += 1;

    if (counterStop > comparador){
      counterStop = 0;
      banderaStop = !banderaStop;
      // digitalWrite(4, banderaStop);
      // Ejecución parada
    }
    else if (counterHusky > comparadorHusky) {
      counterHusky = 0;
      banderaHusky = !banderaHusky;
      // digitalWrite(4, banderaHusky);
      // Ejecución lectura huskylens
    }

    // digitalWrite(4, LOW);
  }
}
