#include <Arduino.h>
#include <ESP32Encoder.h>

#define RADS2RPM 9.5492965964254  // Conversione rad/s -> rpm

/*
    Riduzione:
    - vite senza fine: 20:1
    - ingranaggi: 5:1

    Riduzione totale = 100:1
    1 giro girarrosto = 20 giri motore = 100 giri encoder
*/
#define ENCODER2GIRARROSTO 100

// Encoder per velocita` del motore
#define ENCA 19     // Pin A dell'encoder
#define ENCB 18     // Pin B dell'encoder
#define PULSES_PER_ROTATION 200   // 200 impulsi per ogni rotazione
const float RADIANS_PER_PULSE = 2*PI / PULSES_PER_ROTATION;     // A quanti radianti corrisponde un pulse dell'encoder
ESP32Encoder Enc;   // Oggetto dell'encoder NEMA17
float speed;

// Variabili temporali
unsigned long deltaT;       // Intervallo di campionamento
unsigned long t=0, oldT=0;  // Per calcolare l'intervallo
unsigned long period = 100;    // Periodo in microsecondi

// Motore con driver L298n
#define EN 13       // Pin su cui mandare il segnale pwm
#define IN1 12      // Pin per la direzione 
#define IN2 14      // Pin per la direzione
#define CW 1        // Senso orario
#define CCW 0       // Senso antiorario

// Parametri per PWM : https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
#define CHANNEL 0           // Canale per la funzione ledc
#define FREQUENCY 20000     // Frequenza dell'onda quadra
#define RESOLUTION 11       // Bit di risoluzione del dutycycle => livelli = 2^RESOLUTION
const int maxDutyCycle = pow(2, RESOLUTION) -1;     // Massimo valore passabile come dutyCycle
float dutyCycle = 0.5;          

// Encoder di controllo
#define ENC2A 27
#define ENC2B 26
#define ENC2SWITCH 25           // Quando premo sulla manopola
ESP32Encoder knob;              // Oggetto per gestire la manopola
volatile bool rotate = false;   // indica se il motore puo` girare 
void IRAM_ATTR startStop();     // Fa partire/fermare il motore
volatile unsigned long knobT=0; // Debounce 

// Controllo in velocita`
float target = 0, memTarget = 1;               // rpm del girarrosto desiderati
float error = 0;
float kp = 0.01;
float ki = 0.5;
float gainP, gainI;

// Dichiarazioni delle funzioni
void setDirection(int);
void setDutyCycle(float);
void getSpeed(unsigned long);



// ================================ SETUP ========================================
void setup() {
    // Comunicazione seriale
    Serial.begin(115200);

    // Inizializzazione Encoder di controllo
    pinMode(ENC2A, INPUT);
    pinMode(ENC2B, INPUT);
    pinMode(ENC2SWITCH, INPUT);
    Serial.printf("# Controller: CLK: %d, DT: %d, SW: %d\n", ENC2A, ENC2B, ENC2SWITCH);
    attachInterrupt(ENC2SWITCH, startStop, FALLING);

    // Inizializzazione ledc channel
    ledcSetup(CHANNEL, FREQUENCY, RESOLUTION);
    ledcAttachPin(EN, CHANNEL);
    Serial.printf("# PWM motor signal: EN: %d\n", EN);

    // Inizializzazione pin del motore per la direzione
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    setDirection(CW);
    Serial.printf("# Motor direction pins: IN3: %d, IN4: %d\n", IN1, IN2);

    // Inizializzazione dell'encoder
    Enc.attachFullQuad(ENCA, ENCB);
    Serial.printf("# Encoder: pinA: %d, pinB: %d", ENCA, ENCB);

    // Inizializzo il motore impostando velocita` 0
    setDutyCycle(0);

    // Inizializzo la variabile t
    t = micros();
    Serial.println("Dutycycle: %.2f, Target: %.2f, Speed: %.2f\n");
}



// ================================ LOOP ========================================
void loop() {

    // Calcolo del periodo deltaT
    oldT = t;
    t = micros();
    deltaT = t - oldT;

    // Misuro la velocita`
    getSpeed(deltaT);

    // Calcolo l'errore
    error = target - speed;

    // Calcolo le varie componenti del PID

        // Proporzionale
        gainP = error * kp;

        // Integrale
        gainI += ki * error * (float)deltaT/1000000.0;

    // Calcolo il dutycycle da applicare
    dutyCycle = gainP + gainI;

    // Controllo la saturazione
    if (dutyCycle > 1) dutyCycle = 1;
    if (target == 0 && speed == 0) dutyCycle = 0;     // Per risparmiare energia

    // Imposto il dutycycle
    setDutyCycle(dutyCycle);

    // Logging
    Serial.printf("%.2f, %.2f, %.2f\n", dutyCycle, target, speed);

    // Aspetto
    delay(period);
}



// ================================ ALTRO ========================================

// Imposto la direzione desiderata
void setDirection(const int newDir) {
    if (newDir == CW) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    }
    else if (newDir == CCW) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }
    else {
        perror("Direzione indicata non valida");
        exit(1);
    }
}

// Imposto il valore di dutyCycle per il motore
void setDutyCycle(float dutyCycle) {
    // Accetto anche un argomento negativo: il segno indica la direzione
    if (dutyCycle < 0) {
        setDirection(CCW);
        dutyCycle *= -1;
    }
    else {
        setDirection(CW);
    }


    // Imposto il dutyCycle nel canale PWM
    ledcWrite(CHANNEL, (int)(dutyCycle*maxDutyCycle));
}

// Fa partire/fermare il motore
void IRAM_ATTR startStop() {
    if( millis() - knobT > 300) {
        // Cambio il flag
        rotate = !rotate;

        // Se mi fermo metto a 0, altrimenti rimetto il dutyCycle di prima
        if (rotate) {
            target = memTarget;
        }
        else {
            memTarget = target;
            target = 0;
        }

        // Aggiorno il timer per il debounce
        knobT = millis();
    }
}

void getSpeed(unsigned long deltaT) {
    // Conto quanti impulsi sono stati registrati
    int pulseCounter = Enc.getCount();

    //Serial.printf("\t%d, %lu, %.2f\n", pulseCounter, deltaT, (float)deltaT/1000000.0);

    // Calcolo la velocita`, prima in pulse/s, poi in rad/s, poi in rpm dell'encoder, poi in rpm del carico
    speed = pulseCounter / ((float)deltaT/1000000.0) * RADIANS_PER_PULSE * RADS2RPM / ENCODER2GIRARROSTO;

    // Azzero il contatore dell'encoder
    Enc.clearCount();
}