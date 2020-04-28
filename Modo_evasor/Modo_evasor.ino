#include <Servo.h>
#define In1 2
#define In2 4
#define Pwm1 3
#define In3 6
#define In4 7
#define Pwm2 5
#define B_TX 8
#define B_RX 9
#define Echo 11
#define Trigger 10
#define Servo_pin 13

#define S1 A0
#define S2 A1
#define S3 A2
#define S4 A3
#define S5 12


int Menu;
//Movimientos basicos
void Motores_init();
void Motores_mv(int velocidad_izquierda, int velocidad_derecha);

//Para evasor de obstaculos
Servo servo_1;
void evasor_init();
float Obtener_Distancia();

// Para seguidor de linea
bool sensores[5];
void seguidor_init();
void leer_sensores();
void print_sensores();

void setup()
{
  Serial.begin(9600);
  evasor_init();
  Motores_init();
  
}

void loop() 
{
    Serial.println(Obtener_Distancia());
    delay(500);
    Motores_mv(50, 60);
}
void Motores_mv(int velocidad_izquierda, int velocidad_derecha){
    if(velocidad_izquierda>0)
    {
        analogWrite(Pwm1, velocidad_izquierda);
        digitalWrite(In1, LOW);
        digitalWrite(In2, HIGH);

    }
    else
    {
        velocidad_izquierda = -1 * velocidad_izquierda;
        analogWrite(Pwm1, velocidad_izquierda);
        digitalWrite(In1, HIGH);
        digitalWrite(In2, LOW);
    }
    if (velocidad_derecha > 0)
    {
        analogWrite(Pwm2, velocidad_derecha);
        digitalWrite(In3, LOW);
        digitalWrite(In4, HIGH);
    }
    else
    {
        velocidad_derecha = -1 * velocidad_derecha;
        analogWrite(Pwm2, velocidad_derecha);
        digitalWrite(In3, HIGH);
        digitalWrite(In4, LOW);
    }
}

void Motores_init(){
    pinMode(In1, OUTPUT);
    pinMode(In2, OUTPUT);
    pinMode(Pwm1, OUTPUT);
}
void evasor_init(){
    pinMode(Trigger, OUTPUT);
    pinMode(Echo, INPUT);
    servo_1.attach(Servo_pin);
    servo_1.write(90);
}

float Obtener_Distancia(){
    long lduration;
    long ldistance;

    digitalWrite(Trigger, LOW);
    delayMicroseconds(2);
    digitalWrite(Trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trigger, LOW);
    lduration = pulseIn(Echo, HIGH);
    ldistance = (lduration / 2) / 29.1;
    return ldistance;
}

void leer_sensores(){
    sensores[0] = digitalRead(S1);
    sensores[1] = digitalRead(S2);
    sensores[2] = digitalRead(S3);
    sensores[3] = digitalRead(S4);
    sensores[4] = digitalRead(S5);
}
void print_sensores(){
    for (size_t i = 0; i < 5; i++)
    {
        Serial.print(sensores[i]);
        Serial.print("\t");
        /* code */
    }
    Serial.println("");
}

void seguidor_init(){
    pinMode(S1, INPUT_PULLUP);
    pinMode(S2, INPUT_PULLUP);
    pinMode(S3, INPUT_PULLUP);
    pinMode(S4, INPUT_PULLUP);
    pinMode(S5, INPUT_PULLUP);
}