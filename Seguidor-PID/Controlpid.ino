#include <QTRSensors.h>
#define NUM_SENSORS 8            // number of sensors used
#define NUM_SAMPLES_PER_SENSOR 4 // average 4 analog samples per sensor reading
#define EMITTER_PIN 2            // emitter is controlled by digital pin 2
#define PWMA 3
#define AIN2 4
#define AIN1 5
#define STBY 6
#define BIN1 7
#define BIN2 8
#define PWMB 9
QTRSensorsAnalog qtra((unsigned char[]){
                          0, 1, 2, 3, 4, 5, 6, 7},
                      NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
//Velocidad de los frenos
int VM1 = 150;
int VM2 = 50;
/////////////////
int position = 0;
int salida_pwm = 0;
int velocidad = 0;
int derivativo = 0;
int proporcional = 0;
int error = 0;
int error_pasado;
int integral = 0;
float kp = 0;
float kd = 0;
float ki = 0;

int cruzero = 80;
float vel;
int P = 0;
int I = 0;
int D = 0;
int LAST = 0;

unsigned long temp;

void setup()
{
    pinMode(PWMA, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    digitalWrite(STBY, HIGH);
    for (int i = 0; i < 400; i++) // make the calibration take about 10 seconds
    {
        qtra.calibrate(); // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
    }
}

void loop()
{
    //temp = millis();
    //while (millis() < temp + 10000) {
    /*velocidad = 110;
      kp = 0.5;
      kd = 0;
      ki = 0;

      pid(velocidad, kp, ki, kd);
      frenos_contorno(600);
      //adelanted();*/
    position = qtra.readLine(sensorValues, QTR_EMITTERS_ON, 0);

    P = ((position) - (3500)); /// ERROR
    /////FRENOS////
    if (P < -3500)
    {
        analogWrite(PWMA, VM1);   // VELOCIDAD PARA EL MOTOR DERECHO
        analogWrite(PWMB, VM2);   //  VELOCIDAD PARA EL MOTOR IZQUIERDO
        digitalWrite(AIN1, HIGH); ///FRENTE
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, HIGH); ///RETROCEDE
        digitalWrite(BIN2, LOW);
    }
    else if (P > 3500)
    {
        analogWrite(PWMA, VM2);  // VELOCIDAD PARA EL MOTOR DERECHO
        analogWrite(PWMB, VM1);  //  VELOCIDAD PARA EL MOTOR IZQUIERDO
        digitalWrite(AIN1, LOW); ///RETROCEDE
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN1, LOW); ///FRENTE
        digitalWrite(BIN2, HIGH);
    }
    /////////////////////////
    else
    {
        D = (P - LAST); /// ERROR MENOS EL ERROR ANTERIOR , DERIVATIVO
        I = (P + LAST); //INTEGRAL

        vel = (P * 0.05) + (D * 0) + (I * 0)
              //vel=(P*0.025)+(D*0.095)+(I*0); // PID
              //vel = (P * 0.05) + (D * 0.025) + (I * 0.0065); // para velocidad 120//////estaba en 0.0925
              //vel=(P*0.0428)+(D*0.085)+(I*0); //para velocidad 80 kd=0.06

              ///CRUZERO =VELOCIDAD PUNTA , V

              if (vel > cruzero) vel = cruzero;
        if (vel < -cruzero)
            vel = -cruzero;

        analogWrite(PWMA, cruzero - vel); // VELOCIDAD PARA EL MOTOR DERECHO
        analogWrite(PWMB, cruzero + vel); //  VELOCIDAD PARA EL MOTOR IZQUIERDO

        digitalWrite(AIN1, HIGH); ///FRENTE
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, LOW); ///FRENTE
        digitalWrite(BIN2, HIGH);

        LAST = P;
    }
}
//digitalWrite(STBY, LOW);
//}
/*
  void pid(int velocidad, float Kp, float Ki, float Kd) {
  position = qtra.readLine(sensorValues, QTR_EMITTERS_ON, 0);
  error = (position) - 7
  3

  68;
  proporcional = error;
  integral = integral + error_pasado;
  derivativo = error - error_pasado;
  if (integral > 700) integral = 700;
  if (integral < -700) integral = -700;
  salida_pwm = (proporcional * Kp) + (derivativo * Kd) + (integral * Ki);

  if (salida_pwm > velocidad) salida_pwm = velocidad;
  if (salida_pwm < -velocidad) salida_pwm = -velocidad;

  if (salida_pwm < 0) {
    motores(velocidad + salida_pwm, velocidad);
  }
  if (salida_pwm > 0) {
    motores(velocidad, velocidad - salida_pwm);
  }
  error_pasado = error;
  }

  void motores(int motor_izq, int motor_der) {
  if (motor_izq >= 0)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, motor_izq);
  }
  else
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    motor_izq = motor_izq * (-1);
    analogWrite(PWMA, motor_izq);
  }

  if (motor_der >= 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, motor_der);
  }
  else
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    motor_der = motor_der * (-1);
    analogWrite(PWMA, motor_der);
  }
  }

  void frenos_contorno(int flanco_comparacion) {
  if (position <= 0) {
    motores(-30, 100);
    while (true)
    {
      qtra.read(sensorValues);
      if (sensorValues[0] > flanco_comparacion || sensorValues[1] > flanco_comparacion)
      {
        break;
      }
    }
  }
  if (position >= 7000) {
    motores(100, -30);
    while (true) {
      qtra.read(sensorValues);
      if (sensorValues[7] > flanco_comparacion || sensorValues[6] > flanco_comparacion) {
        break;
      }
    }
  }
  }

  void adelanted() {
  digitalWrite(AIN1, 0);
  digitalWrite(AIN2, 1);
  analogWrite(PWMA, 80);
  digitalWrite(BIN1, 0);
  digitalWrite(BIN2, 1);
  analogWrite(PWMB, 80);
  }
*/
