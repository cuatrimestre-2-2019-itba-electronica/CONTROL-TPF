#include <NewPing.h>



#define TRIGGER_PIN  12
#define ECHO_PIN     11
#define MAX_DISTANCE 200
#define PWM_PIN 9

/* ELIMINACION DELTAS EN LA DERIVADA */
#define DERIVATIVE_MEASUREMENT //Si esta definido, no se considera el efecto de modificar el setPoint \
                                  para calcular el termino de la derivada. Esto elimina el efecto \
                                  "derivative Kick" ante un escalon del setPoint

/* PERIODO SAMPLEO (ms) */
#define Ts 50.0

/* CONSTANTES PID */
#define _Kp 0.65
#define _Ki 0.5
#define _Kd 0.1

#define Kp _Kp
#define Ki _Ki * Ts / 1e3
#define Kd _Kd / Ts / 1e3

/* LIMITES LECTURA SENSOR */
#define SENSOR_READ_MIN 12
#define SENSOR_READ_MAX 86

/* LIMITES VELOCIDAD GIRO HELICE */
//Duty de PWM para lograr las velocidades
#define MOTOR_SPEED_MIN 150
#define MOTOR_SPEED_MAX 250

/* PARA CONTROL DE DELAY */
unsigned long lastReadyTime;

/* ENTRADA DE LA PLANTA */
unsigned int prevPIDoutput = 0;

/*
   iAmReady:
   Devuelve true:
   1) La primera vez que se llama.
   2) Si sucedieron mas de Ts milisegundos desde la ultima vez que devolvio true.
*/
bool iAmReady();

signed int PID_cycle(int setPoint, int sensorRead);
int setPoint=40;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
  Serial.begin(9600);
  pinMode(PWM_PIN, OUTPUT);

  //Inicializacion motor helice
  analogWrite(PWM_PIN, 40);
  delay(1000);
  analogWrite(PWM_PIN, 100);
  delay(1000);
  analogWrite(PWM_PIN, 150);
  delay(1000);
}

void loop() 
{
  String uartData;

  //OBTENER SET POINT
  if(Serial.available()){
    uartData = Serial.readStringUntil('\n');
    setPoint=uartData.toInt();
    Serial.print("Set point RX: ");
    Serial.print(setPoint);
    Serial.println("CM");
    if(setPoint==0 || setPoint>86){//apagar el sistema
      analogWrite(PWM_PIN, 100);
      Serial.println("Motor apagado");
      while(1);
    }
  }
  //ESPERAR QUE HAYAN SUCEDIDO TS MILISEGUNDOS
  while(!iAmReady()){}

  //OBTENER SALIDA DE LA PLANTA
  int sensorRead = sonar.ping_cm();

  //CALCULAR LA ENTRADA A LA PLANTA Y MANDAR POR PWM 
  analogWrite(PWM_PIN, PID_cycle(setPoint, sensorRead));
  
  Serial.println(sensorRead);
}


//todo: inicializacion.
signed int PID_cycle(int setPoint, int sensorRead)
{
  static int prevSensorRead = 0;
  static int prevError = 0;
  static long int integralTerm = 0;
  int proportionalTerm;
  int derivativeTerm;
  
  //Si la lectura del sensor esta fuera de rango, usar la anterior
  if ( sensorRead < SENSOR_READ_MIN || sensorRead > SENSOR_READ_MAX ) { sensorRead = prevSensorRead; }
  else { prevSensorRead = sensorRead; }

  /******************************/
  /* Calculo de los 3 terminos: */
  /******************************/
  //(P)
  proportionalTerm = setPoint - sensorRead;
  //(D)
#ifdef DERIVATIVE_MEASUREMENT
  derivativeTerm = -sensorRead - (-prevSensorRead); //no considera setPoint para evitar "Derivative Kick"
#else
  derivativeTerm = error - prevError;
#endif
  //(I)
  integralTerm += error;
  /******************************/
  
  prevError = error;
   
  int output = Kp * proportionalTerm + Ki * integralTerm + Kd * derivativeTerm;
  if(output < MOTOR_SPEED_MIN) return MOTOR_SPEED_MIN;
  if(output > MOTOR_SPEED_MAX) return MOTOR_SPEED_MAX;
  return output;
  
}

bool iAmReady()
{
  static bool firstTime = true;
  if (firstTime) //Primera vez que se llama
  {
    lastReadyTime = millis();
    firstTime = false;
    return true;  
  }
  unsigned long currentTime = millis();
  if(currentTime >= lastReadyTime + Ts) //Sucedieron mas de Ts milisegundos desde la ultima vez que devolvio true
  {
    lastReadyTime = currentTime;
    return true;  
  }
  return false;
}