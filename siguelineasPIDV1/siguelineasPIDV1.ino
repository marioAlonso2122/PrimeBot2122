#include <QTRSensors.h>
#define MaxSpeed 255 //Velocidad Maxima Motores
#define MinSpeed 0 //Velocidad minima Motores

// Motor driver
const int AIN1 = 9; 
const int AIN2 = 10;
const int PWMA = 11; 
const int BIN1 = 12; 
const int BIN2 = 13;
const int PWMB = 8;
const int STBY = 7;

// QTR Sensors
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


// PID constants
const float Kp = 0;   // Constante Proporcional
const float Ki = 0;  // Constante Integral
const float Kd = 1;  // Constante Derivativa
int lastError = 0;      // Error anterior
int integral = 0;       // Acumulación de errores

void setup() {

  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  qtr.setEmitterPin(2);


  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);


  //Serial.begin(9600);
  //Serial.println("Calibrando motor 1");
  //digitalWrite(BIN1, HIGH);
  //digitalWrite(BIN2, LOW);
  //analogWrite(PWMB, MaxSpeed);
  //delay(1000);

  //Serial.println("Calibrando motor 2");
  //digitalWrite(AIN1, HIGH);
  //digitalWrite(AIN2, LOW);
  //analogWrite(PWMA, MaxSpeed);
  //delay(1000);
  
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop() {
  int position = qtr.readLineBlack(sensorValues);
  Serial.println("Posicion:");
  Serial.println(position);
  
  int error = position - 2500;  // Suponemos que 3500 es el centro

  // Calcula la derivativa (diferencia entre el error actual y el error anterior)
  int derivative = error - lastError;

  // Acumula el error para la integral
  integral += error;

  // Calcula el ajuste del PID
  int motorSpeedAdjustment = Kp * error + Kd * derivative;
  int baseSpeed = 70;  // Velocidad base de los motores
  int leftSpeed = baseSpeed - motorSpeedAdjustment;
  int rightSpeed = baseSpeed + motorSpeedAdjustment;

  //Comprobacion de Velocidades
  if (rightSpeed > MaxSpeed ) rightSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (leftSpeed > MaxSpeed ) leftSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (rightSpeed < 0) rightSpeed = 0;    
  if (leftSpeed < 0)leftSpeed = 0;

  setMotorSpeeds(leftSpeed, rightSpeed);

  

  Serial.println("Vder:");
  Serial.println(rightSpeed);
  Serial.println("Vizq:");
  Serial.println(leftSpeed);
  Serial.println("Correcion:");
  Serial.println(motorSpeedAdjustment);
  delay(10);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(PWMA, leftSpeed);

  if (rightSpeed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(PWMB, rightSpeed);
}
