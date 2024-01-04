#include <QTRSensors.h>
QTRSensors qtr;


const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int enA = 1;   //PWM
int enB = 0;  //PWM
int in1 = 2;
int in2 = 3;
int in3 = 4;
int in4 = 5;
int P, I, D, lasterror = 0, error;
float Kp = 0.17;
float Ki = 0.0005;
float Kd = 0.7;
float threshold[8];


void setup()
{
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){6, 7, 8, 9, 10, 11, 12, 13}, SensorCount);
  qtr.setEmitterPin(0);
  delay(250);


  //CALIBRATION MODE
  // turn on Arduino's LED to indicate we are in calibration mode
  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 
  for (uint16_t i = 0; i < 400; i++)
    qtr.calibrate();
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration



  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
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



  //motor driver
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}


void movement(int speedA, int speedB) 
{
  analogWrite(enA, speedA);
  analogWrite(enB, speedB);
  if (speedA <= 0) {
    speedA = 0 - speedA;
    digitalWrite(in2, HIGH);
    digitalWrite(in1, LOW);
  } else {
    digitalWrite(in2, LOW);
    digitalWrite(in1, HIGH);
  }
  if (speedB <= 0) {
    speedB = 0 - speedB;
    digitalWrite(in4, HIGH);
    digitalWrite(in3, LOW);
  } else {
    digitalWrite(in4, LOW);
    digitalWrite(in3, HIGH);
  }
}


void PID_control() {
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);
  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  error = position - 2500;
  // error=0 no correction
  // error>3500 near to sensor 7
  // error<3500 near to sensor 1
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
  delay(1000);
  P = error;
  I = error + I;
  D = lasterror - error;
  lasterror = error;
  int motorSpeedChange = P * Kp + I * Ki + D * Kd;
  int speedA = 75 + motorSpeedChange;
  int speedB = 75 - motorSpeedChange;
  if (speedA > 150)
    speedA = 100;
  if (speedB > 150)
    speedB = 100;
  if (speedA < -100)
    speedA = -100;
  if (speedB < -100)
    speedB = -100;
  movement(speedA, speedB);
}



void loop() {
  for (int i = 0; i < SensorCount; i++) {
    threshold[i] = (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i]) / 2;
    Serial.println(i);
    Serial.print("=");
    Serial.print(threshold[i]);
  }
  if (analogRead(6) > threshold[6] && analogRead(13) < threshold[13])  //sensor1-black sensor2-white
  {
    movement(0, 125);
  } 
  else if (analogRead(6) > threshold[6] && analogRead(13) < threshold[13])  //sensor1-white sensor2-black
  {
    movement(125, 0);
  } 
  else
      PID_control();
}