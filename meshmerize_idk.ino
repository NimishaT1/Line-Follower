#include <SparkFun_TB6612.h>
#include <Arduino.h>
#include <RotaryEncoder.h>

#define HOMING_THRESHOLD 500
#define IR 5         // number of IR sensors
#define BaseSpeed 150 // base speed of motors
#define CalSpeed 80  // calibration speed of motors
#define FwdSpeed 160  // forward speed of motors
#define RevSpeed 160  // backward speed of motors
#define turnSpeed 100

#define KS 1000     // sensor factor used for calibration
#define overlap 300 // line centering post calibration
#define sens 550
#define turnThreshold 1
#define turnThresholdMax 1

#define ticksPerTurn 100

#define AIN1 3
#define BIN1 13
#define AIN2 2
#define BIN2 12
#define PWMA 11
#define PWMB 10
#define STBY 9

#define IRR 8
#define IRL 6

#define LEN1 7
#define LEN2 9
#define REN1 4
#define REN2 5

#define TPin A5

char path[500];
int top = 0;

void appendToPath(char c) { path[top++] = c; }

const int offsetA = 1;
const int offsetB = -1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

RotaryEncoder rightEncoder(REN1, REN2, RotaryEncoder::LatchMode::TWO03);
RotaryEncoder leftEncoder(LEN1, LEN2, RotaryEncoder::LatchMode::TWO03);

float Kp = 0.022; // 0.0015       ---- 6.40V, 11.61V
float Kd = 0.05;  // 0.04       ---- base speed 50, turn speed 45 others 40
float Ki = 0.002;

// for 40 speed :Kp = 0.015; for 60 speed: 0.035
// Kd = 0.3; 0.7
// Ki = 0.005; 0.001
long P = 0, I = 0, D = 0, PIDvalue = 0, PrevError = 0;

// sensor PINS and Sensors

const byte SensorPin[IR] = {A0, A1, A2, A3, A4}; // sensor pins array
int Sensors[IR];    // array that stores sensor values
int MinSensors[IR]; // array that stores minimum values of sensors
int MaxSensors[IR]; // array that stores maximum values of sensors

// Sensor Position
unsigned long positionX = 0; // position of the line w.r.t. sensors
unsigned long positionM = ((IR * KS) - KS) / 2; // middle position of the line
int positionH = 0; // horizontal offset from the line

// line detection
bool lineDetected = false;

// Motor Speeds;
int speedLeft = 0;
int speedRight = 0;

// Switch Pins
int StartSwitch = A6;

int currentRound = 0;

int sL = 0, sR = 0;

int leftPathVal = 0, rightPathVal = 0;
bool leftPath = 0, rightPath = 0, straightPath = 0, deadEnd = 0;

void waitForSwitch(void);

void calibrate(void);
void geSensorValues(void);
void calculatePID(float Kp=Kp, float Kd=Kd, float Ki=Ki);

void setup(){
  /*
    SETUP
  */
  pinMode(StartSwitch, INPUT_PULLUP);
  pinMode(TPin, OUTPUT);
  Serial.begin(500000);
  /*
    SETUP END
  */

  calibrate();
  brake(motor1, motor2);
  homeToLine();
  while(true){
  waitForSwitch();
  followLine();
  }
}

void loop(){}

void waitForSwitch(){
  while(true){
    if (analogRead(StartSwitch)<10){
      break;
    }
  }
  delay(2000);
}

void calibrate(void) {
  // initialise sensor readings, min and max values
  for (int i = 0; i < IR; i++) {
    Sensors[i] = analogRead(SensorPin[i]);
    MinSensors[i] = Sensors[i];
    MaxSensors[i] = Sensors[i];
  }
  // calibrate sensors

  double lastTime = millis();
  bool ledOn = true;

  if (millis() - lastTime > 250) {
    ledOn = !ledOn;
    digitalWrite(LED_BUILTIN, ledOn);
    lastTime = millis();
  }

  for (int i = 0; i < 7000; i++) {
    for (int j = 0; j < IR; j++) {
      Sensors[j] = analogRead(SensorPin[j]);
      // MinSensors[j] = min(MinSensors[j], Sensors[j]);
      if (Sensors[j] < MinSensors[j])
        MinSensors[j] = Sensors[j];
      if (Sensors[j] > MaxSensors[j])
        MaxSensors[j] = Sensors[j];
    }

    motor1.drive(CalSpeed);  // Drive left wheel forward
    motor2.drive(-CalSpeed); // Drive right wheel backward
  }
}

void homeToLine() {

  getSensorValues();
  calculatePID();

  digitalWrite(TPin, 1);

  while (abs(P) > overlap || !lineDetected) {
    if (P < -overlap) {
      speedLeft = -CalSpeed;
      speedRight = CalSpeed;
      sL = speedLeft;
      sR = speedRight;  
      motor1.drive(speedLeft);
      motor2.drive(speedRight);
    } else if (P > overlap) {
      speedLeft = CalSpeed;
      speedRight = -CalSpeed;
      sL = speedLeft;
      sR = speedRight;
      motor1.drive(speedLeft);
      motor2.drive(speedRight);
    } else {
      sL = 0;
      sR = 0;
      brake(motor1, motor2);
    }

    getSensorValues();
    calculatePID();
  }

  brake(motor1, motor2);
}

void getSensorValues() {
  lineDetected = false;
  deadEnd = 1;
  unsigned long weightedSum = 0;
  unsigned int sum = 0;

  for (int i = 0; i < IR; i++) {
    Sensors[i] = analogRead(SensorPin[i]);
    Sensors[i] = map(Sensors[i], MinSensors[i], MaxSensors[i], KS, 0); 
    Sensors[i] = constrain(Sensors[i], 0, KS);
    if (Sensors[i] >= sens) {
      lineDetected = true;
      deadEnd = false;
    }

    if (Sensors[i] >= 50) {
      weightedSum += long(Sensors[i]) * (i * KS);
      sum += Sensors[i];
    }
  }

  if (Sensors[2] >= sens) straightPath = true;

  leftPath = digitalRead(IRL);
  rightPath = digitalRead(IRR);

  if (leftPath || rightPath) {
    deadEnd = false;
  }

  if (lineDetected)
    positionX = weightedSum/sum; // stores the weighted mean of the index of the ir sensors
  else if (positionX < positionM)
    positionX = 0;
  else
    positionX = positionM * 2; // basically 4 lol
}
void calculatePID(float Kp = Kp, float Kd = Kd, float Ki = Ki) {
  P = positionX - positionM;
  I = P + PrevError; // OG was I = P + PrevError
  D = P - PrevError;
  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  PrevError = P;
}

void followLine(){
  do{
    getSensorValues();
    calculatePID();
    if (lineDetected) {
    speedLeft = BaseSpeed + PIDvalue;
    speedRight = BaseSpeed - PIDvalue;
    speedLeft = constrain(speedLeft, 0, 255);
    speedRight = constrain(speedRight, 0, 255);

    sL = speedLeft;
    sR = speedRight;
    motor1.drive(speedLeft);
    motor2.drive(speedRight);
  } else {
    if (P == -positionM) {
      speedLeft = FwdSpeed;
      speedRight = RevSpeed;
      motor1.drive(-speedLeft);
      motor2.drive(speedRight);
    } else if (P == positionM) {
      speedLeft = RevSpeed;
      speedRight = FwdSpeed;
      motor1.drive(speedLeft);
      motor2.drive(-speedRight);
    }
  }
  delay(10);
  }while(!leftPath&&!rightPath);
  brake(motor1, motor2);
}