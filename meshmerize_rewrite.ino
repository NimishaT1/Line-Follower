#include <SparkFun_TB6612.h>

#define HOMING_THRESHOLD 500
#define IR 5 //number of IR sensors
#define BaseSpeed 95 //base speed of motors
#define CalSpeed 40 // calibration speed of motors
#define FwdSpeed 40 //forward speed of motors
#define RevSpeed 40 //backward speed of motors
#define turnSpeed 60

#define KS 1000 // sensor factor used for calibration
#define overlap 150 //line centering post calibration
#define sens 550


#define AIN1 3
#define BIN1 5
#define AIN2 2
#define BIN2 4
#define PWMA 11
#define PWMB 10
#define STBY 9

#define IRR 8
#define IRL 6

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

float Kp = 0.022; //0.0015       ---- 6.40V, 11.61V
float Kd = 0.05;//0.04       ---- base speed 50, turn speed 45 others 40
float Ki = 0.002;

// for 40 speed :Kp = 0.015; for 60 speed: 0.035
//Kd = 0.3; 0.7
//Ki = 0.005; 0.001
long P = 0, I = 0, D = 0, PIDvalue = 0, PrevError = 0;

//sensor PINS and Sensors


const byte SensorPin[IR] = {A0,A1,A2,A3,A4}; //sensor pins array
int Sensors[IR]; //array that stores sensor values
int MinSensors[IR]; //array that stores minimum values of sensors
int MaxSensors[IR]; //array that stores maximum values of sensors

// Sensor Position
unsigned long positionX = 0; //position of the line w.r.t. sensors
unsigned long positionM = ((IR*KS)-KS)/2; //middle position of the line
int positionH = 0; //horizontal offset from the line

//line detection
bool lineDetected = false;

//Motor Speeds;
int speedLeft = 0;
int speedRight = 0;

//Switch Pins
int StartSwitch = 9;
int switchState = 0;

//Led Indicator
int TPin = 7;

int sL = 0, sR = 0;

bool leftPath=0,rightPath=0,straightPath=0;


void calibrate(void);
void getSensorValues(void);
void calculatePID(float Kp = Kp, float Kd = Kd, float Ki = Ki);
void dryRun(void);
void check_switch(void);

void setup(){
    Serial.begin(500000);
    delay(5000); //wait for a while before doing calibration

    pinMode(StartSwitch, INPUT_PULLUP);
    pinMode(TPin, OUTPUT);

    Serial.println("Calibration Started");
    calibrate();
    brake(motor1, motor2);
    Serial.println("Calibration Complete");

    digitalWrite(TPin,1);

    homeToLine();

    digitalWrite(TPin,0);

    delay(500);
}

void loop(){
    check_switch();

    getSensorValues();

    calculatePID();

    if(switchState){
        dryRun();
    } else {

    }
}

void calibrate(void){
//initialise sensor readings, min and max values
  for (int i=0; i<IR;i++){
    Sensors[i] = analogRead(SensorPin[i]);
    MinSensors[i] = Sensors[i];
    MaxSensors[i] = Sensors[i];
  }
  // calibrate sensors

  double lastTime = millis();
  bool ledOn = true;

  if(millis()-lastTime > 250){
    ledOn = !ledOn;
    digitalWrite(LED_BUILTIN, ledOn);
    lastTime = millis();
  }

  for(int i = 0; i<7000;i++){
    for (int j = 0; j<IR; j++){
      Sensors[j] = analogRead(SensorPin[j]);
      //MinSensors[j] = min(MinSensors[j], Sensors[j]);
      if (Sensors[j] < MinSensors[j]) MinSensors[j] = Sensors[j];
      if (Sensors[j] > MaxSensors[j]) MaxSensors[j] = Sensors[j];
    }

    motor1.drive(CalSpeed); //Drive left wheel forward
    motor2.drive(-CalSpeed); //Drive right wheel backward
  }
}

void getSensorValues(){
  lineDetected = false;
  bool straightPathDetected = true;
  unsigned long weightedSum = 0;
  unsigned int sum = 0;

  for (int i = 0; i<IR; i++){
    Sensors[i] = analogRead(SensorPin[i]);
    Sensors[i] = map(Sensors[i], MinSensors[i], MaxSensors[i], KS, 0); //print this out during testing to see what kinda values we get (high vals for black)
    Sensors[i] = constrain(Sensors[i], 0, KS);
    if (Sensors[i]>=sens) lineDetected = true;
    if (Sensors[i]<sens) straightPathDetected = false;
    if (Sensors[i] >= 50){
      weightedSum += long(Sensors[i])*(i*KS);
      sum += Sensors[i];
    }

    // if(Sensors[0] >= sens){
    //   digitalWrite(TPinLeft, 1);
    // } else {
    //   digitalWrite(TPinLeft, 0);
    // }

    // if(Sensors[4] >= sens){
    //   digitalWrite(TPinRight, 1);
    // } else {
    //   digitalWrite(TPinRight, 0);
    // }
  }

  leftPath = digitalRead(IRL);
  rightPath = digitalRead(IRR);
  straightPath = straightPathDetected;

  if (lineDetected) positionX = weightedSum/sum; // stores the weighted mean of the index of the ir sensors
  else if (positionX < positionM) positionX = 0;
  else positionX = positionM*2; //basically 4 lol
  positionH = positionX - positionM; // testing ke liye this will be used otherwise it is pointless

  if (lineDetected) {
    digitalWrite(TPin, 1);
  } else {
    digitalWrite(TPin, 0);
  }

}

void calculatePID(float Kp = Kp, float Kd = Kd, float Ki = Ki){
  P = positionX - positionM;
  I = P + PrevError; // OG was I = P + PrevError
  D = P - PrevError;
  PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
  PrevError = P;
}

void dryRun(){

    if(rightPath){
      turnRight();
    } else if (straightPath){
      return;
    } else if (leftPath) {
      turnLeft();
    }

    if(lineDetected){
        speedLeft = BaseSpeed + PIDvalue;
        speedRight = BaseSpeed - PIDvalue;
        speedLeft = constrain(speedLeft, 0, 255);
        speedRight = constrain(speedRight, 0, 255);
        
        sL = speedLeft;
        sR = speedRight;
        motor1.drive(speedLeft);
        motor2.drive(speedRight);
    } else {
        if (P == -positionM){
            speedLeft = FwdSpeed;
            speedRight = RevSpeed;
            motor1.drive(-speedLeft);
            motor2.drive(speedRight);
        } else if (P == positionM){
            speedLeft = RevSpeed;
            speedRight = FwdSpeed;
            motor1.drive(speedLeft);
            motor2.drive(-speedRight);
        }
    }

}

void turnRight(){
  brake(motor1, motor2);
  while(rightPath || !lineDetected){
    motor2.drive(turnSpeed/2);
    motor1.drive(turnSpeed*2);
    getSensorValues();
  }
}

void turnLeft(){
  brake(motor1, motor2);
  while(leftPath || !lineDetected){
    motor1.drive(turnSpeed/2);
    motor2.drive(turnSpeed*2);
    getSensorValues();
  }
}

void homeToLine(){

    getSensorValues();
    calculatePID();

    digitalWrite(TPin,1);

    while(abs(P) < overlap){
        if (P<-overlap){
            speedLeft = -PIDvalue;
            speedRight = PIDvalue;
            sL = speedLeft;
            sR = speedRight;
            motor1.drive(speedLeft);
            motor2.drive(speedRight);
        } else if (P > overlap){
            speedLeft = PIDvalue;
            speedRight = -PIDvalue;
            sL = speedLeft;
            sR = speedRight;
            motor1.drive(speedLeft);
            motor2.drive(speedRight);
        } else {
            sL = 0;
            sR = 0;
            brake(motor1, motor2);
        }
    }

    brake(motor1, motor2);
}

void check_switch(){
  switchState = digitalRead(StartSwitch);
  if (switchState != HIGH){
    delay(2000); // avoid repeated reading
    digitalWrite(TPin, 0);
  }
}
