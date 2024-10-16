#define IR 5 //number of IR sensors
#define BaseSpeed 80 //base speed of motors
#define CalSpeed 40 // calibration speed of motors
#define FwdSpeed 40 //forward speed of motors
#define RevSpeed 40 //backward speed of motors
#define turnSpeed 80

#define KS 1000 // sensor factor used for calibration
#define overlap 150 //line centering post calibration
#define EN1 9
#define EN2 10
#define TPin 6
#define Sens 550

#define TPinLeft 8
#define TPinRight 7

//IN is direction control
//EN is speed control
//LEFT: EN1 = 9  IN1 = 5  IN2 = 4 RIGHT: EN2 = 10  IN3 = 2  IN4 = 3

//PID constants subject to change 

//16/10/2024 For 40 speed
// Kp = 0.04;
// rest all 0;
//17/10/2024 For 80 speed
// Kp = 0.025
// Kd = 0.06
// Ki = 0 This is probably being used to fix the offset of the sensor from the line

// for 60 speed
// float Kp = 0.04;
// float Kd = 0.08;
// float Ki = 0.00;

// for 40 speed
float Kp = 0.025; //0.0015
float Kd = 0.06;//0.04
float Ki = 0.000;

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

//Running status
bool Running = false; //set to true for testing, make it false later on

//test shit
int sL = 0, sR = 0;

//Time counters
unsigned long time0 = 0;
unsigned long time1 = 0;

//Motor A
int IN1 = 5;
int IN2 = 4;
//Motor B
int IN3 = 2;
int IN4 = 3;

//Motor Speeds;
int speedLeft = 0;
int speedRight = 0;

//Switch Pins
int StartSwitch = 11;
int switchState = 0;

bool detectingJunction = false;
bool turning = false;


//function prototypes
void Calibrate(void);
void GetSensorValues(void);
void CalculatePID(void);
void MotorControl(int running);
void rightFwd(int speed);
void rightRev(int speed);
void leftFwd(int speed);
void leftRev(int speed);
void check_and_toggle_switch(void);
void testSuite(void);

bool checkPosition(char *str);
 
void setup(){
  Serial.begin(500000);
  delay(5000); //wait for a while before doing calibration
  // Set all the motor control pins to outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT); //originally setting this to high sets left to fwd
  pinMode(EN1, OUTPUT);//setting this to high will put Motor A in FWD
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT); //originally setting this to high sets right to fwd
  pinMode(EN2, OUTPUT);//setting this to high will put Motor B in FWD
  pinMode(StartSwitch, INPUT_PULLUP);
  pinMode(TPin, OUTPUT);
  pinMode(TPinLeft, OUTPUT);
  pinMode(TPinRight, OUTPUT);

  // Motor A
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  
  // Motor B
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(TPin, 140);
  digitalWrite(TPinLeft, 1);
  digitalWrite(TPinRight, 1);

  // Switches
  //pinMode(StartSwitch, INPUT); 
  //pinMode(CalibrateSwitch, INPUT); // might not be needed as calibration is done on power on
  //calibrate
  Serial.println("Calibration Started");
  Calibrate();
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Calibration Ended");
  // Calibration End
  analogWrite(TPin, 0);
  digitalWrite(TPinRight, 0);
  digitalWrite(TPinLeft, 0);
  delay(500);
}

void loop(){
  //control sequence
  check_and_toggle_switch();

  //Look for line
  GetSensorValues();

  //Track turning
  CalculatePID();

  //Follow line
  MotorControl(Running);
  testSuite();
}

void Calibrate(){
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

    //analogWrite(IN2, CalSpeed);
    leftFwd(CalSpeed);
    //analogWrite(IN3, CalSpeed + 10);
    rightRev(CalSpeed+10);
  }
}

void GetSensorValues(){
  lineDetected = false;
  unsigned long weightedSum = 0;
  unsigned int sum = 0;

  for (int i = 0; i<IR; i++){
    float curVal = 0;
    for(int j = 0; j < 3; j++){
      curVal += analogRead(SensorPin[i]);
    }
    Sensors[i] = curVal/3;
    //ks,0 -> 1000,0
    //smin < s < smax
    Sensors[i] = map(Sensors[i], MinSensors[i], MaxSensors[i], KS, 0); //print this out during testing to see what kinda values we get (high vals for black)
    Sensors[i] = constrain(Sensors[i], 0, KS);
    // threshold for line detect
    if (Sensors[i]>=Sens) lineDetected = true;
    if (Sensors[i] >= 50){
      //Weight sensor values by position of sensor?
      //Center sensor should have least priority and extreme sensors should have most priority
      weightedSum += long(Sensors[i])*(i*KS);
      sum += Sensors[i];
    }

    // if(Sensors[0] >= Sens){
    //   digitalWrite(TPinLeft, 1);
    // } else {
    //   digitalWrite(TPinLeft, 0);
    // }

    // if(Sensors[4] >= Sens){
    //   digitalWrite(TPinRight, 1);
    // } else {
    //   digitalWrite(TPinRight, 0);
    // }
  }

  if (lineDetected) positionX = weightedSum/sum; // stores the weighted mean of the index of the ir sensors
  else if (positionX < positionM) positionX = 0;
  else positionX = positionM*2; //basically 4 lol
  positionH = positionX - positionM; // testing ke liye this will be used otherwise it is pointless


  //Position x = 0 is left and position x = 4 is right
  //0 1 2 3 4
  //-2 -1 0 1 2
}

void CalculatePID(){
  P = positionX - positionM;
  I = P + PrevError; // OG was I = P + PrevError
  D = P - PrevError;
  PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
  PrevError = P;
}

bool checkPosition(char *str){
  //Refresh Sensor Cache
  GetSensorValues();
  bool match = true;
  for(int i = 0; i < IR and match; i++){
    switch (str[i])
    {
    case '1':
      match = Sensors[i]>=550;
      break;
    case '0':
      match = Sensors[i]<550;
      break;
    }
  }
  return match;
}

void MotorControl(int running){
  
  if(running){
    explore();

  }

  if(lineDetected){
    //Case 1
    if(running){
        speedLeft = BaseSpeed + PIDvalue;
        speedRight = BaseSpeed - PIDvalue;

        speedLeft = constrain(speedLeft, 0, 255);
        speedRight = constrain(speedRight, 0, 255);
      sL = speedLeft;
      sR = speedRight;
      //motor A
      // analogWrite(IN1, 0);
      // analogWrite(IN2, speedLeft);
      leftFwd(speedLeft);
      //motor B
      // analogWrite(IN3, 0);
      // analogWrite(IN4, speedRight);
      rightFwd(speedRight);
    } else {
      if (P<-overlap){
        speedLeft = -PIDvalue;
        speedRight = speedLeft;
        sL = speedLeft;
        sR = speedRight;
        // motor A
        // analogWrite(IN1, speedLeft);
        // analogWrite(IN2, 0);
        leftRev(speedLeft);
        // motor B
        // analogWrite(IN3, 0);
        // analogWrite(IN4, speedRight);
        rightFwd(speedRight);
      } else if (P > overlap){
        speedLeft = PIDvalue;
        speedRight = speedLeft;
        sL = speedLeft;
        sR = speedRight;
        // motor A
        // analogWrite(IN1, 0);
        // analogWrite(IN2, speedLeft);
        leftFwd(speedLeft);
        // motor B
        // analogWrite(IN3, speedRight);
        // analogWrite(IN4, 0);
        rightRev(speedRight);
      } else {
        // motor A
        // analogWrite(IN1, 0);
        // analogWrite(IN2, 0);
        sL = 0;
        sR = 0;
        leftFwd(0);
        // motor B
        // analogWrite(IN3, 0);
        // analogWrite(IN4, 0);
        rightFwd(0);
      }
    }
  } else {
    if (P == -positionM){
      speedLeft = FwdSpeed;
      speedRight = RevSpeed;
      sL = speedLeft;
      sR = speedRight;
      //motor A
      // analogWrite(IN1, speedLeft);
      // analogWrite(IN2, 0);
      leftRev(speedLeft);
      //motor B
      // analogWrite(IN3, 0);
      // analogWrite(IN4, speedRight);
      rightFwd(speedRight);
    } else if (P == positionM){
      speedLeft = RevSpeed;
      speedRight = FwdSpeed;
      sL = speedLeft;
      sR = speedRight;
      //motor A
      // analogWrite(IN1, 0);
      // analogWrite(IN2, speedLeft);
      leftFwd(speedLeft);
      //motor B
      // analogWrite(IN3, speedRight);
      // analogWrite(IN4, 0);
      rightRev(speedRight);
    }
  }
}

void analyzeJunction(){
  digitalWrite(TPin, 1);
  //Make sure the bot is going straight while checking junction
  leftFwd(turnSpeed);
  rightFwd(turnSpeed);
  while(!checkPosition("0***0")){
    delay(10);
  }
  digitalWrite(TPin, 0);
  leftRev(10);
  rightRev(10);
}

void explore(){
  //Implementing LSRB

  if(checkPosition("11111")){
    detectingJunction = true;
    analyzeJunction();
    detectingJunction = false;
    //TODO Add L to the turns list;
    turnLeft();
  } else if (checkPosition("111*0")) {
    //Turn left
    detectingJunction = true;
    analyzeJunction();
    detectingJunction = false;
    //TODO Add L to the turns list;
    turnLeft();

  } else if (checkPosition("0*111")) {
    digitalWrite(TPin, 1);
    //Turn right
    detectingJunction = true;
    analyzeJunction();
    detectingJunction = false;
    if(checkPosition("**1**")){
      //TODO add S to the turns list;
    } else {
      //TODO add R to the turns list;
      turnRight();
    }

  } else if (checkPosition("00000")) {
    //TODO add  B to the turns list;
    turnBack();
  }
}

void leftFwd(int speed){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN1, speed);
}

void leftRev(int speed){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(EN1, speed);
}

void rightFwd(int speed){
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(EN2, speed);
}

void rightRev(int speed){
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(EN2, speed);
}

void adjustSpeed(){
  //Call after calculating PID Constants
  speedLeft = BaseSpeed + PIDvalue;
  speedRight = BaseSpeed - PIDvalue;

  speedLeft = constrain(speedLeft, 0, 255);
  speedRight = constrain(speedRight, 0, 255);
}

//Implement something to check if the turn is complete
//Maybe check if only the middle sensor gives a reading??
//PID should be able to handle the alighnment later


void turnRight(){
  Serial.println("Turning Right...");
  turning = true;
  int c = 0;
  bool online = true;
  digitalWrite(TPinRight, 1);
  leftFwd(0);
  rightFwd(0);
  delay(1000);
  leftFwd(turnSpeed);
  rightRev(turnSpeed);

  //Wait 1 second
  //delay(200);

  while(c < 2){
    if(checkPosition("**1**") != online){
      c++;
      online = !online;
    }

    delay(10);
  }

  //keep checking if turn is complete
  while(!checkPosition("**1**")){
    delay(10);
  }

  digitalWrite(TPinRight, 0);
  turning = false;
}

void turnLeft() {
  Serial.println("Turning Left...");
  turning = true;
  int c = 0;
  bool online = true;
  digitalWrite(TPinLeft, 1);
  leftFwd(0);
  rightFwd(0);
  delay(1000);
  rightFwd(turnSpeed);
  leftRev(turnSpeed);

  //Wait 1 second
  //delay(200);

  //keep checking if turn is complete

  while(c < 2){
    if(checkPosition("**1**") != online){
      c++;
      online = !online;
    }

    delay(10);
  }

  digitalWrite(TPinLeft, 0);

  turning = false;
}

void turnBack() {
  turning = true;
  //Try turning at full speed???
  leftRev(turnSpeed);
  rightFwd(turnSpeed);

  while(!checkPosition("**1**")){
    delay(10);
  }

  turning = false;
}

void check_and_toggle_switch(){
  switchState = digitalRead(StartSwitch);
  if (switchState != HIGH){
    delay(2000); // avoid repeated reading
    Running = !Running;
  }
}

void testSuite(){
  //for testing
  char data[150];
  char sensor_position[6];
  sprintf(sensor_position, "%5d", positionH);
  // sensor 1 2 3 4 5; 
  sprintf(data, "Sensors: %4d  %4d  %4d  %4d  %4d Motors: %4d  %4d position: %4d", Sensors[0], Sensors[1], Sensors[2], Sensors[3], Sensors[4],sL ,sR, positionX);
  Serial.print(data);
  Serial.print(sensor_position);
  Serial.print(" Running status: ");
  Serial.println(Running);
}
