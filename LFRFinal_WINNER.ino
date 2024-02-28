#define IR 5 //number of IR sensors
#define BaseSpeed 40 //base speed of motors
#define CalSpeed 40 // calibration speed of motors
#define FwdSpeed 40 //forward speed of motors
#define RevSpeed 60 //backward speed of motors
#define KS 1000 // sensor factor used for calibration
#define overlap 150 //line centering post calibration
#define EN1 9
#define EN2 10
//LEFT: EN1 = 9  IN1 = 5  IN2 = 4 RIGHT: EN2 = 10  IN3 = 2  IN4 = 3

//PID constants subject to change 
float Kp = 0.015;
float Kd = 0.3;
float Ki = 0.005;
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
int SpeedA = 0;
int SpeedB = 0;

//Switch Pins
int StartSwitch = 11;
int switchState = 0;

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

void setup(){
  Serial.begin(500000);
  delay(10000); //wait for a while before doing calibration
  // Set all the motor control pins to outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT); //originally setting this to high sets left to fwd
  pinMode(EN1, OUTPUT);//setting this to high will put Motor A in FWD
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT); //originally setting this to high sets right to fwd
  pinMode(EN2, OUTPUT);//setting this to high will put Motor B in FWD
  pinMode(StartSwitch, INPUT_PULLUP);

  // Motor A
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  
  // Motor B
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

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
  delay(500);
}

void loop(){
  //control sequence
  check_and_toggle_switch();
  GetSensorValues();
  CalculatePID();
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
  for(int i = 0; i<7000;i++){
    for (int j = 0; j<IR; j++){
      Sensors[j] = analogRead(SensorPin[j]);
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
    Sensors[i] = analogRead(SensorPin[i]);
    Sensors[i] = map(Sensors[i], MinSensors[i], MaxSensors[i], KS, 0); //print this out during testing to see what kinda values we get (high vals for black)
    Sensors[i] = constrain(Sensors[i], 0, KS);
    // threshold for line detect
    if (Sensors[i]>=450) lineDetected = true;
    if (Sensors[i] >= 50){
      weightedSum += long(Sensors[i])*(i*KS);
      sum += Sensors[i];
    }
  }

  if (lineDetected) positionX = weightedSum/sum;
  else if (positionX < positionM) positionX = 0;
  else positionX = positionM*2;
  positionH = positionX - positionM; // testing ke liye this will be used otherwise it is pointless
}

void CalculatePID(){
  P = positionX - positionM;
  I = P + PrevError; // OG was I = P + PrevError
  D = P - PrevError;
  PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
  PrevError = P;
}

void MotorControl(int running){
  int speedLeft = 0, speedRight = 0;

  if(lineDetected){
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

void check_and_toggle_switch(){
  switchState = digitalRead(StartSwitch);
  if (switchState != HIGH){
    delay(300); // avoid repeated reading
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












