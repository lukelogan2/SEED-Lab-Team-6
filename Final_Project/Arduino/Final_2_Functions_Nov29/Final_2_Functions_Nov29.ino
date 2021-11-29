//#define PI 3.141592653589793238
#define RENCA 2   //White right
#define RENCB 5   //Yellow right
#define LENCA 3   //White left
#define LENCB 6   //Yellow left
#define PWMR 9
#define PWML 10
#define INR 8
#define INL 7
#define pwmPower 4

double start_time = 0;
double last_time = 0;
double tape_found_time = 0;

int rightEncoderCurrent = LOW;
int rightEncoderLast = rightEncoderCurrent;
int leftEncoderCurrent = LOW;
int leftEncoderLast = leftEncoderCurrent;

bool moved = 0;
bool rotationComplete = 0;
bool movementComplete = 0;

volatile int rightCount = 0;
volatile int leftCount = 0;
volatile double totalCount = 0;
volatile int diffCount = 0;


double wheelRadius = 0.075; //meters
double wheelDistance = 0.308; //meters real 0.31m

double rho_e = 0;   //distance error
double phi_e = 0;   //angle error

double current_rho = 0;
double current_phi = 0;

double target_rho = 0.1;    //meters
double target_phi = 0;   //Degrees

double step_phi = 20;   //degrees
int step_count = 0;

int directionRight;
int directionLeft;
double rightPower;
double leftPower;

double wait_time = 0;
int waitFlag = 1;
int stepFlag = 1;
double step_time;
double delay_time = 0;

double count_error = 0; //count sum for movement
double errorSum = 0;    //error sum for integral controller

double Kp;
double Ki;

//String str_distance;
//String str_distance;
double angle;
double distance;

double encoderSteps = 16*50;
double theta = 360/encoderSteps;
double  target_counts = target_rho*100/47.1*800;     //meters

// Serial Input from Pi Variables 
String data;
bool DataRead = false;
int doneFlag = 0;  // Declares the rover has reached the tape
int reachedFlag = 0;
int startFlag = 0; 
double angle_to_tape = 0; // Angle between the rover's path and the tape
bool setTarget = false;

void setup() {
  Serial.begin(115200);
  pinMode(RENCA, INPUT);
  pinMode(RENCB, INPUT);
  pinMode(LENCA, INPUT);
  pinMode(LENCB, INPUT);
  
  pinMode(pwmPower, OUTPUT);
  pinMode(PWML, OUTPUT);
  pinMode(PWMR, OUTPUT);
  pinMode(INL, OUTPUT);
  pinMode(INR, OUTPUT);
  pinMode(13,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(RENCA),rightRotation,RISING);
  attachInterrupt(digitalPinToInterrupt(LENCA),leftRotation,RISING);

  theta = (360)/encoderSteps;     //degrees
//  theta = (2*PI)/encoderSteps;    //radians

}

void loop() {
    // Read Serial Input from pi and print it
  if (DataRead) {
    DataRead = false;
    for (int i = data.length();i++;){
      if(data[i] = ','){
        String str_angle = data.substring(0,i-1);
        //angle = atof(str_angle);
        angle = str_angle.toDouble();
        String str_distance = data.substring(i+1,data.length());
        //distance = atof(str_distance);
        distance = str_distance.toDouble();
      }
    }
    //Serial.println(data);
  }
  
  start_time = millis();      //initial time polling
  double target_counts = distance*800*12/5.9;       //feet 

  digitalWrite(pwmPower, HIGH);   //give the motor power


  if (angle >= 5 || angle <= -5){
    stepRotation();
  }
  else {
    stepMovement();
  }
      
  
}


// ----------------------------------------------------------------------
// Function to rotate a predetermined degrees 
// ----------------------------------------------------------------------
void stepRotation(){
  Kp = 25;
  Ki = 5;

  /************************************************/  
  // Rotation Control
  /************************************************/
    
    current_phi = (rightCount - leftCount)*theta*wheelRadius / wheelDistance;   //calculate the current angle
    //Serial.println(current_phi);

  /************************************************/  
  // PI Controller
  /************************************************/

   phi_e = angle - current_phi;                   //find the error between desired and current
   double timeChange = start_time - last_time;     //calculate the time difference
   errorSum += (phi_e * timeChange);              //sum the previous error
   double u = Kp * phi_e + Ki * errorSum;         //determine the power from the PI
   
/************************************************/  
// Direction determination
/************************************************/

  if (phi_e >= 0) {         //go counterclockwise
      directionRight = 1;
      directionLeft = 1;
  }
  else{                   //go clockwise
      directionRight = 0;
      directionLeft = 0;
  }

      
  /************************************************/
  //Motor Control
  /************************************************/

    rightPower = fabs(u);
    rightPower = constrain(rightPower, 0, 60);    //constrain the power to prevent slipping on slick surfaces
    leftPower = fabs(u);
    leftPower = constrain(leftPower, 0, 65);      //constrain the power to prevent slipping on slick surfaces
    

    setMotor(PWMR, rightPower, INR, directionRight);    //call the motor function right motor based on direction needed to rotate
    setMotor(PWML, leftPower, INL, directionLeft);      //call the motor function left motor based on direction needed to rotate

    last_time = start_time;
  /************************************************/  
  // Check to end rotation
  /************************************************/
      if(phi_e < 5 && phi_e > -5){
        setMotor(PWMR, 0, INR, 1);
        setMotor(PWML, 0, INL, 0);
        rotationComplete = 1;
        delay_time = millis();
  }
}

// ----------------------------------------------------------------------
// Function to Drive forwards a predetermined distance
// ----------------------------------------------------------------------
void stepMovement() {
  Kp = 35;
  Ki = 5;
/************************************************/  
// Movement Control
/************************************************/
if(target_counts > totalCount){
      totalCount = (rightCount + leftCount) / 2;
      start_time = millis();

/************************************************/  
// PI controller
/************************************************/
      count_error = target_counts - totalCount;   //find the error between desired and current
      double timeChange = start_time - last_time;  //calculate the time difference
      errorSum += (count_error * timeChange);     //sum the previous error
      double u = Kp * phi_e + Ki * errorSum;      //determine the power from the PI

/************************************************/  
// Motor Control
/************************************************/
   
      rightPower = fabs(u);
      rightPower = constrain(rightPower, 0, 95);    //constrain the power to prevent slipping on slick surfaces
      leftPower = fabs(u);
      leftPower = constrain(leftPower, 0, 101);     //constrain the power to prevent slipping on slick surfaces

      setMotor(PWMR, rightPower, INR, 1);   //call the motor function right motor forward
      setMotor(PWML, leftPower, INL, 0);    //call the motor function left motor forward
      last_time = start_time;
   }
   
/************************************************/  
// Turn off the motor to stop
/************************************************/
else{
      setMotor(PWMR, 0, INR, 1);    //call the motor function
      setMotor(PWML, 0, INL, 0);    //call the motor function
      digitalWrite(pwmPower, LOW);  //stop power to the motor
      movementComplete = 1;         
      rotationComplete = 1;
      step_count++;
  }
    
last_time = start_time;
}

/************************************************/  
// Motor Function
/************************************************/
void setMotor (int pwm, int pwmVal, int inval, int dir){
  analogWrite(pwm, pwmVal);     //write the power to the motor
  if(dir == 1){
    digitalWrite(inval, HIGH);  //define direction of the motor
  }
  if(dir == 0){
    digitalWrite(inval, LOW);   //define direction of the motor
  }
}

void rightRotation() {
   rightEncoderCurrent = digitalRead(RENCA);

    if(digitalRead(RENCB) == LOW){
      rightCount = rightCount + 1;
    }
    else{
      rightCount = rightCount - 1;
    }
   moved = 1;
}

void leftRotation() {
  leftEncoderCurrent = digitalRead(LENCA);

    if(digitalRead(LENCB) == HIGH){
      leftCount = leftCount + 1;
    }
    else{
      leftCount = leftCount - 1;
    }
   moved = 1;
}

// Serial Communication Handler
void serialEvent() {
  // If a message from the Raspberry Pi is present, read the data
  if (Serial.available() > 0) {
    data = Serial.readStringUntil('\n');
    DataRead = true;
    rightCount = 0;
    leftCount = 0;
    errorSum = 0;
  }
  // Reset the serial buffer
  Serial.flush();
}
