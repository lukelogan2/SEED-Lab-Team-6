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
double lastTime = 0;

int rightEncoderCurrent = LOW;
int rightEncoderLast = rightEncoderCurrent;
int leftEncoderCurrent = LOW;
int leftEncoderLast = leftEncoderCurrent;

bool moved = 0;
bool rotationComplete = 0;
bool movementComplete = 1;

volatile int rightCount = 0;
volatile int leftCount = 0;


double wheelRadius = 0.075; //meters
double wheelDistance = 0.308; //meters real 0.31m

double rho_e = 0;   //distance error
double phi_e = 0;   //angle error

double current_rho = 0;
double current_phi = 0;

double target_rho = 1;    //meters
double target_phi = 90;   //Degrees

int directionRight;
int directionLeft;
double rightPower;
double leftPower;

double count_error = 0; //count sum for movement
double errorSum = 0;    //error sum for integral controller

double encoderSteps = 16*50;
double theta = 360/encoderSteps;

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

  attachInterrupt(digitalPinToInterrupt(RENCA),rightRotation,RISING);
  attachInterrupt(digitalPinToInterrupt(LENCA),leftRotation,RISING);

  theta = (360)/encoderSteps;     //degrees
//  theta = (2*PI)/encoderSteps;    //radians

}

void loop() {
  start_time = millis();      //initial time polling
  double target_counts = target_rho*100/47.1*800;     //meters
//  double target_counts = target_rho*800*12/5.9;       //feet 

  digitalWrite(pwmPower, HIGH);   //give the motor power
  
/************************************************/  
// PI controller configuration
/************************************************/
  double Kp = 35;
  double Ki = 5;

/************************************************/  
// Rotation Control
/************************************************/

  if (rotationComplete == 0){
    current_phi = (rightCount - leftCount)*theta*wheelRadius / wheelDistance;   //calculate the current angle

/************************************************/  
// PI Controller
/************************************************/

   phi_e = target_phi - current_phi;              //find the error between desired and current
   if(phi_e>30)
   {
    Ki =0;
   }
   else
   {
    Ki =5;
   }
   double timeChange = start_time - lastTime;     //calculate the time difference
   errorSum += (phi_e * timeChange);              //sum the previous error
   double u = Kp * phi_e + Ki * errorSum;         //determine the power from the PI
   
/************************************************/  
// Direction determination
/************************************************/

  if(phi_e >= 0){         //go counterclockwise
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
      rightPower = constrain(rightPower, 0, 65);    //constrain the power to prevent slipping on slick surfaces
      leftPower = fabs(u);
      leftPower = constrain(leftPower, 0, 70);      //constrain the power to prevent slipping on slick surfaces
      

      setMotor(PWMR, rightPower, INR, directionRight);    //call the motor function right motor based on direction needed to rotate
      setMotor(PWML, leftPower, INL, directionLeft);      //call the motor function left motor based on direction needed to rotate

/************************************************/  
// Check to end rotation
/************************************************/
      if(phi_e < 1 && phi_e > -1){
        movementComplete = 0;
        rotationComplete = 1;

        setMotor(PWMR, 0, INR, 1);
        setMotor(PWML, 0, INL, 0);
        delay(1000);
        rightCount = 0;
        leftCount = 0;
      }
  }

/************************************************/  
// Movement Control
/************************************************/
  if(movementComplete == 0){
    while( rightCount < target_counts){
      start_time = millis();

/************************************************/  
// PI controller
/************************************************/
      count_error = target_counts - rightCount;   //find the error between desired and current
      double timeChange = start_time - lastTime;  //calculate the time difference
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
      lastTime = start_time;
   }
   
/************************************************/  
// Turn off the motor to stop
/************************************************/
      setMotor(PWMR, 0, INR, 1);    //call the motor function
      setMotor(PWML, 0, INL, 0);    //call the motor function
      digitalWrite(pwmPower, LOW);  //stop power to the motor
      movementComplete = 1;         
      rotationComplete = 1;
  }
    
lastTime = start_time;
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
