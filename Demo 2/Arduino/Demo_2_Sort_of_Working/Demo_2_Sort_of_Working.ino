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

// State variables
#define STATE_FIND_TAPE         20
#define STATE_ROTATE_TO_TAPE    21
#define STATE_DRIVE_TO_TAPE     22
#define STATE_FOLLOW_TAPE       23
#define STATE_DONE              24
#define STATE_WAIT              25

int state = STATE_FIND_TAPE;
int prev_state = STATE_FIND_TAPE;

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

double step_phi = 30;   //degrees

int directionRight;
int directionLeft;
double rightPower;
double leftPower;

double wait_time = 0;
int waitFlag = 1;
int stepFlag = 1;
double step_time;

double count_error = 0; //count sum for movement
double errorSum = 0;    //error sum for integral controller

double Kp;
double Ki;

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
    //Serial.println(data);
  }
  
  start_time = millis();      //initial time polling
//  double target_counts = target_rho*800*12/5.9;       //feet 

  digitalWrite(pwmPower, HIGH);   //give the motor power
  
/************************************************/  
// PI controller configuration
/************************************************/
  Kp = 35;
  Ki = 5;

//  Serial.print("R ");
//  Serial.print(rightCount);
//  Serial.print("  L ");
//  Serial.println(leftCount);
  // --------------------------------------
  // State Machine Conditional Statements
  // --------------------------------------
    if (startFlag == 1){
  if (state == STATE_FIND_TAPE) {
    state_find_tape();
  }
  if (state == STATE_ROTATE_TO_TAPE) {
    state_rotate_to_tape();
  }
  if (state == STATE_DRIVE_TO_TAPE) {
    state_drive_to_tape();
  }
  if (state == STATE_WAIT){
    state_wait();
  }
  if (state == STATE_DONE){
    state_done();
  }
  else if (state == STATE_FOLLOW_TAPE) {
    state_follow_tape();
  }
}
}

// =======================================================================
// Start of State Functions
// =======================================================================

// #########################################################
// STATE FUNCTION TO ROTATE UNTIL TAPE IS FOUND
// #########################################################
//void state_find_tape() {
//  /************************************************/  
//// Rotation Control
///************************************************/
//
//    if(angle_to_tape == 0){
//       setMotor(PWMR, 55, INR, 0);    //call the motor function right motor based on direction needed to rotate
//       setMotor(PWML, 50, INL, 0);      //call the motor function left motor based on direction needed to rotate
//    }
//    else{
//      state = STATE_ROTATE_TO_TAPE;
//      rightCount = 0;
//      leftCount = 0;
////      tape_found_time = millis();
//    }
////    if (millis()-tape_found_time > 3000) {
////        target_phi = angle_to_tape;
////        state = STATE_DONE;
////      }
//    }


 void state_find_tape(){
  if(stepFlag == 1){
     step_time = millis();
     stepFlag = 0;
  }
  if(angle_to_tape != 0){
      //Serial.println("STATE_ROTATE_TO_TAPE");
//      target_phi = angle_to_tape;
      state = STATE_WAIT;
      doneFlag = 0;
   }
   if(angle_to_tape == 0){
     stepRotation();
   }
  if(millis() - step_time > 5000){
    stepFlag = 1;
    rightCount = 0;
    leftCount = 0;
   }
  
}
// ###############################################################
// STATE FUNCTION TO ROTATE UNTIL ROVER IS CENTERED TOWARDS TAPE
// ###############################################################
void state_rotate_to_tape() {
  /************************************************/  
  // PI controller configuration
  /************************************************/
  Kp = 35;
  Ki = 5;

  /************************************************/  
  // Rotation Control
  /************************************************/

    current_phi = (rightCount - leftCount)*theta*wheelRadius / wheelDistance;   //calculate the current angle
    //Serial.println(current_phi);

  /************************************************/  
  // PI Controller
  /************************************************/

   phi_e = target_phi - current_phi;              //find the error between desired and current
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
        rightCount = 0;
        leftCount = 0;
        doneFlag = 0;
        totalCount = 0;
        movementComplete = 0;
        //state = STATE_DRIVE_TO_TAPE;
        ///Serial.println("STATE_DRIVE_TO_TAPE");
        if (prev_state == STATE_FIND_TAPE) {
          state = STATE_DRIVE_TO_TAPE;
        }
        else {
          doneFlag = 0;
          setMotor(PWMR, 0, INR, 1);
          setMotor(PWML, 0, INL, 0);
          state = STATE_FOLLOW_TAPE;
//          Serial.println("STATE_FOLLOW_TAPE");
        }
        prev_state = STATE_ROTATE_TO_TAPE;
      }
}

// ###############################################################
// STATE FUNCTION TO DRIVE UNTIL ROVER IS AT THE START OF TAPE
// ###############################################################
void state_drive_to_tape() {
    if(stepFlag == 1){
     step_time = millis();
     stepFlag = 0;
    }
    if(reachedFlag == 1){
        stepMovement();
        prev_state = STATE_DRIVE_TO_TAPE;
        state = STATE_WAIT;
        delay(10000);
        rightCount = 0;
        leftCount = 0;
        doneFlag = 0;
        movementComplete = 0;
        count_error = 0;
     }
     if(reachedFlag == 0){
       stepMovement();
     }
    if(millis() - step_time > 5000){
      stepFlag = 1;
      rightCount = 0;
      leftCount = 0;
      movementComplete = 0;
      count_error = 0;
     }
}

// ###############################################################
// STATE FUNCTION TO FOLLOW TAPE UNTIL THE ROVER REACHES THE END
// ###############################################################
void state_follow_tape() {
  if(stepFlag == 1){
     step_time = millis();
     stepFlag = 0;
    }
    if(doneFlag == 1){
        //Serial.println("STATE_ROTATE_TO_TAPE");
        stepMovement();
        state = STATE_DONE;
        rightCount = 0;
        leftCount = 0;
        doneFlag = 0;
        movementComplete = 0;
        count_error = 0;
     }
     if(doneFlag == 0){
       //Serial.println("Step Movement");
       stepMovement();
     }
    if(millis() - step_time > 5000){
      stepFlag = 1;
      rightCount = 0;
      leftCount = 0;
      movementComplete = 0;
      count_error = 0;
     }
}

void state_done(){
  setMotor(PWMR, 0, INR, 1);    //call the motor function
      setMotor(PWML, 0, INL, 0);    //call the motor function
      digitalWrite(pwmPower, LOW);  //stop power to the motor
}

void state_wait(){
    delay(5000);
    target_phi = angle_to_tape;
    state = STATE_ROTATE_TO_TAPE;
    Serial.println(target_phi);

  
//  if( waitFlag = 1){
//    wait_time = millis();
//    waitFlag = 0;
//  }
//  if ((millis() - wait_time) < 10000){
//    setMotor(PWMR, 0, INR, 1);    //call the motor function
//    setMotor(PWML, 0, INL, 0);    //call the motor function
//    target_phi = angle_to_tape;
//  }
//  else{
//    waitFlag = 1;
//    rightCount = 0;
//    digitalWrite(13,HIGH);
//    leftCount = 0;
//    state = STATE_ROTATE_TO_TAPE;
//  }
}

// =======================================================================
// End of State Functions
// =======================================================================



void stepRotation(){
  Kp = 35;
  Ki = 0;

  /************************************************/  
  // Rotation Control
  /************************************************/

    current_phi = (rightCount - leftCount)*theta*wheelRadius / wheelDistance;   //calculate the current angle
    //Serial.println(current_phi);

  /************************************************/  
  // PI Controller
  /************************************************/

   phi_e = step_phi - current_phi;              //find the error between desired and current
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

  /************************************************/  
  // Check to end rotation
  /************************************************/
      if(phi_e < 1 && phi_e > -1){
        setMotor(PWMR, 0, INR, 1);
        setMotor(PWML, 0, INL, 0);
        delay(1000);
//        rightCount = 0;
//        leftCount = 0;
}
}

void stepMovement() {
  Kp = 35;
  Ki = 5;
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
      setMotor(PWMR, 0, INR, 1);    //call the motor function
      setMotor(PWML, 0, INL, 0);    //call the motor function
      digitalWrite(pwmPower, LOW);  //stop power to the motor
      movementComplete = 1;         
      rotationComplete = 1;
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
    if (data[0] == 'a') {
      String angle_sub = data.substring(1);
      //Serial.println(angle_sub);
      angle_to_tape = angle_sub.toDouble();
    }
    else if (data[0] == 'f') {
      String flag_char = data.substring(1);
      //Serial.println(flag_char);
      doneFlag = flag_char.toInt();
    }
    else if (data[0] == 'r') {
      String flag_char = data.substring(1);
      //Serial.println(flag_char);
      reachedFlag = flag_char.toInt();
    }
    else if (data[0] == 's') {
      String flag_char = data.substring(1);
      //Serial.println(flag_char);
      startFlag = flag_char.toInt();
    }
    DataRead = true;
  }
  // Reset the serial buffer
  Serial.flush();
}
