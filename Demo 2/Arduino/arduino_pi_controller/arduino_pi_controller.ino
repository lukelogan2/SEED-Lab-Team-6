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

int current_state = STATE_FIND_TAPE;


double start_time = 0;
double lastTime = 0;
double tape_found_time = 0;

int rightEncoderCurrent = LOW;
int rightEncoderLast = rightEncoderCurrent;
int leftEncoderCurrent = LOW;
int leftEncoderLast = leftEncoderCurrent;

bool moved = 0;
bool rotationComplete = 0;
bool movementComplete = 1;

volatile int rightCount = 0;
volatile int leftCount = 0;
volatile int totalCount = 0;
volatile int diffCount = 0;


double wheelRadius = 0.075; //meters
double wheelDistance = 0.308; //meters real 0.31m

double rho_e = 0;   //distance error
double phi_e = 0;   //angle error

double current_rho = 0;
double current_phi = 0;

double target_rho = 0.5;    //meters
double target_phi = 0;   //Degrees

int directionRight;
int directionLeft;
double rightPower;
double leftPower;

double count_error = 0; //count sum for movement
double errorSum = 0;    //error sum for integral controller

double encoderSteps = 16*50;
double theta = 360/encoderSteps;

// Serial Input from Pi Variables 
String data;
bool DataRead = false;
int doneFlag = 0;  // Declares the rover has reached the tape
int startFlag = 0; 
double angle_to_tape; // Angle between the rover's path and the tape
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

  attachInterrupt(digitalPinToInterrupt(RENCA),rightRotation,RISING);
  attachInterrupt(digitalPinToInterrupt(LENCA),leftRotation,RISING);

  theta = (360)/encoderSteps;     //degrees
//  theta = (2*PI)/encoderSteps;    //radians

}

void loop() {
    // Read Serial Input from pi and print it
  if (DataRead) {
    DataRead = false;
  }
  
  start_time = millis();      //initial time polling
  double target_counts = target_rho*100/47.1*800;     //meters
//  double target_counts = target_rho*800*12/5.9;       //feet 

  digitalWrite(pwmPower, HIGH);   //give the motor power
  
/************************************************/  
// PI controller configuration
/************************************************/
  double Kp = 35;
  double Ki = 5;

//  Serial.print("R ");
//  Serial.print(rightCount);
//  Serial.print("  L ");
//  Serial.println(leftCount);
/************************************************/  
// Rotation Control
/************************************************/

  if (startFlag == 1 && rotationComplete == 0){
    if(angle_to_tape == 0){
       setMotor(PWMR, 60, INR, 0);    //call the motor function right motor based on direction needed to rotate
       setMotor(PWML, 60, INL, 0);      //call the motor function left motor based on direction needed to rotate
    }
    else if (tape_found_time == 0) {
      //Serial.println("Tape Found");
      setMotor(PWMR, 0, INR, 1);
      setMotor(PWML, 0, INL, 0);
      rightCount = 0;
      leftCount = 0;
      tape_found_time = millis();
      setTarget = true;
//    current_phi = (rightCount - leftCount)*theta*wheelRadius / wheelDistance;   //calculate the current angle
    }
    else if (millis() - tape_found_time > 3000) {
      //Serial.println("Rotate to Tape");
      if (setTarget) {
        Serial.print("Set target angle: ");
        Serial.println(angle_to_tape);
        target_phi = 13.7;
        setTarget = false;
      }
      state_rotate_to_tape();
    }
  }


/************************************************/  
// Movement Control
/************************************************/
  if(movementComplete == 0) {
    //Serial.println("Movement");
    while (totalCount < target_counts) {
      totalCount = (rightCount + leftCount)/2;
      start_time = millis();
      
      /************************************************/  
      // PI controller
      /************************************************/
      count_error = target_counts - totalCount;   //find the error between desired and current
      double timeChange = start_time - lastTime;  //calculate the time difference
      errorSum += (count_error * timeChange);     //sum the previous error
      double u = Kp * count_error + Ki * errorSum;      //determine the power from the PI
      
      /************************************************/  
      // Motor Control
      /************************************************/
      
      diffCount = rightCount - leftCount;
      if(diffCount > 0){
        leftPower = fabs(u + diffCount);
        rightPower = fabs(u);
      }
      if(diffCount < 0){
        rightPower = fabs(u) + fabs(diffCount);
        leftPower = fabs(u);
      }
      if(diffCount == 0){
        rightPower = fabs(u);
        leftPower = fabs(u);
      }
   
//      rightPower = constrain(rightPower, 0, 95);    //constrain the power to prevent slipping on slick surfaces
//      leftPower = constrain(leftPower, 0, 101);     //constrain the power to prevent slipping on slick surfaces

      setMotor(PWMR, rightPower, INR, 1);   //call the motor function right motor forward
      setMotor(PWML, leftPower, INL, 0);    //call the motor function left motor forward
      lastTime = start_time;
   }
   
/************************************************/  
// Turn off the motor to stop
/************************************************/
      //Serial.println("Movement Complete");
      setMotor(PWMR, 0, INR, 1);    //call the motor function
      setMotor(PWML, 0, INL, 0);    //call the motor function
      digitalWrite(pwmPower, LOW);  //stop power to the motor
      movementComplete = 1;         
      rotationComplete = 1;
  }
    
  lastTime = start_time;
}

/***********************************************/
// State Functions
/***********************************************/

void state_rotate_to_tape() {
  /************************************************/  
  // PI controller configuration
  /************************************************/
  double Kp = 35;
  double Ki = 5;

  /************************************************/  
  // Rotation Control
  /************************************************/

  if (rotationComplete == 0) {
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
    rightPower = constrain(rightPower, 0, 65);    //constrain the power to prevent slipping on slick surfaces
    leftPower = fabs(u);
    leftPower = constrain(leftPower, 0, 70);      //constrain the power to prevent slipping on slick surfaces
    

    setMotor(PWMR, rightPower, INR, directionRight);    //call the motor function right motor based on direction needed to rotate
    setMotor(PWML, leftPower, INL, directionLeft);      //call the motor function left motor based on direction needed to rotate

  /************************************************/  
  // Check to end rotation
  /************************************************/
      if(phi_e < 1 && phi_e > -1){
        Serial.println("Rotation Complete");
        movementComplete = 0;
        rotationComplete = 1;

        setMotor(PWMR, 0, INR, 1);
        setMotor(PWML, 0, INL, 0);
        delay(1000);
        rightCount = 0;
        leftCount = 0;
      }
  }
}

//void state_drive_to_tape() {
//   /************************************************/  
//  // Movement Control
//  /************************************************/
//    if(movementComplete == 0) {
//      //Serial.println("Movement");
//      while (totalCount < target_counts) {
//        totalCount = (rightCount + leftCount)/2;
//        start_time = millis();
//        
//        /************************************************/  
//        // PI controller
//        /************************************************/
//        count_error = target_counts - totalCount;   //find the error between desired and current
//        double timeChange = start_time - lastTime;  //calculate the time difference
//        errorSum += (count_error * timeChange);     //sum the previous error
//        double u = Kp * count_error + Ki * errorSum;      //determine the power from the PI
//        
//        /************************************************/  
//        // Motor Control
//        /************************************************/
//        
//        diffCount = rightCount - leftCount;
//        if(diffCount > 0){
//          leftPower = fabs(u + diffCount);
//          rightPower = fabs(u);
//        }
//        if(diffCount < 0){
//          rightPower = fabs(u) + fabs(diffCount);
//          leftPower = fabs(u);
//        }
//        if(diffCount == 0){
//          rightPower = fabs(u);
//          leftPower = fabs(u);
//        }
//     
//  //      rightPower = constrain(rightPower, 0, 95);    //constrain the power to prevent slipping on slick surfaces
//  //      leftPower = constrain(leftPower, 0, 101);     //constrain the power to prevent slipping on slick surfaces
//  
//        setMotor(PWMR, rightPower, INR, 1);   //call the motor function right motor forward
//        setMotor(PWML, leftPower, INL, 0);    //call the motor function left motor forward
//        lastTime = start_time;
//     }
//     
//    /************************************************/  
//    // Turn off the motor to stop
//    /************************************************/
//    //Serial.println("Movement Complete");
//    setMotor(PWMR, 0, INR, 1);    //call the motor function
//    setMotor(PWML, 0, INL, 0);    //call the motor function
//    digitalWrite(pwmPower, LOW);  //stop power to the motor
//    movementComplete = 1;         
//    rotationComplete = 1;
//    }
//      
//    lastTime = start_time;
//}


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
