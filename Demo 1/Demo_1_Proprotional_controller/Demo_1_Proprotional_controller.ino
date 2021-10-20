#define PI 3.141592653589793238
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
double run_time = 0;
double loop_time = 5;   //milliseconds

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
double wheelDistance = 0.31; //meters 
double v_l;
double v_r;
double loopTime = 10;

double rho_e = 0;
double phi_e = 0;

double current_rho = 0;
double current_phi = 0;

double target_rho = 1;    //meters
double target_phi = 45;   //Degrees

int directionRight;
int directionLeft;
double rightPower;
double leftPower;

double count_error = 0;



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
  start_time = millis();
  double target_counts = target_rho*100/47.1*800;     //meters
//  double target_counts = target_rho*800*12/5.9;       //feet 

//  if(moved == 1){
//    Serial.print(rightCount);
//    Serial.print("\t");
//    Serial.println(leftCount);
//    moved = 0;
//  }
  digitalWrite(pwmPower, HIGH);
  double Kp = 25;
  double Kpr = 35;
  double Kpl = 35;

  if (rotationComplete == 0){
    Serial.print("im in rotation ");
    Serial.print("rightcount: ");
    Serial.print(rightCount);
    Serial.print(" leftcount: ");
    Serial.println(leftCount);
    current_phi = (rightCount - leftCount)*theta*wheelRadius / wheelDistance;

    /************************************************/  
    // PI controller configuration
    /************************************************/
//      rho_e = target_rho - current_rho;
      phi_e = target_phi - current_phi;
  if(target_phi > 0){
      directionRight = 1;
      directionLeft = 1;
  }
  else{
    directionRight = 0;
    directionLeft = 0;
  }
//      Kp = 25;
      

    /************************************************/
    //Motor Control
    /************************************************/

      rightPower = fabs(Kpr*phi_e);
      rightPower = constrain(rightPower, 0, 190);
      leftPower = fabs(Kpl*phi_e);
      leftPower = constrain(leftPower, 0, 190);
      

      setMotor(PWMR, rightPower, INR, directionRight);
      setMotor(PWML, leftPower, INL, directionLeft);

      
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
//  if (movementComplete == 0){
//    
//    Serial.println("rightcount: ");
//    Serial.print(rightCount);
//    Serial.print(" leftcount: ");
//    Serial.println(leftCount);
//    current_rho = (rightCount + leftCount)*theta*0.5*wheelRadius;
//    double Kp_rho = 25;
//    rho_e = target_rho - current_rho;
//
//      rightPower = Kp_rho*rho_e;
//      rightPower = constrain(rightPower, 0, 50);
//      leftPower = Kp_rho*rho_e;
//      leftPower = constrain(leftPower, 0, 50);
//      
//
//      setMotor(PWMR, rightPower, INR, 1);
//      setMotor(PWML, leftPower, INL, 0);
//      
//      
//      if(rho_e < 1 && rho_e > -1){
//        movementComplete = 1;
//        setMotor(PWMR, 0, INR, 1);
//        setMotor(PWML, 0, INL, 0);
//        digitalWrite(pwmPower, LOW);
//        Serial.println("finished");
//      }
//  }
if(movementComplete == 0){
  while( rightCount < target_counts){
    count_error = target_counts - rightCount;
    rightPower = Kp*count_error;
    leftPower = Kp*count_error;
//    Serial.print("rightcount: ");
//    Serial.print(rightCount);
//    Serial.print(" leftcount: ");
//    Serial.println(leftCount);
      setMotor(PWMR, rightPower, INR, 1);
      setMotor(PWML, leftPower, INL, 0);
}
        setMotor(PWMR, 0, INR, 1);
        setMotor(PWML, 0, INL, 0);
        digitalWrite(pwmPower, LOW);
        movementComplete = 1;
        rotationComplete = 1;
}
      
//    analogWrite(PWMR, 170);
//    analogWrite(PWML, 170);
//    digitalWrite(INR, HIGH);
//    digitalWrite(INL, HIGH);
    

  
  run_time = millis();
}

void setMotor (int pwm, int pwmVal, int inval, int dir){
  analogWrite(pwm, pwmVal);
  if(dir == 1){
    digitalWrite(inval, HIGH);
  }
  if(dir == 0){
    digitalWrite(inval, LOW);
  }
}

void rightRotation() {
   rightEncoderCurrent = digitalRead(RENCA);

//   if((rightEncoderLast == LOW) && (rightEncoderCurrent == HIGH)){
    if(digitalRead(RENCB) == LOW){
      rightCount = rightCount + 1;
    }
    else{
      rightCount = rightCount - 1;
    }
//   }
 
   moved = 1;
}

void leftRotation() {
  leftEncoderCurrent = digitalRead(LENCA);

//   if((leftEncoderLast == LOW) && (leftEncoderCurrent == HIGH)){
    if(digitalRead(LENCB) == HIGH){
      leftCount = leftCount + 1;
    }
    else{
      leftCount = leftCount - 1;
    }
//   }
   moved = 1;
}
