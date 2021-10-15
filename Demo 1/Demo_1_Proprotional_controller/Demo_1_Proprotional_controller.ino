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
double run_time = 0;
double loop_time = 5;   //milliseconds

int rightEncoderCurrent = LOW;
int rightEncoderLast = rightEncoderCurrent;
int leftEncoderCurrent = LOW;
int leftEncoderLast = leftEncoderCurrent;

bool moved = 0;
bool rotationComplete = 0;
bool movementComplete = 1;

int rightCount = 0;
int leftCount = 0;
double theta;

double wheelRadius = 0.075; //meters
double wheelDistance = 0.31; //meters 
double v_l;
double v_r;
double loopTime = 10;

double rho_e;
double phi_e;

double current_rho = 0;
double current_phi = 0;

double target_rho = 0.5;    //meters
double target_phi = 45;   //Degrees

int directionRight;
int directionLeft;
double rightPower;
double leftPower;



double encoderSteps = 16*50;

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
  attachInterrupt(digitalPinToInterrupt(LENCA),rightRotation,RISING);

  theta = (360)/encoderSteps;

}

void loop() {
  start_time = millis();
  digitalWrite(pwmPower, HIGH);
  double Kp = 10;

  if (rotationComplete == 0){
    Serial.println("im in rotation");
    current_phi = (rightCount - leftCount)*theta*wheelRadius / wheelDistance;

    /************************************************/  
    // PI controller configuration
    /************************************************/
//      rho_e = target_rho - current_rho;
      phi_e = target_phi - current_phi;
//      Kp = 25;
      

    /************************************************/
    //Motor Control
    /************************************************/

      rightPower = Kp*phi_e;
      rightPower = constrain(rightPower, 0, 255);
      leftPower = Kp*phi_e;
      leftPower = constrain(leftPower, 0, 255);
      

      setMotor(PWMR, rightPower, INR, 1);
      setMotor(PWML, leftPower, INL, 1);
      
      
      if(phi_e < 1 && phi_e > -1){
        movementComplete = 0;
        rotationComplete = 1;
        rightCount = 0;
        leftCount = 0;
      }
  }
  if (movementComplete == 0){
    Serial.println("im in movement");
    current_rho = (rightCount + leftCount)*theta*0.5*wheelRadius;
    rho_e = target_rho - current_rho;

      rightPower = Kp*rho_e;
      rightPower = constrain(rightPower, 0, 255);
      leftPower = Kp*rho_e;
      leftPower = constrain(leftPower, 0, 255);
      

      setMotor(PWMR, rightPower, INR, 1);
      setMotor(PWML, leftPower, INL, 0);
      
      
      if(rho_e < 0.1 && rho_e > -0.1){
        movementComplete = 1;
        Serial.println("finished");
      }
  }


      
//    analogWrite(PWMR, 170);
//    analogWrite(PWML, 170);
//    digitalWrite(INR, HIGH);
//    digitalWrite(INL, HIGH);
    

    
    
//    v_r = rightCount * theta * wheelRadius;   //right velocity
//    v_l = leftCount * theta * wheelRadius;    //left velocity
//
//    theta_r = rightCount * theta;
//    theta_l = leftCount * theta;

  
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

   if((rightEncoderLast == LOW) && (rightEncoderCurrent == HIGH)){
    if(digitalRead(RENCB) == LOW){
      rightCount += 1;
    }
    else{
      rightCount -= 1;
    }
   }
   
}

void leftRotation() {
  leftEncoderCurrent = digitalRead(LENCA);

   if((leftEncoderLast == LOW) && (leftEncoderCurrent == HIGH)){
    if(digitalRead(LENCB) == LOW){
      leftCount += 1;
    }
    else{
      leftCount -= 1;
    }
   }
}
