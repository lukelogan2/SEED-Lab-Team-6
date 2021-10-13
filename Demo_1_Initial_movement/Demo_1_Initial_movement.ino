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
int leftEncodeCurrent = LOW;
int leftEncoderLast = leftEncodeCurrent;

bool moved = 0;

int rightCount = 0;
int leftCount = 0;
double theta;
double current_angle = 0;
double target_angle;

double wheelRadius = 0.075; //meters
double wheelDistance = 0.31; //meters 
double v_l;
double v_r;
double loopTime = 10;

double rho_e;
double phi_e;

double current_rho;
double current_phi;

double target_rho = 5;    //meters
double target_phi = PI/4;   //Radians



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
//
//  attachInterrupt(digitalPinToInterrupt(RENCA),rightRotation,RISING);
//  attachInterrupt(digitalPinToInterrupt(LENCA),rightRotation,RISING);

  theta = (2*PI)/encoderSteps;

}

void loop() {
  start_time = millis();
  digitalWrite(pwmPower, HIGH);
  while(run_time - start_time <= loopTime){

    
    
//    v_r = rightCount * theta * wheelRadius;   //right velocity
//    v_l = leftCount * theta * wheelRadius;    //left velocity
//
//    theta_r = rightCount * theta;
//    theta_l = leftCount * theta;

/************************************************/
// PI controller configuration
/************************************************/
  rho_e = target_rho - current_rho;
  phi_e = target_phi - current_phi;

/************************************************/
    //Motor Control
/************************************************/
  analogWrite(PWMR, 170);
  analogWrite(PWML, 170);
  digitalWrite(INR, HIGH);
  digitalWrite(INL, HIGH);



  
    run_time = millis();
  }
}

//void rightRotation() {
//   rightEncoderCurrent = digitalRead(RENCA);
//
//   if((rightEncoderLast == LOW) && (rightEncoderCurrent == HIGH)){
//    if(digitalRead(RENCB) == LOW){
//      rightCount += 1;
//    }
//    else{
//      rightCount -= 1;
//    }
//   }
//   
//}

//void leftRotation() {
//  leftEncoderCurrent = digitalRead(LENCA);
//
//   if((leftEncoderLast == LOW) && (leftEncoderCurrent == HIGH)){
//    if(digitalRead(LENCB) == HIGH){
//      leftCount += 1;
//    }
//    else{
//      leftCount -= 1;
//    }
//   }
//}
