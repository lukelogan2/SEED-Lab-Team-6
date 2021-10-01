
#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM 9
#define IN1 7
#define IN2 8
#define pwmPower 4
volatile int posi = 0; 
long prevT = 0;
float eprev = 0;
float eintegral = 0;

//Encoder variables
/*******************************************/
double new_time = 0;
double old_time = 0;
double clkPinRightCurrent = LOW;
double clkPinRightLast = clkPinRightCurrent;
int rightCount = 0;
bool moved = 0;
double encoderSteps = 16*50;
double theta;
double current_angle = 0;
double target_angle = 0;

// Serial Input from Pi 
String data;
bool DataRead;

void setup() {
  Serial.begin(115200);
  pinMode(pwmPower,OUTPUT);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),rightRotation,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2, OUTPUT);

  theta = 360.0/encoderSteps;
  Serial.println("target pos");
}

void loop() {
  // Receive Serial Input from Raspberry Pi
  if (DataRead) {
    Serial.print("You sent me: ");
    Serial.println(data);
    if (data == "1") {
      Serial.println("Quadrant I");
      target_angle = 0;
    }
    else if (data == "2") {
      Serial.println("Quadrant II");
      target_angle = 90;
    }
    else if (data == "3") {
      Serial.println("Quadrant III");
      target_angle = 180;
    }
    else if (data == "4") {
      Serial.println("Quadrant IV");
      target_angle = 270;
    }
    DataRead = false;
  }
  
  double loop_time_start = millis();
  digitalWrite(pwmPower,HIGH);
  // PID constants
  float kp = 25;

  // motor power
  double e = current_angle - target_angle;
  double pwr = e*kp;
    
  int dir = 0;
  // motor direction
  if(current_angle - target_angle < 0){
    dir = 1;
  }
  if(current_angle - target_angle > 0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);
  //setMotor(pwr,PWM);
  
  if(moved == 1){
    Serial.println(current_angle);
    moved = 0;
  }
  //  Serial.print(" ");
  //  Serial.print(pos);
  //  Serial.println();
  double loop_time_end = millis();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
  }  
}

//void setMotor(double pwrVal, int pwmPin){
//  analogWrite(pwmPin,pwrVal);
////  digitalWrite(in1, HIGH);
//}

void rightRotation() {
if(old_time == millis()){
  return; 
}
  clkPinRightCurrent = digitalRead(ENCA);    //read the state of the right clk pin


//Check where the right dt pin is to check whether it moved clockwise or counter clockwise
  if((clkPinRightLast == LOW) && (clkPinRightCurrent == HIGH)) {
    if(digitalRead(ENCB) == LOW) {
      rightCount = 1;    //increase count for clockwise
    }
    else {
      rightCount = -1;    //decrease count for counter clockwise
    }
  }
  old_time = millis();
  current_angle  = current_angle + rightCount*theta;
  clkPinRightLast = LOW;   //set the last state as the current to restart the loop
  moved = 1;    //set flag to begin printing data
}

// Serial Communication Handler
void serialEvent() {
  if (Serial.available() > 0) {
    data = Serial.readStringUntil('\n');
    DataRead = true;
  }
  Serial.flush();
}
