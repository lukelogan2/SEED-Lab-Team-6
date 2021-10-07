#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM 9       //PWM wave for motor power
#define IN1 7       //Motor Direction Control
#define IN2 8       //Motor Direction Control
#define pwmPower 4  //Motor Power

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
  Serial.begin(115200);         //Serial Baud rate
  pinMode(pwmPower,OUTPUT);     //initialize pwmPower pin for output
  pinMode(ENCA,INPUT);          //set encoder pin A as an input
  pinMode(ENCB,INPUT);          //set encoder pin B as input
  attachInterrupt(digitalPinToInterrupt(ENCA),rightRotation,RISING);    //create interrupt for rising edge on channel A of encoder
  
  pinMode(PWM,OUTPUT);          //set PWM pin to an output
  pinMode(IN1,OUTPUT);          //set IN1 pin to an output
  pinMode(IN2, OUTPUT);         //set IN2 pin to an output

  //create a fixed theta which is encoder dependent, this is the amount "stepped" every movement
  theta = 360.0/encoderSteps;   
  
}

void loop() {
  /*************************************************************/
  // Receive Serial Input from Raspberry Pi and set the target angle accordingly
  //the values are chosen arbitrarily
  /*************************************************************/
  if (DataRead) {     //read if the flag DataRead is set true
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
    DataRead = false;     //clear the DataRead flag
  }
  /*************************************************************/  

  double loop_time_start = millis();      //poll the loop to get an initialized time, currently unused but useful later
  
  /*************************************************************/
  //Determine the power and direction of the motor and create the control system
  /*************************************************************/
  digitalWrite(pwmPower,HIGH);            //start the motor power supply
  // PID constants
  float kp = 25;                          //proportional constant determined through experimentation

  // Determine the motor power using the proportional controller
  double e = current_angle - target_angle;          //determines the error 
  double pwr = e*kp;                                 //uses the error to set the power
    
 
  // motor direction
  int dir = 0;    //initialize direction each loop
  
  //determine the direction based on the sign of the error
  if(current_angle - target_angle < 0){
    dir = 1;
  }
  if(current_angle - target_angle > 0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);
  /*************************************************************/
  
  //serial print of the current angle
  if(moved == 1){
    Serial.println(current_angle);
    moved = 0;
  }

  double loop_time_end = millis();    //end poll of the loop time
}
/*************************************************************/
//The function managing the motor control
/*************************************************************/
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);    //writes the pwm wave to the motor power
  if(dir == 1){                     //sets pins so rotation is clockwise
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){              //sets pins so direction is counter clockwise
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
  }  
}
/*************************************************************/
//ISR for determining position using the encoder
/*************************************************************/
void rightRotation() {
if(old_time == millis()){                    //quits the ISR if electricl noise triggers the ISR too fast
  return; 
}
  clkPinRightCurrent = digitalRead(ENCA);    //read the state of the encoder channel A


//Check where the encoder channel B is to check whether it moved clockwise or counter clockwise
  if((clkPinRightLast == LOW) && (clkPinRightCurrent == HIGH)) {
    if(digitalRead(ENCB) == LOW) {
      rightCount = 1;                                   //the current angle increased for clockwise
    }
    else {
      rightCount = -1;                                  //the current angle decreased for counter clockwise
    }
  }
  old_time = millis();                                  //poll the time to get ISR timing
  current_angle  = current_angle + rightCount*theta;    //calculate the new current angle using the old one
  clkPinRightLast = LOW;                                //set the last state as the current to restart the loop
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
