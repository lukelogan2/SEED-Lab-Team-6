/* Author: Justin Olson
 * Created 2021
 * 
 * Assignment 2:  Exercise 4 Hardware Implementation Velocity
 * Making a simulated robot and tracking its movement using two
 * mechanical encoders. This simulation allows us to determine robot
 * position based on the movement of the wheels (encoders).
 *
 * 
 * The circuit:
 * -5V power supply to both encoders positive power supply
 * -Board ground to both encoder grounds
 * -Wire from the left clk encoder to digital pin 3 through a 10nF capacitor
 * -wire from the right clk encoder to digital pin 2 through a 10nF capacitor
 * -wire from dt on right encoder to digital pin 8
 * -wire from dt on left encoder to digital pin 9
 * 
 */
#define PI 3.1415926535897932384626433832795
#define loop_time = 10    //loop time of 10 milliseconds


//Defining the pins
const int clkPinRight = 2;

const int dtPinRight = 3;

int motor = 4;
int motordirection1 = 7;
int motorspeed1 = 9;

//setting a baseline for the clk pin and setting the states to
//match
int clkPinRightCurrent = LOW;
int clkPinRightLast = clkPinRightCurrent;


//an initial count which will increase for clockwise
//and decrease for counter clockwise
int rightCount = 0;

//global flag for when to write to output serial out
bool moved = 0;

//dimensions of the robot
double wheelRadius = .5;
double wheelWidth = 0.1;
double encoderSteps = 16*50;
double theta;
double old_time = 0;
double new_time;
double new_time_seconds;
double v_r;
double v_l;

  int start_time = 0;
  int run_time = 0;
  


void setup() {
    Serial.begin(115200);       //initialize serial monitor
  
    pinMode(clkPinRight, INPUT);      //Set the right clk pin as input

    pinMode(dtPinRight, INPUT);

    pinMode(motor, OUTPUT);
    pinMode(motordirection1, OUTPUT);
    pinMode(motorspeed1, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(clkPinRight), rightRotation, RISING);


  theta = 2*PI / encoderSteps;
  run_time = millis();
}


void loop() {
  long initial_time = millis();
  if(millis() - run_time < 3000){
  if(start_time == 0){
    while (millis() - initial_time < 1000){
      digitalWrite(motor,HIGH);
      digitalWrite(motordirection1,LOW);
      analogWrite(motorspeed1,0);
    }
    start_time = 1;
  }

    analogWrite(motorspeed1, 128);

  if(moved == 1){
    Serial.print(new_time_seconds,4);
    Serial.print("\t");
    //Serial.print(v_l);
    //Serial.print("\t");
    Serial.println(v_r,4);
    moved = 0;
    v_r = 0;
    //v_l = 0;
  }
  }
  else{
    analogWrite(motorspeed1,0);
  }
}
  

void rightRotation() {
 if(old_time == millis()){
  return;
 }
  clkPinRightCurrent = digitalRead(clkPinRight);    //read the state of the right clk pin


//Check where the right dt pin is to check whether it moved clockwise or counter clockwise
  if((clkPinRightLast == LOW) && (clkPinRightCurrent == HIGH)) {
    if(digitalRead(dtPinRight) == LOW) {
      rightCount = -1;    //increase count for clockwise
    }
    else {
      rightCount = 1;    //decrease count for counter clockwise
    }
  }
  new_time = millis();
  double time_diff_r = (double)(new_time - old_time)/1000.0;
//   Serial.println(time_diff_r);
  v_r = rightCount * theta / time_diff_r;
  old_time = new_time;
  clkPinRightLast = LOW;   //set the last state as the current to restart the loop
  moved = 1;    //set flag to begin printing data
  new_time_seconds = new_time / 1000.0;
}
