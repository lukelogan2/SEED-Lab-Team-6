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
double encoder_count = 0;

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

long start_time = 0;
int run_time = 0;
double sample_time = 5; // 5 ms


void setup() {
  Serial.begin(115200);       //initialize serial monitor

  pinMode(clkPinRight, INPUT);      //Set the right clk pin as input

  pinMode(dtPinRight, INPUT);

  pinMode(motor, OUTPUT);
  pinMode(motordirection1, OUTPUT);
  pinMode(motorspeed1, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(clkPinRight), rightRotation, RISING);


  theta = 2*PI / encoderSteps;
  digitalWrite(motor,HIGH);
  digitalWrite(motordirection1,LOW);
  analogWrite(motorspeed1,0);
  delay(1);
  analogWrite(motorspeed1, 128);
  start_time = millis();
}


void loop() {
  while (millis() - start_time < 2000) {
    long initial_time = millis();
    while (millis() - initial_time < sample_time) {
      // Spin wait
    }
    v_r = (encoder_count * theta / sample_time) * 1000;
    //Serial.print("Velocity: ");
    Serial.print(v_r);
    Serial.print("\t");
    Serial.println(millis());
    encoder_count = 0;
  }
  analogWrite(motorspeed1,0);
}
  

void rightRotation() {
  clkPinRightCurrent = digitalRead(clkPinRight);    //read the state of the right clk pin


//Check where the right dt pin is to check whether it moved clockwise or counter clockwise
  if((clkPinRightLast == LOW) && (clkPinRightCurrent == HIGH)) {
    if(digitalRead(dtPinRight) == LOW) {
      encoder_count -= 1;    //increase count for clockwise
    }
    else {
      encoder_count += 1;    //decrease count for counter clockwise
    }
  }
  clkPinRightLast = LOW;   //set the last state as the current to restart the loop
  moved = 1;    //set flag to begin printing data
}
