/// Libraries
#include <Adafruit_MotorShield.h>
// README
// Pin Definitions
#define IRPin A0
#define Line_Left_Sensor A1
#define Line_Right_Sensor A2
#define Proximity_Front_LED 8
// README
// Left Line Sensor = A1
// Right Line Sensor = A2
// IR Front Sensor = A3
// Left Motor = Port 3
// Right Motor = Port 4
// Front Proximity = 13

/// Variable Declaration
const int max_speed_delta = 80;
int slow;
const int fast = 255;
float Line_Left;
float Line_Right;
int Line_Threshold = 60;
float IR_Front;
int IR_Threshold = 350;
uint8_t Left_Motor_Speed = slow;
uint8_t Right_Motor_Speed = slow;
int cycles_deviated = 0;
int cycles_max = 5000;

/// Motor Shield Setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the motor shield object with the default I2C address
Adafruit_DCMotor *Left_Motor = AFMS.getMotor(1); // Connect Left Motor as Port 1
Adafruit_DCMotor *Right_Motor = AFMS.getMotor(2); // And Right to Port 2

/// INITIAL SETUP
void setup() {
  Serial.begin(9600);
  pinMode(IRPin, INPUT);
  pinMode(Proximity_Front_LED, OUTPUT);
  // Ensure Motor Shield is connected to Arduino.
  while(!AFMS.begin()){
        Serial.println("Could not find Motor Shield. Check wiring.");
  }
  // Set forward motor direction.
  // TODO Affix labels to the motors to ensure port consistency, ensure "FORWARD" in code is forward on bot.
  Left_Motor->run(BACKWARD);
  Right_Motor->run(BACKWARD);
}

/// Reads Line Sensors, Outputs to Serial
void ReadLineSensor(){
  Line_Left = analogRead(Line_Left_Sensor);
  Line_Right = analogRead(Line_Right_Sensor);
  //Serial.println("L / R Line Sensors");
  Serial.print(Line_Left);
  Serial.print(" ");
  Serial.print(Line_Threshold);
  Serial.print(" ");
  /// this should be somewhere else
  // IR_Front = analogRead(IRPin);
  // Serial.print(IR_Front);
  // Serial.print(" ");
  Serial.print(cycles_deviated);
  Serial.print(" ");
  Serial.print(slow);
  Serial.print(" ");
  ///
  Serial.println(Line_Right);
  //Serial.println("L/R Sensor Output");
}

// ENCAPSULATE MOTOR IN CLASS TO REDUCE CODE

void Move_Straight(){
  //Serial.print("STRAIGHT");
  cycles_deviated = 0;
  if(Left_Motor_Speed != fast) { 
    Left_Motor->setSpeed(fast); 
    Left_Motor_Speed = fast;
  }
  if(Right_Motor_Speed != fast){
    Right_Motor->setSpeed(fast); 
    Right_Motor_Speed = fast;
  }
}

void Move_Left(){
  //Serial.print("LEFT");
  if(Left_Motor_Speed != slow) { 
    Left_Motor->setSpeed(slow); 
    Left_Motor_Speed = slow;
  }
  if(Right_Motor_Speed != fast){
    Right_Motor->setSpeed(fast); 
    Right_Motor_Speed = fast;
  }
}

void Move_Right(){
  //Serial.print("RIGHT");
  if(Left_Motor_Speed != fast) { 
    Left_Motor->setSpeed(fast); 
    Left_Motor_Speed = fast;
  }
  if(Right_Motor_Speed != slow){
    Right_Motor->setSpeed(slow); 
    Right_Motor_Speed = slow;
  }
}

void Calc_Turning_Rate(){
  cycles_deviated += 1;
  slow = 150;
  //slow = fast - (max_speed_delta*cycles_deviated/cycles_max);
}
void Move_Lost(){
  cycles_deviated = cycles_max;
  // if(Left_Motor_Speed == fast && Right_Motor_Speed == slow){

  // }
  // else if(){

  // }
  // else {
  //   // what happened??
  // }
}

void loop() {
  ReadLineSensor();
  if(Line_Left > Line_Threshold && Line_Right > Line_Threshold) { Move_Straight(); }
  else if (Line_Left > Line_Threshold && Line_Right < Line_Threshold){ Move_Left(); }
  else if (Line_Left < Line_Threshold && Line_Right > Line_Threshold){ Move_Right(); }
  else { Move_Lost(); }
  // TODO test if setting motor speed here or in the function is faster? Use of two variable versus one
  Left_Motor -> setSpeed(Left_Motor_Speed);
  Right_Motor -> setSpeed(Right_Motor_Speed);
  if(IR_Front > IR_Threshold){ digitalWrite(Proximity_Front_LED, HIGH); }
  else { digitalWrite(Proximity_Front_LED, LOW); }
}