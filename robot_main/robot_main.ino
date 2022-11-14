/// Libraries
#include <Adafruit_MotorShield.h>

// README
// Left Line Sensor = A1
// Right Line Sensor = A2
// Left Motor = Port 1
// Right Motor = Port 2

/// Variable Declaration
const int slow = 50;
const int fast = 250;
float Line_Left;
float Line_Right;
float Line_Threshold = 20.0;
uint8_t Left_Motor_Speed = slow;
uint8_t Right_Motor_Speed = slow;

/// Motor Shield Setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the motor shield object with the default I2C address
Adafruit_DCMotor *Left_Motor = AFMS.getMotor(1); // Connect Left Motor as Port 1
Adafruit_DCMotor *Right_Motor = AFMS.getMotor(2); // And Right to Port 2

/// INITIAL SETUP
void setup() {
  Serial.begin(9600);
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
  Line_Left = analogRead(A1);
  Line_Right = analogRead(A2);
  //Serial.println("L / R Line Sensors");
  Serial.print(Line_Left);
  Serial.print(" ");
  Serial.print(Line_Threshold);
  Serial.print(" ");
  Serial.println(Line_Right);
  //Serial.println("L/R Sensor Output");
}

// ENCAPSULATE MOTOR IN CLASS TO REDUCE CODE

void Move_Straight(){
  //Serial.print("STRAIGHT");
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

void Move_Lost(){
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
  //if(Left_Motor_Speed )
  Left_Motor -> setSpeed(Left_Motor_Speed);
  Right_Motor -> setSpeed(Right_Motor_Speed);
}
