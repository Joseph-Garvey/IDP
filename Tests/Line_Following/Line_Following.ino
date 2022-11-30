#include <Adafruit_MotorShield.h>

const int Left_Line_Sensor = A3; //LEFT
const int Right_Line_Sensor = A2; //RIGHT
const int slow = 100;
const int fast = 255;
float Line_Left;
float Line_Right;
float offset = 20;
float threshold = 350.0; // edit this
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *Left_Motor = AFMS.getMotor(1);
Adafruit_DCMotor *Right_Motor = AFMS.getMotor(2);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!AFMS.begin()){
        Serial.println("Could not find Motor Shield. Check wiring.");
  }
  Left_Motor->setSpeed(100);
  Right_Motor->setSpeed(100);
  Left_Motor->run(FORWARD);
  Right_Motor->run(FORWARD);
}

void loop() {
  
  //READ SENSORS
  Line_Left = analogRead(Left_Line_Sensor);
  Line_Right = analogRead(Right_Line_Sensor) + offset;
  Serial.print("Threshold:");
  Serial.print(threshold);
  Serial.print(",");
  Serial.print("Left_Sensor:");
  Serial.print(Line_Left);
  Serial.print(",");
  Serial.print("Right_Sensor:");
  Serial.println(Line_Right);
  //NAVIGATE
  if(Line_Left > threshold){
    if(Line_Right > threshold){
      //Serial.print("STRAIGHT");
      //if(Left_Motor.speed) to avoid too many unnecessary commands
      Left_Motor->setSpeed(fast);
      Right_Motor->setSpeed(fast);
    }
    else{
      //left
      //Serial.print("LEFT");
      Left_Motor->setSpeed(slow);
      Right_Motor->setSpeed(fast);
    }
  }
  else{
    if(Line_Right > threshold){
      // right
      //Serial.print("right");
      Left_Motor->setSpeed(fast);
      Right_Motor->setSpeed(slow);
    }
    else{
      //Serial.print("Lost");
      //this is lost
      //Left_Motor->run(FORWARD);
    }
  }
  
}