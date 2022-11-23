#include <Adafruit_MotorShield.h>

const int linereflect1 = A2; //LEFT
const int linereflect2 = A3; //RIGHT
const int slow = 100;
const int fast = 150;
float Line_Left;
float Line_Right;
float threshold = 80.0; // edit this
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *Motor_Left = AFMS.getMotor(1);
Adafruit_DCMotor *Motor_Right = AFMS.getMotor(2);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!AFMS.begin()){
        Serial.println("Could not find Motor Shield. Check wiring.");
  }
  Motor_Left->setSpeed(100);
  Motor_Right->setSpeed(100);
  Motor_Left->run(BACKWARD);
  Motor_Right->run(BACKWARD);
}

void loop() {
  /*
  //READ SENSORS
  linevolt1 = analogRead(linereflect1);
  linevolt2 = analogRead(linereflect2);
  Serial.print("Line sensor 2 reading: ");
  Serial.print(linevolt1);
  Serial.print(" Line sensor 3 reading: ");
  Serial.println(linevolt2);
  //NAVIGATE
  if(linevolt1 > threshold){
    if(linevolt2 > threshold){
      Serial.print("STRAIGHT");
      //if(Left_Motor.speed) to avoid too many unnecessary commands
      Left_Motor->setSpeed(fast);
      Motor_Right->setSpeed(fast);
    }
    else{
      //left
      Serial.print("LEFT");
      Left_Motor->setSpeed(slow);
      Motor_Right->setSpeed(fast);
    }
  }
  else{
    if(linevolt2 > threshold){
      // right
      Serial.print("right");
      Left_Motor->setSpeed(fast);
      Motor_Right->setSpeed(slow);
    }
    else{
      Serial.print("Lost");
      //this is lost
      //Left_Motor->run(FORWARD);
    }
  }
  */
}