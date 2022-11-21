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
  Left_Motor->run(BACKWARD);
  Right_Motor->run(BACKWARD);
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
      Right_Motor->setSpeed(fast);
    }
    else{
      //left
      Serial.print("LEFT");
      Left_Motor->setSpeed(slow);
      Right_Motor->setSpeed(fast);
    }
  }
  else{
    if(linevolt2 > threshold){
      // right
      Serial.print("right");
      Left_Motor->setSpeed(fast);
      Right_Motor->setSpeed(slow);
    }
    else{
      Serial.print("Lost");
      //this is lost
      //Left_Motor->run(FORWARD);
    }
  }
  */
}