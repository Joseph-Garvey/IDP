#include <Adafruit_MotorShield.h>

const int linereflect1 = A2; //LEFT
const int linereflect2 = A3; //RIGHT
const int slow = 100;
const int fast = 250;
float linevolt1;
float linevolt2;
int robotmovement;
float threshold = 40.0; // edit this
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
void ReadSensor(){
  linevolt1 = analogRead(linereflect1);
  linevolt2 = analogRead(linereflect2);
  Serial.print("Line sensor 2 reading: ");
  Serial.print(linevolt1); //right
  Serial.print(" Line sensor 3 reading: ");
  Serial.println(linevolt2); // left
}

void straightrobot(){
  Serial.print("STRAIGHT");
  Left_Motor->setSpeed(fast);
  Right_Motor->setSpeed(fast);
}

void leftrobot(){
  Serial.print("LEFT");
  Left_Motor->setSpeed(slow);
  Right_Motor->setSpeed(fast);
}

void rightrobot(){
  Serial.print("RIGHT");
  Left_Motor->setSpeed(fast);
  Right_Motor->setSpeed(slow);
}

/*void lostrobot(){
  Left_Motor->run(FORWARD);
  Serial.print("Lost");
}
*/
void loop() {
  ReadSensor();
   
  //NAVIGATE
  if(linevolt1 > threshold && linevolt2 > threshold){
      //if(Left_Motor.speed) to avoid too many unnecessary commands
      if (robotmovement != 1){
        straightrobot();
      }
      robotmovement = 1;
  }
  else if (linevolt1 > threshold && linevolt2 < threshold){
      //left change to right
      if (robotmovement != 2){
        rightrobot();
      }
      robotmovement = 2;
    }
  
  else if (linevolt1 < threshold && linevolt2 > threshold){
 
      // right change to left
      if (robotmovement != 2){
        rightrobot();
      }
      robotmovement = 2;
    }
//    else{
//      Serial.print("Lost");
//      //this is lost
//      //Left_Motor->run(FORWARD);
//    }
//  }
  
}
