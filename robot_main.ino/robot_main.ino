/// Libraries
#include <Adafruit_MotorShield.h>

// README
// Left Line Sensor = A1
// Right Line Sensor = A2
// Left Motor = Port 1
// Right Motor = Port 2

/// Variable Declaration
const int slow = 100;
const int fast = 250;
float Line_Left;
float Line_Right;
int robotmovement;
float Line_Threshold = 40.0;

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
  Line_Left = analogRead(PIN_A1);
  Line_Right = analogRead(PIN_A2);
  //Serial.println("L / R Line Sensors");
  Serial.print(Line_Left);
  Serial.print(" ");
  Serial.print(Line_Threshold);
  Serial.print(" ");
  Serial.println(Line_Right);
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
  ReadLineSensor();
   
  //NAVIGATE
  if(Line_Left > Line_Threshold && Line_Right > Line_Threshold){
      //if(Left_Motor.speed) to avoid too many unnecessary commands
      if (robotmovement != 1){
        straightrobot();
      }
      robotmovement = 1;
  }
  else if (Line_Left > Line_Threshold && Line_Right < Line_Threshold){
      //left change to right
      if (robotmovement != 2){
        rightrobot();
      }
      robotmovement = 2;
    }
  
  else if (Line_Left < Line_Threshold && Line_Right > Line_Threshold){
 
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
