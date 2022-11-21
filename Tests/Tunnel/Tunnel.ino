#define IRPin A0
#include <Adafruit_MotorShield.h>

/// Libraries
#include <Adafruit_MotorShield.h>
// README
// Pin Definitions
#define IRPin A0
#define Line_Left_Sensor A1
#define Line_Right_Sensor A2
#define LDR 7
#define LED_Tunnel 8
// #define J_Line_Left_Sensor
// #define J_Line_Right_Sensor
// #define Side_US_Sensor 
// #define LDR_Sensor 
// README
// Left Line Sensor = A1
// Right Line Sensor = A2
// IR Front Sensor = A3
// Left Motor = Port 3
// Right Motor = Port 4
// Front Proximity = 13
// Side_US_Sensor =
// LDR_Sensor =

/// Variable Declaration
// Motor Shield Setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the motor shield object with the default I2C address
Adafruit_DCMotor *Left_Motor = AFMS.getMotor(1);    // Connect Left Motor as Port 1
Adafruit_DCMotor *Right_Motor = AFMS.getMotor(2);   // And Right to Port 2
const int max_speed_delta = 150;
int slow;
const int fast = 255;
uint8_t Left_Motor_Speed = slow;
uint8_t Right_Motor_Speed = slow;
int cycles_deviated = 0;
int cycles_max = 3000;
// IR
float IR_Tunnel_Initial;
int IR_Threshold = 350;
// Nav
bool Tunnel_Entered = 0;
/// INITIAL SETUP
void setup()
{
  Serial.begin(9600);
  pinMode(IRPin, INPUT);
  pinMode(LDR, INPUT);
  pinMode(LED_Tunnel, OUTPUT);
  // Ensure Motor Shield is connected to Arduino.
  while (!AFMS.begin()) 
  {
    Serial.println("Could not find Motor Shield. Check wiring.");
  }
  // Set forward motor direction.
  // TODO Affix labels to the motors to ensure port consistency, ensure "FORWARD" in code is forward on bot.
  Left_Motor->run(BACKWARD);
  Right_Motor->run(BACKWARD);
}

/// Reads Line Sensors, Outputs to Serial
float ReadIRSensor()
{
  float IR = analogRead(IRPin);
  Serial.printLn(IR);
  return IR;
}

void Move_Straight()
{
  // Serial.print("STRAIGHT");
  cycles_deviated = 0;
  if (Left_Motor_Speed != fast)
  {
    Left_Motor->setSpeed(fast);
    Left_Motor_Speed = fast;
  }
  if (Right_Motor_Speed != fast)
  {
    Right_Motor->setSpeed(fast);
    Right_Motor_Speed = fast;
  }
}

void Move_Left()
{
  Calc_Turning_Rate();
  // Serial.print("LEFT");
  if (Left_Motor_Speed != slow)
  {
    Left_Motor->setSpeed(slow);
    Left_Motor_Speed = slow;
  }
  if (Right_Motor_Speed != fast)
  {
    Right_Motor->setSpeed(fast);
    Right_Motor_Speed = fast;
  }
}

void Move_Right()
{
  Calc_Turning_Rate();
  // Serial.print("RIGHT");
  if (Left_Motor_Speed != fast)
  {
    Left_Motor->setSpeed(fast);
    Left_Motor_Speed = fast;
  }
  if (Right_Motor_Speed != slow)
  {
    Right_Motor->setSpeed(slow);
    Right_Motor_Speed = slow;
  }
}

void Calc_Turning_Rate(){
  cycles_deviated += 1;
  //slow = 150;
  slow = fast - (max_speed_delta*cycles_deviated/cycles_max);
}

void Move_Lost()
{
  cycles_deviated = cycles_max;
  // if(Left_Motor_Speed == fast && Right_Motor_Speed == slow){

  // }
  // else if(){

  // }
  // else {
  //   // what happened??
  // }
}

bool ReadLDR()
{
  return digitalRead(LDR_);
/*LDR_Reading = analogRead(LDR_Sensor)
if (LDR_Reading > LDR_Threshold)
{
  bool LDR = false;
}
else
{
  bool LDR = true;
}
return LDR*/

}

void loop()
{
  if(ReadLDR()){
    digitalWrite(LED_Tunnel, HIGH);
    if(Tunnel_Entered == false){
      Tunnel_Entered = true;
      IR_Tunnel_Initial = ReadIRSensor;
      Move_Straight();
    }
    else{
      if(ReadIRSensor() < IR_Tunnel_Initial){
        Move_Left();
      }
      else if(ReadIRSensor > IR_Tunnel_Initial){
        Move_Right();
      }
      else {
        Move_Straight();
      }
    }
  }
  else{ digitalWrite(LED_Tunnel, LOW); }
  // TODO test if setting motor speed here or in the function is faster? Use of two variable versus one
  Left_Motor->setSpeed(Left_Motor_Speed);
  Right_Motor->setSpeed(Right_Motor_Speed);
}


