/// Libraries
#include <Adafruit_MotorShield.h>
// README
// Pin Definitions
#define IRPin A0
#define Line_Left_Sensor A1
#define Line_Right_Sensor A2
#define Proximity_Front_LED 8
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
const int max_speed_delta = 50;
int slow;
const int fast = 255;
float Line_Left;
float Line_Right;
int Line_Threshold = 400;
float IR_Front;
int IR_Threshold = 350;
uint8_t Left_Motor_Speed = slow;
uint8_t Right_Motor_Speed = slow;
int cycles_deviated = 0;
//const int LDR_Threshold = 10;
//int intersections = 0;
//const int distance_to_wall
int cycles_max = 3000;

/// Motor Shield Setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the motor shield object with the default I2C address
Adafruit_DCMotor *Motor_Left = AFMS.getMotor(1);    // Connect Left Motor as Port 1
Adafruit_DCMotor *Motor_Right = AFMS.getMotor(2);   // And Right to Port 2

/// INITIAL SETUP
void setup()
{
  Serial.begin(9600);
  // pinMode(IRPin, INPUT);
  // pinMode(Proximity_Front_LED, OUTPUT);
  // Ensure Motor Shield is connected to Arduino.
  while (!AFMS.begin()) 
  {
    Serial.println("Could not find Motor Shield. Check wiring.");
  }
  // Set forward motor direction.
  // TODO Affix labels to the motors to ensure port consistency, ensure "FORWARD" in code is forward on bot.
  Motor_Left->run(BACKWARD);
  Motor_Right->run(BACKWARD);
}

/// Reads Line Sensors, Outputs to Serial
void ReadLineSensor()
{
  Line_Left = analogRead(Line_Left_Sensor);
  Line_Right = analogRead(Line_Right_Sensor);
  // Serial.println("L / R Line Sensors");
  Serial.print("Left_Sensor:");
  Serial.print(Line_Left);
  Serial.print(",");
  Serial.print("Right_Sensor:");
  Serial.print(Line_Right);
  Serial.print(",");
  /// this should be somewhere else
  // IR_Front = analogRead(IRPin);
  // Serial.print(IR_Front);
  // Serial.print(" ");
  //Serial.print(cycles_deviated);
  //Serial.print(" ");
  //Serial.print(slow);
  //Serial.print(" ");
  ///
  Serial.print("Line_Threshold:");
  Serial.println(Line_Threshold);
  // Serial.println("L/R Sensor Output");
}

// ENCAPSULATE MOTOR IN CLASS TO REDUCE CODE

void Move_Straight()
{
  // Serial.print("STRAIGHT");
  cycles_deviated = 0;
  if (Left_Motor_Speed != fast)
  {
    Motor_Left->setSpeed(fast);
    Left_Motor_Speed = fast;
  }
  if (Right_Motor_Speed != fast)
  {
    Motor_Right->setSpeed(fast);
    Right_Motor_Speed = fast;
  }
}

void Move_Left()
{
  Calc_Turning_Rate();
  // Serial.print("LEFT");
  if (Left_Motor_Speed != slow)
  {
    Motor_Left->setSpeed(slow);
    Left_Motor_Speed = slow;
  }
  if (Right_Motor_Speed != fast)
  {
    Motor_Right->setSpeed(fast);
    Right_Motor_Speed = fast;
  }
}

void Move_Right()
{
  Calc_Turning_Rate();
  // Serial.print("RIGHT");
  if (Left_Motor_Speed != fast)
  {
    Motor_Left->setSpeed(fast);
    Left_Motor_Speed = fast;
  }
  if (Right_Motor_Speed != slow)
  {
    Motor_Right->setSpeed(slow);
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
//Analog read
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

//Digital read
//bool LDR = digitalRead(LDR_Sensor)
}

void ReadSideUS()
{
// US_Reading = analogRead(Side_US_Sensor)
//return US_Reading
}

void ReadJLineSensor()
{
/*bool J_Line_Right = digitalRead(J_Line_Right_Sensor)
  bool J_Line_Left = digitalRead(J_Line_Left_Sensor)
  if (J_Line_Right || J_Line_Left)
  {
    intersections += 1;
    delay (1000);
  }*/
}

void TunnelRoutine()
{
  /*ReadSideUS();
    if (US_Reading < distance_to_wall)
    {
      Move_Left();
    }
    else if (US_Reading > distance_to_wall)
    {
      Move_Right();
    }
    else
    {
      Move_Straight();
    }
    Left_Motor->setSpeed(Left_Motor_Speed);
    Motor_Right->setSpeed(Right_Motor_Speed);
    */
}

void loop()
{
/*while (ReadLDR())
  {
    TunnelRoutine();
  }*/ 
  ReadLineSensor();
  if (Line_Left > Line_Threshold && Line_Right > Line_Threshold)
  {
    Move_Straight();
  }
  else if (Line_Left > Line_Threshold && Line_Right < Line_Threshold)
  {
    Move_Left();
  }
  else if (Line_Left < Line_Threshold && Line_Right > Line_Threshold)
  {
    Move_Right();
  }
  else
  {
    Move_Lost();
  }
  // TODO test if setting motor speed here or in the function is faster? Use of two variable versus one
  Motor_Left->setSpeed(Left_Motor_Speed);
  Motor_Right->setSpeed(Right_Motor_Speed);
  // if (IR_Front > IR_Threshold)
  // {
  //   digitalWrite(Proximity_Front_LED, HIGH);
  // }
  // else
  // {
  //   digitalWrite(Proximity_Front_LED, LOW);
  // }
}