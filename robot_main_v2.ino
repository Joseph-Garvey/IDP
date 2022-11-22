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
// Tunnel_LDR_Sensor =
// Grabber_LDR_Sensor =

/// Variable Declaration
const int max_speed_delta = 150;
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
// const int LDR_Threshold = 10;
int intersection = 0;
int doubleintersection = 0;
// const int distance_to_wall
int cycles_max = 3000;
bool clockwise = true;
int Desired_Intersection;
bool grabbed = false;
int junction_iteration = 0;

/// Motor Shield Setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the motor shield object with the default I2C address
Adafruit_DCMotor *Left_Motor = AFMS.getMotor(3);    // Connect Left Motor as Port 1
Adafruit_DCMotor *Right_Motor = AFMS.getMotor(4);   // And Right to Port 2

/// INITIAL SETUP
void setup()
{
  Serial.begin(9600);
  pinMode(IRPin, INPUT);
  pinMode(Proximity_Front_LED, OUTPUT);
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
void ReadLineSensor()
{
  Line_Left = analogRead(Line_Left_Sensor);
  Line_Right = analogRead(Line_Right_Sensor);
  // Serial.println("L / R Line Sensors");
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
  // Serial.println("L/R Sensor Output");
}

bool ReadTunnelLDR()
{
  // Digital read
  // bool Tunnel_LDR = digitalRead(Tunnel_LDR_Sensor)
}

void ReadSideUS()
{
  // US_Reading = analogRead(Side_US_Sensor)
  // return US_Reading
}

void ReadJLineSensor()
{
  bool J_Line_Right = digitalRead(J_Line_Right_Sensor)
  bool J_Line_Left = digitalRead(J_Line_Left_Sensor)
}

void CountJunctions()
{
  ReadJLineSensor();
  if (J_Line_Right && J_Line_Left)
  {
    doubleintersection += 1
    delay(1000);
  }
  else if (J_Line_Right || J_Line_Left && clockwise)
  {
    intersection += 1;
    delay(1000);
  }
  else if (J_Line_Right || J_Line_Left && clockwise == false)
  {
    intersection -= 1;
    delay(1000);
  }
}

// ENCAPSULATE MOTOR IN CLASS TO REDUCE CODE

void Move_Stop()
{
  Left_Motor->setSpeed(0)
  Right_Motor->setSpeed(0)
}

void Move_Straight()
{
  Left_Motor->run(FORWARD)
  Right_Motor->run(FORWARD)
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

void Move_Backwards()
{
  Left_Motor->run(BACKWARDS)
  Right_Motor->run(BACKWARDS)
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
  Left_Motor->run(FORWARD)
  Right_Motor->run(FORWARD)
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
  Left_Motor->run(FORWARD)
  Right_Motor->run(FORWARD)
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

void Calc_Turning_Rate()
{
  cycles_deviated += 1;
  // slow = 150;
  slow = fast - (max_speed_delta * cycles_deviated / cycles_max);
}

void Move_Lost()
{
  /*cycles_deviated = cycles_max;
  slow = 0
  if(Left_Motor_Speed == fast && Right_Motor_Speed == slow)
  {
    Right_Motor->setSpeed(slow);
    Right_Motor_Speed = slow;
  }
  else if(Left_Motor_Speed == slow && Right_Motor_Speed == fast)
  {
    Left_Motor->setSpeed(slow);
    Left_Motor_Speed = slow;
  }
  else
  {
    Left_Motor->run(BACKWARD)
    Right_Motor->run(BACKWARD)
  }*/
}

void Move_ACW()
{
  Left_Motor->run(BACKWARD);
  Right_Motor->run(FORWARD);
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

void Move_CW()
{
  Left_Motor->run(FORWARD);
  Right_Motor->run(BACKWARD);
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

void GrabberRoutine()
{
  if (grabbed)
  {
    // Open
  }
  else
  {
    // Close
  }
}

void DropRoutine()
{
  Move_Straight();
  delay(500):
  while (J_Line_Left != true && J_Line_Right != true)
  {
    ReadJLineSensor();
    Move_Straight();
  }
  Move_Stop();
  // drop
  Move_Backwards;
  delay(500);
  while (J_Line_Left != true && J_Line_Right != true)
  {
    ReadJLineSensor();
    Move_Backwards();  
  }
  Move_Stop();

void JunctionRoutine()
{
  ReadJLineSensor();
  if (J_Line_Right && J_Line_Left && grabbed != true && doubleintersection == 2)
  {
    while (J_Line_Right != true)
    {
      ReadJLineSensor();
      Move_CW();
    }
  }
  else if (J_Line_Right && J_Line_Left && grabbed != true)
  {
    while (J_Line_Right != true)
    {
      ReadJLineSensor();
      Move_ACW();
    }
  }
  else if (J_Line_Right && J_Line_Left && grabbed)
  {
    while (J_Line_Left != true)
    {
      ReadJLineSensor();
      Move_CW();
    }
  }
  else if (J_Line_Right)
  {
    while(J_Line_Left != true)
    {
      ReadJLineSensor();
      Move_CW();
    }
  }
  else if (J_Line_Left)
  {
    while(J_Line_Right != true)
    {
      ReadJLineSensor();
      Move_ACW();
    }
  }
  Move_Stop();
}

void NormalRoutine()
{
  ReadLineSensor();
  ReadJLineSensor();
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
  Left_Motor->setSpeed(Left_Motor_Speed);
  Right_Motor->setSpeed(Right_Motor_Speed);
  if (IR_Front > IR_Threshold)
  {
    digitalWrite(Proximity_Front_LED, HIGH);
  }
  else
  {
    digitalWrite(Proximity_Front_LED, LOW);
  }
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
    Right_Motor->setSpeed(Right_Motor_Speed);
    */
}

void loop()
{
  while (ReadTunnelLDR())
  {
    TunnelRoutine();
  }
  CountJunctions();
  if (doubleintersection == 0)
  {
    Move_Straight();
  }
  if (doubleintersection == 2)
  {
    Move_Stop();
    JunctionRoutine();
  }
  if (grabbed && intersection == Desired_Intersection)
    {
      Move_Stop();
      JunctionRoutine();
      DropRoutine();
    }
  NormalRoutine();
}