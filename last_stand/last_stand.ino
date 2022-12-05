/// Libraries
#include <Adafruit_MotorShield.h>
#include <Servo.h>
// README
// Pin Definitions
#define IR_Front_Sensor A0 // Pin for block sensor on front (Analog).
#define Line_Left_Sensor A1 // Pin for Left Line Sensor (Analog).
#define Line_Right_Sensor A2 // Pin for Right Sensor (Analog).
#define IR_Left_Sensor A3 // Pin for block sensor on Left side (Analog).
#define LDR_Tunnel_Sensor 2 // Pin for detecting if the robot is entering the tunnel.
#define LED_HighDensity 6 // RED LED.
#define LED_Moving 7 // Flashing LED.
#define LDR_Grabber 8 // Sensor for block detection.
#define Grabber 9 // Pin for Grabber Servo
#define LED_LowDensity 11 // GREEN LED.
#define Junction_Left_Sensor 12 // Pin for Junction on Left (Digital).
#define Junction_Right_Sensor 13 // Pin for Junction on Right (Digital).
#define Button_On 0
#define Flashing_Light 1
/// Variable Declaration
// Sensors
float Line_Left_Reading;
float Line_Right_Reading;
bool Junction_Left_Reading;
bool Junction_Right_Reading;
int Line_Threshold = 60;
float IR_Front_Reading;
float IR_Left_Reading;
int IR_Threshold = 350;
// const int LDR_Threshold = 10;

// Motors
const int max_speed_delta = 50;
int slow;
int turn = 150;
const int fast = 255;
uint8_t Left_Motor_Speed = slow;
uint8_t Right_Motor_Speed = slow;
int cycles_deviated = 0;
int cycles_max = 3000;
// Logic
int intersection = 0;
int doubleintersection = 0;
const int distance_to_wall = 3;

int Desired_Intersection = 500;
bool grabbed = false;
int junction_iteration = 0;
bool JunctionDetected = false;
bool BlockDetected_Front = false;
bool BlockDetected_Left = false;
bool loopon;
int rotatingtime1 = 500;
int rotatingtime2 = 1500;
int forwardtime1 = 3000;
int forwardtime2 = 3000;

Servo Servo1;

/// Motor Shield Setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the motor shield object with the default I2C address
Adafruit_DCMotor *Motor_Left = AFMS.getMotor(1);    // Connect Left Motor as Port 1
Adafruit_DCMotor *Motor_Right = AFMS.getMotor(2);   // And Right to Port 2

/// INITIAL SETUP
void setup()
{
    Serial.begin(9600);
    // Set forward motor direction.v
    pinMode(1, OUTPUT);
    while (!AFMS.begin())
    {
        Serial.println("Could not find Motor Shield. Check wiring.");
    }
    Serial.println("Motor Shield Connected. Checking Motor Connections.");
    Motor_Left->run(FORWARD);
    Motor_Right->run(FORWARD);
    //Test_Connections();
}

/// Test Suites
void Test_Connections(){
    // Ensure Motor Shield is connected to Arduino.
    while (!AFMS.begin())
    {
        Serial.println("Could not find Motor Shield. Check wiring.");
    }
    Serial.println("Motor Shield Connected. Checking Motor Connections.");
    bool motors_connected = false;
    Motor_Left->run(BACKWARD);
    Motor_Right->run(FORWARD);
    Motor_Left->setSpeed(fast);
    Motor_Left->setSpeed(fast);
    while(!motors_connected)
    {
        Serial.println(("Right Motor spinning forward, Left spinning backward. Check connections and type 'y' to proceed."));
        String command = Serial.readStringUntil('\n');
        if (command == "y"){
            motors_connected = true;
        }
    }
    // line following sensors
    // Then test grabber
    // bool grabbers_calibrated = false;
    // while (!grabbers_calibrated)
    // {
    //     Servo1.write(40);
    //     Serial.println(("Grabber is open. Check and type 'y' to proceed."));
    //     if (Serial.read() == 'y')
    //     {
    //         Servo1.write(85);
    //         Serial.println(("Grabber is closed. Check and type 'y' to proceed."));
    //         {
    //             if (Serial.read() == 'y')
    //             {
    //                 grabbers_calibrated = true;
    //             }
    //         }
    //     }
    // }
    // // test optical sensor
    // bool optics_functioning = false;
    // while (!optics_functioning)
    // {
    //     block_type = digitalRead(LDR_Grabber);
    //     if (block_type == HIGH){ Serial.println("LOW DENSITY"); }
    //     else { Serial.println("HIGH DENSITY"); }
    // }
}

void Move_Stop()
{
  Serial.print("STOP");
    Motor_Left->setSpeed(0);
    Motor_Right->setSpeed(0);
}

void Move_Straight()
{
  Serial.println("STRAIGHT");
    Motor_Left->run(FORWARD);
    Motor_Right->run(FORWARD);
    Motor_Left->setSpeed(fast);
    Motor_Right->setSpeed(fast);
}

void Move_CW()
{
  Serial.print("cw");
    Motor_Left->run(FORWARD);
    Motor_Right->run(BACKWARD);
    Motor_Left->setSpeed(turn);
    Motor_Right->setSpeed(turn);
}

void Move_ACW()
{
  Serial.print("acw");
    Motor_Left->run(BACKWARD);
    Motor_Right->run(FORWARD);
    Motor_Left->setSpeed(turn);
    Motor_Right->setSpeed(turn);
}

void loop()
{
    //loopon = digitalRead(Flip_Flop_Circuit);
    //if (loopon)
    //{
      digitalWrite(1, LOW);
      delay(5000);
      digitalwrite(1, HIGH);
      Move_Stop();
      digitalwrite(1, LOW);
      delay(100);
      Move_ACW();
      delay(4000);
      Move_Stop();
      Move_Straight();
      digitalwrite(1, HIGH);
      delay(9000);
      Move_CW();
      delay(1700);
      Move_Straight();
      delay(10000);
      Move_Stop();
      Move_CW();
      delay(700);
      Move_Straight();
      delay(10000);
      Move_Stop();
      Move_ACW();
      delay(2000);
      Move_Straight();
      delay(20000);
      digitalwrite(1, low;
    //}
}