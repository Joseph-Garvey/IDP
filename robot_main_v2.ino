/// Libraries
#include <Adafruit_MotorShield.h>
#include <Servo.h>
// README
// Pin Definitions
#define Front_IR_Sensor A0
#define Line_Left_Sensor A1
#define Line_Right_Sensor A2
#define Proximity_Front_LED 9
#define currentservopin 3
#define J_Line_Left_Sensor 12
#define J_Line_Right_Sensor 13
#define Side_IR_Sensor A3
#define Tunnel_LDR_Sensor 6
#define block_sensor 8
// README
// Left Line Sensor = A1
// Right Line Sensor = A2
// IR Front Sensor = A3
// Left Motor = Port 3
// Right Motor = Port 4
// Front Proximity = 13
// Side_IR_Sensor =
// Tunnel_LDR_Sensor =
// Grabber_LDR_Sensor =

/// Variable Declaration
const int max_speed_delta = 150;
int slow;
const int fast = 255;
float Line_Left;
float Line_Right;
bool J_Line_Left;
bool J_Line_Right;
int Line_Threshold = 60;
float IR_Front;
int IR_Threshold = 350;
uint8_t Left_Motor_Speed = slow;
uint8_t Right_Motor_Speed = slow;
int cycles_deviated = 0;
// const int LDR_Threshold = 10;
int intersection = 0;
int doubleintersection = 0;
const int distance_to_wall = 3;
int cycles_max = 3000;
bool clockwise = true;
int Desired_Intersection = 500;
bool grabbed = false;
int junction_iteration = 0;
bool JunctionDetected = false;
bool FrontBlockDetected = false;
bool SideBlockDetected = false;

int servoPin 3

Servo Servo1;

/// Motor Shield Setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the motor shield object with the default I2C address
Adafruit_DCMotor *Left_Motor = AFMS.getMotor(3);    // Connect Left Motor as Port 1
Adafruit_DCMotor *Right_Motor = AFMS.getMotor(4);   // And Right to Port 2

/// INITIAL SETUP
void setup()
{
    Serial.begin(9600);
    // Enable Pins
    pinMode(Front_IR_Sensor, INPUT);
    pinMode(Proximity_Front_LED, OUTPUT);
    pinMode(Side_IR_Sensor, INPUT);
    pinMode(block_sensor, INPUT_PULLUP);
    pinMode(currentservoPin, INPUT);
    // We need to attach the servo to the used pin number 
    Servo1.attach(servoPin); 
    // Ensure Motor Shield is connected to Arduino.
    while (!AFMS.begin()) { Serial.println("Could not find Motor Shield. Check wiring."); }
    // Sanity Checks for initial testing (Find a way to disable these for restarts)
    Serial.println("Motor Shield Connected. Checking Motor Connections.");
    Serial.println(("Right Motor spinning forward. Check connection and type 'y' to proceed."));
    if (Serial.read("y"))
    // Set forward motor direction.
    Left_Motor->run(BACKWARD);
    Right_Motor->run(BACKWARD);
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
        String command = Serial.readStringUntil('/n/');
        if (command = "y")
        {
            motors_connected = true;
        }
    }
    Motor_Left->setSpeed(0);
    Motor_Right->setSpeed(0);
    // Then test grabber
    bool grabbers_calibrated = false;
    while (!grabbers_calibrated)
    {
        Servo1.write(40);
        Serial.println(("Grabber is open. Check and type 'y' to proceed."));
        String command = Serial.readStringUntil('/n/');
        if (command = "y")
        {
            Servo.write(85);
            Serial.println(("Grabber is closed. Check and type 'y' to proceed."));
            {
                String command = Serial.readStringUntil('/n/');
                if (command = "y")
                {
                    grabbers_calibrated = true;
                }
            }
        }
    }
    // test optical sensor
    bool optics_functioning = false;
    while (!optics_functioning)
    {
    block_type = digitalRead(LDR_Grabber);
    if (block_type == HIGH)
    {
        Serial.println("LOW DENSITY");
    }
    else
    {
        Serial.println("HIGH DENSITY");
    }
    }
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
    bool Tunnel_LDR = digitalRead(Tunnel_LDR_Sensor)
}

void ReadFrontIR()
{
    Front_IR_Reading = anologRead(Front_IR_Sensor)
    if (Front_IR_Reading < 10) // distance from junction 2 to middle block + some kind of additional distance for leeway
    {
        FrontBlockDetected = true
    }
    else
    {
        FrontBlockDetected = false
    }
}

void ReadSideIR()
{
    Side_IR_Reading = analogRead(Side_IR_Sensor)
    if (Side_IR_Reading < 5)
    {
        SideBlockDetected = true
    }
    else
    {
        SideBlockDetected = false
    }
}

void ReadJLineSensor()
{
    J_Line_Right = digitalRead(J_Line_Right_Sensor)
    J_Line_Left = digitalRead(J_Line_Left_Sensor)
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
        bool JunctionDetected = true
        intersection += 1;
        delay(1000);
    }
    else if (J_Line_Right || J_Line_Left && clockwise == false)
    {
        bool JunctionDetected = true
        intersection -= 1;
        delay(1000);
    }
    else
    {
        bool JunctionDetected = False
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
    slow = 0 if (Left_Motor_Speed == fast && Right_Motor_Speed == slow)
    {
        Right_Motor->setSpeed(slow);
        Right_Motor_Speed = slow;
    }
    else if (Left_Motor_Speed == slow && Right_Motor_Speed == fast)
    {
        Left_Motor->setSpeed(slow);
        Left_Motor_Speed = slow;
    }
    else
    {
        Left_Motor->run(BACKWARD)
            Right_Motor->run(BACKWARD)
    }
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

void DetectDensityRoutine()
{
    block_type = digitalRead(block_sensor);
    Serial.print("Block type:  ");
    if (block_type == HIGH)
    {
        Desired_Intersection = 1;
    }
    else
    {
        Desired_Intersection = 4;
    }
}

void GrabRoutine(){
    // To change: Move forwards until front IR sensor reads near 0, Move backwards until J line sensors read junction
    ReadFrontIR();
    while (Front_IR_Reading > 2)
    {
        ReadFrontIR();
        Move_Straight();
    }
    Move_Stop();
    Servo1.write(85) // close
    grabbed = true
    ReadJLineSensor();
    while (J_Line_Left != true && J_Line_Right != true)
    {
        ReadJLineSensor();
        Move_Backwards();
    }
    Move_Stop();
}

void DropRoutine()
{
    Move_Straight();
    delay(500);
    while (J_Line_Left != true && J_Line_Right != true)
    {
        ReadJLineSensor();
        Move_Straight();
    }
    Move_Stop();
    Servo1.write(40) // open
    grabbed = false
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
        while (J_Line_Left != true)
        {
            ReadJLineSensor();
            Move_CW();
        }
    }
    else if (J_Line_Left)
    {
        while (J_Line_Right != true)
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
    ReadSideIR();
    if (Side_IR_Reading < distance_to_wall)
    {
        Move_Left();
    }
    else if (Side_IR_Reading > distance_to_wall)
    {
        Move_Right();
    }
    else
    {
        Move_Straight();
    }
}

void loop()
{
    // Tunnel Routine
    while (ReadTunnelLDR())
    {
        TunnelRoutine();
    }
    CountJunctions(); // Count single intersections and double intersections
    ReadSideIR();
    if (doubleintersection == 0)
    {
        // Initial Movement
        Move_Straight();
    }
    if (doubleintersection == 2)
    {
        // Initial Turn
        Move_Stop();
        JunctionRoutine();
    }
    if (grabbed && intersection == Desired_Intersection)
    {
        // Dropping Routine
        Move_Stop();
        JunctionRoutine();
        DropRoutine();
    }
    if (not grabbed && JunctionDetected && intersection == 2)
    {
        ReadFrontIR();
        if (FrontBlockDetected)
        {
            //forward grab routine
            while (Front_IR_Reading > 2)
            {
                ReadFrontIR();
                Move_Straight();
                Move_Stop();
                Servo1.write(85) // grab
                grabbed = true
            }
        }
    }
    if (not grabbed && SideBlockDetected)
    {
        if (intersection == 2)
        {
            ReadFrontIR();
            if (FrontBlockDetected)
            {
                //forward grab routine
                while (Front_IR_Reading > 2)
                {
                    ReadFrontIR();
                    Move_Straight();
                    Move_Stop();
                    Servo1.write(85) // grab
                    grabbed = true
                }
            }
        }
        else if (intersection == 3)
        {
            //side grab routine
            Move_Stop();
            JunctionRoutine();
            GrabRoutine();       // Goes forward, grabs the block, goes backwards. Also sets if grabber = true or false
            DetectDensityRoutine(); // Determines density and sets Desired_Intersection
            JunctionRoutine();      // Turns back to track
        }
    }

    }
    NormalRoutine();
}
// Things to do
// Fix Global variable declarations, all messed up right now
// Set up all sensors in setup function
// Add the Open and close function for the grabbers and then add that function to the Routines
// Add a LED, LED must be flashing 2Hz when moving