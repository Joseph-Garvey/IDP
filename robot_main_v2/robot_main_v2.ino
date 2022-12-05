/// README ///
// Naming Conventions: [INPUT/OUTPUT]_[Position-Optional]_[Descriptor]
// Descriptors will be of the form "Sensor" for the pinouts, "Reading" for sensor outputs and "Threshold".

/// Libraries
// Referenced here without dependencies.
// Dependencies must be installed for libraries to function.
#include <Adafruit_MotorShield.h> // https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library
#include <Servo.h> // https://www.arduino.cc/reference/en/libraries/servo/

/// Arduino Pinouts
#define IR_Front_Sensor A0 // Used for block detection. (Analog IN).
#define Line_Left_Sensor A3 // Line Tracking (Analog IN).
#define Line_Right_Sensor A2 // Line Tracking (Analog IN).
#define IR_Left_Sensor A1 // Block detection perpendicular to robot path. (Analog IN).
#define LDR_Tunnel_Sensor 0 // Detects whether robot has entered tunnel. (Digital IN).
#define PushButton 1 
#define LED_HighDensity 6 // RED LED. Activated when High Density Block is detected. (Digital OUT).
#define LED_Moving 7 // Amber Flashing LED. HIGH when Robot is in motion. (Digital OUT).
#define LDR_Grabber 8 // Distinguishes between HIGH and LOW density blocks. (Digital IN PULLUP).
#define Grabber 9 // Servo output. Connected to the grabber mechanism. (PWM Digital, connected to Motor Shield)
#define LED_LowDensity 11 // Green LED. Activated when Low Density Block is detected. (Digital OUT).
#define Junction_Left_Sensor 12 // HIGH when junction is detected on left. (Digital IN).
#define Junction_Right_Sensor 13 // HIGH when junction is deteced on right. (Digital IN).

/// Variable Declaration
// Sensor Outputs
float Line_Left_Reading; // Analog Output from Left Line Sensor.
float Line_Right_Reading; // Analog Output from Right Line Sensor.
bool Junction_Left_Reading; // HIGH when Junction detected.
bool Junction_Right_Reading; // HIGH when Junction detected.
const int Line_Threshold = 175; // Analog value above which the sensor is considered to be on the line.
float IR_Front_Reading; // Analog output from Front IR.
float IR_Left_Reading; // Analog output from Left IR.
const int Line_sensoroffset = 25; // Added to the right line sensor to account for sensor variance.
const int IR_Front_Threshold = 10; // Distance below which the robot detects a block in front of it.
const int IR_Left_Threshold = 5; // Distance below which the robot detects a block to the side.
const int IR_Grab_Threshold = 5; // Distance at which to position the robot away from the block before grabbing.
// note when calibrating Front Threshold: set to distance from junction 2 to middle block + some kind of additional distance for leeway

// Motor Variables and Constants
const int max_speed_delta = 200; // Sets the difference in speed between the two motors.
int slow; // The current slowest motor speed.
const int fast = 255; // Motor at full speed.
uint8_t Left_Motor_Speed = slow;
uint8_t Right_Motor_Speed = slow;
int cycles_deviated = 0; // Measure of how long robot has deviated from line.
int cycles_max = 5000; // Sets max T at which robot will turn at its fastest rate.
Servo Grabber_Servo; 
const int grabber_closed = 85; // Sets fully closed position of grabber.
const int grabber_open = 40; // Sets fully open position of grabber.

// Navigation Logic
int intersection = 0; // Count of intersections passed in this loop of table.
int doubleintersection = 0; // Count of intersections detected on both sides of bot at the same time in this loop of the table.
const int distance_to_wall = 3; // The distance the robot should maintain from the wall of the tunnel.
// Note that distance to wall is calibrated by placing the robot in the tunnel and reading the side IR sensor.
// Whilst this could be done in a more intelligent way, it wouldn't provide any benefit to this particular task, and would add unnecessary complexity.
bool clockwise = true;
int Block_Dropoff_Location; // Sets the junction at which the robot should dropoff the carried block.
bool grabbed = false; // TRUE if robot is transporting a block.
bool JunctionDetected = false;
bool BlockDetected_Front = false;
bool BlockDetected_Left = false;

/// Motor Shield Setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the motor shield object with the default I2C address
Adafruit_DCMotor *Motor_Left = AFMS.getMotor(1);    // Connect Left Motor as Port 1
Adafruit_DCMotor *Motor_Right = AFMS.getMotor(2);   // And Right to Port 2

/// Establishes pinouts and connects Motor Shield.
void setup()
{
    Serial.begin(9600);
    // Establish Connections
    pinMode(LDR_Grabber, INPUT_PULLUP); // Enable LDR as Pullup Input.
    pinMode(LED_HighDensity, OUTPUT); 
    pinMode(LED_LowDensity, OUTPUT);
    pinMode(LED_Moving, OUTPUT);
    Grabber_Servo.attach(Grabber); // Connect Servo object to Arduino Output.
    // Ensure motor shield is connected before proceding. 
    while (!AFMS.begin()) { Serial.println("Could not find Motor Shield. Check wiring."); }
    Serial.println("Motor Shield Connected. Checking Motor Connections.");
    // Line enabled when electrical or mechanical changes made.
    // Test_Connections();
}

/// Used to test electrical connections after electrical or mechnanical changes are made. 
/// Tests: DC Motors connected, Servo actuates, Block detection functions.
void Test_Connections(){
    // DC Motor Test //
    // Run Motors in different directions at full speed.
    bool motors_connected = false;
    Motor_Left->run(BACKWARD);
    Motor_Right->run(FORWARD);
    Motor_Left->setSpeed(fast);
    Motor_Right->setSpeed(fast);
    // While user has not confirmed that the motors are working as expected, wait for user input.
    while(!motors_connected) {
        Serial.println(("Right Motor spinning forward, Left spinning backward. Check connections and type 'y' to proceed."));
        String command = Serial.readStringUntil('\n');
        // Once user confirms, stop motors and continue.
        if (command == "y"){
            motors_connected = true;
            Motor_Left->run(RELEASE);
            Motor_Right->run(RELEASE);
        }
        delay(1000);
    }
    // Grabber Test //
    bool grabbers_calibrated = false;
    // Actuate grabbers to open, wait for user to confirm correct function.
    // Actuate again to close, wait for user to confirm correct function.
    while (!grabbers_calibrated) {
        Grabber_Servo.write(grabber_open);
        Serial.println(("Grabber is open. Check and type 'y' to proceed."));
        if (Serial.read() == 'y')
        {
            Grabber_Servo.write(grabber_closed);
            Serial.println(("Grabber is closed. Check and type 'y' to proceed."));
            {
                if (Serial.read() == 'y')
                {
                    grabbers_calibrated = true;
                }
            }
        }
        delay(1000);
    }
    // Block Detection Test //
    bool block_detection_passed = false;
    // Manually test the block detection, if user confirms working, proceed.
    while (!block_detection_passed) 
    { 
        DetectDensityRoutine();
        if(Serial.read() == 'y'){
            block_detection_passed = true;
        }
    }
    Block_Dropoff_Location = 500; // reset block dropoff.
    // Ideally line sensors should also be tested.
}

/// Reads Line Sensors and plots sensor output, motor speeds and line deviation time on Serial Plotter.
void ReadLineSensor()
{
    Line_Left_Reading = analogRead(Line_Left_Sensor);
    Line_Right_Reading = analogRead(Line_Right_Sensor) + Line_sensoroffset; // Add offset to ensure sensors have the same threshold.
    Serial.print("Left_Sensor:");
    Serial.print(Line_Left_Reading);
    Serial.print(",");
    Serial.print("Right_Sensor:");
    Serial.print(Line_Right_Reading);
    Serial.print(",");
    Serial.print("Line_Threshold:");
    Serial.print(Line_Threshold);
    Serial.print(",");
    Serial.print("Cycles_Deviated:");
    Serial.print(cycles_deviated);
    Serial.print(",");
    Serial.print("Slow_MotorSpeed:");
    Serial.println(slow);
}

/// Returns TRUE if robot is in the tunnel, outputs this value to Serial.
bool ReadTunnelLDR()
{
    // Digital read 
    bool Tunnel_LDR = digitalRead(LDR_Tunnel_Sensor);
    Serial.print("Tunnel_LDR:");
    Serial.println(Tunnel_LDR);
}

/// Reads Front IR Sensor, determines if there is something in front of the robot.
void ReadFrontIR()
{
    IR_Front_Reading = analogRead(IR_Front_Sensor);
    // Whilst this value will be true due to other obstacles
    // its result is ignored unless the robot is in a section of the table
    // where it would expect to find a block.
    // Hence, false positives aren't an issue here.
    BlockDetected_Front = (IR_Front_Reading < IR_Front_Threshold);
}

/// Reads Left IR Sensor, determines if there is a block to the side of the robot. Outputs the Side IR Reading to Serial.
void ReadLeftIR()
{
    IR_Left_Reading = analogRead(IR_Left_Sensor);
    Serial.print("IR_LEFT:");
    Serial.println(IR_Left_Reading);
    BlockDetected_Left = (IR_Left_Reading < IR_Left_Threshold);
}

/// Updates Junction detection bools.
void ReadJLineSensor()
{
    Junction_Right_Reading = digitalRead(Junction_Right_Sensor);
    Junction_Left_Reading = digitalRead(Junction_Left_Sensor);
}

/// Updates the current intersections passed by the robot.
void CountJunctions()
{
    ReadJLineSensor();
    if (Junction_Right_Reading && Junction_Left_Reading)
    {
        doubleintersection += 1;
        delay(1000);
    }
    else if (Junction_Right_Reading || Junction_Left_Reading && clockwise)
    {
        bool JunctionDetected = true;
        intersection += 1;
        delay(1000);
    }
    else if (Junction_Right_Reading || Junction_Left_Reading && clockwise == false)
    {
        bool JunctionDetected = true;
        intersection -= 1;
        delay(1000);
    }
    else
    {
        bool JunctionDetected = false;
    }
}

// ENCAPSULATE MOTOR IN CLASS TO REDUCE CODE
#pragma region Move_Commands
// Robot stops current movement.
void Move_Stop()
{
    cycles_deviated = 0; // reset time away from line.
    Motor_Left->setSpeed(0);
    Motor_Right->setSpeed(0);
}
void Move_Straight()
{
    Motor_Left->run(FORWARD);
    Motor_Right->run(FORWARD);
    // Avoids sending excessive commands over I2C to the motor shield.
    cycles_deviated = 0; // reset time away from line.
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

void Move_Backwards()
{
    Motor_Left->run(BACKWARD);
    Motor_Right->run(BACKWARD);
    cycles_deviated = 0; //reset time away from line.
    // Avoids sending excessive commands over I2C to the motor shield.
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
    Motor_Left->run(FORWARD);
    Motor_Right->run(FORWARD);
    Calc_Turning_Rate(); // Turn at a rate proportional to time since the robot was moving straight along the line.
    // Avoids sending excessive commands over I2C to the motor shield.
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
    Motor_Left->run(FORWARD);
    Motor_Right->run(FORWARD);
    Calc_Turning_Rate(); // Turn at a rate proportional to time since the robot was moving straight along the line.
    // Avoids sending excessive commands over I2C to the motor shield.
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

void Move_ACW()
{
    Motor_Left->run(BACKWARD);
    Motor_Right->run(FORWARD);
    // Avoids sending excessive commands over I2C to the motor shield.
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

// The robot will start moving clockwise at full speed.
void Move_CW()
{
    Motor_Left->run(FORWARD);
    Motor_Right->run(BACKWARD);
    // Avoids sending excessive commands over I2C to the motor shield.
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

/// Calculates turning rate proportional to time spent away from line (capped at a limit set by cycles_max and max_speed_delta)
void Calc_Turning_Rate()
{
    // nb this script uses processor cycles, so will be dependent on code complexity rather than absolute time.
    // this does however greatly simplify this calculation, and so was deemed a reasonable compromise as processor load
    // will be approximately the same in any situation where this function is called.
    cycles_deviated += 1;
    // slow = 150;
    slow = fast - (max_speed_delta * cycles_deviated / cycles_max);
}

void Move_Lost()
{
    if(Junction_Left_Reading){
        Move_Left();
    }
    else if (Junction_Right_Reading)
    {
        Move_Right();
    }
    else if (Left_Motor_Speed != fast)
    {
        Move_Left();
    }
    else if (Right_Motor_Speed != fast)
    {
        Move_Right();
    }
    
    
}
#pragma endregion

// Reads the Grabber LDR and determines whether the block is of HIGH or LOW density, sets an appropraiate destination for block deposit.
void DetectDensityRoutine()
{
    bool block_type = digitalRead(LDR_Grabber);
    if (block_type == HIGH) // THe block is low density.
    {
        Serial.println("LOW DENSITY");
        Block_Dropoff_Location = 1;
    }
    else // The block is high density.
    {
        Serial.println("HIGH DENSITY");
        Block_Dropoff_Location = 4;
    }
}

/// Grab the block currently in front of the robot and return to position at end of function call.
void GrabRoutine(){
    // Moves forward until the block is immediately in front of the robot.
    ReadFrontIR();
    while (IR_Front_Reading > IR_Grab_Threshold)
    {
        ReadFrontIR();
        Move_Straight();
    }
    delay(100);
    // Stop in front of the block and close the grabber.
    Move_Stop();
    Grabber_Servo.write(85);
    grabbed = true;
    ReadJLineSensor();
    // Reverse until the robot is back on the line.
    while (Junction_Left_Reading != true && Junction_Right_Reading != true)
    {
        ReadJLineSensor();
        Move_Backwards();
    }
    Move_Stop();
}

/// Drops the block in the box and returns to position at end of function call.
void DropRoutine()
{
    // Move a short distance off of the main line going around the table.
    Move_Straight();
    delay(100);
    // Move until the edge of the box is reached.
    while (Junction_Left_Reading != true && Junction_Right_Reading != true)
    {
        ReadJLineSensor();
        Move_Straight();
    }
    Move_Stop();
    // Drop the block here.
    Grabber_Servo.write(40); // open
    grabbed = false;
    // Reverse a short distance so that the junction sensors aren't on the box.
    Move_Backwards;
    delay(500);
    // Continue to reverse until the robot is back on the line.
    while (Junction_Left_Reading != true && Junction_Right_Reading != true)
    {
        ReadJLineSensor();
        Move_Backwards();
    }
    Move_Stop();
}

void JunctionRoutine()
{
    ReadJLineSensor();
    if (Junction_Right_Reading && Junction_Left_Reading && grabbed != true && doubleintersection == 2)
    {
        while (Junction_Right_Reading != true)
        {
            ReadJLineSensor();
            Move_CW();
        }
    }
    else if (Junction_Right_Reading && Junction_Left_Reading && grabbed != true)
    {
        while (Junction_Right_Reading != true)
        {
            ReadJLineSensor();
            Move_ACW();
        }
    }
    else if (Junction_Right_Reading && Junction_Left_Reading && grabbed)
    {
        while (Junction_Left_Reading != true)
        {
            ReadJLineSensor();
            Move_CW();
        }
    }
    else if (Junction_Right_Reading)
    {
        while (Junction_Left_Reading != true)
        {
            ReadJLineSensor();
            Move_CW();
        }
    }
    else if (Junction_Left_Reading)
    {
        while (Junction_Right_Reading != true)
        {
            ReadJLineSensor();
            Move_ACW();
        }
    }
    Move_Stop();
}

/// This routine keeps the robot tracking the line on the table surface.
void NormalRoutine()
{
    ReadJLineSensor();
    if (Line_Left_Reading > Line_Threshold && Line_Right_Reading > Line_Threshold)
    {
        Move_Straight();
    }
    else if (Line_Left_Reading > Line_Threshold && Line_Right_Reading < Line_Threshold)
    {
        Move_Left();
    }
    else if (Line_Left_Reading < Line_Threshold && Line_Right_Reading > Line_Threshold)
    {
        Move_Right();
    }
    else
    {
        //Move_Lost();
    }
}

/// When robot is in tunnel, this subroutine maintains the robots distance to the wall of the tunnel, keeping it moving in a straight line.
void TunnelRoutine()
{
    ReadLeftIR();
    if (IR_Left_Reading > distance_to_wall)
    {
        Move_Left();
    }
    else if (IR_Left_Reading < distance_to_wall)
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
    bool LoopOn = digitalRead(PushButton);
    while (!LoopOn)
    {
        bool LoopOn = digitalRead(PushButton);
    }
    ReadLineSensor();
    NormalRoutine();
    // Tunnel Routine
    while (ReadTunnelLDR() && (Line_Left_Reading < Line_Threshold) && (Line_Right_Reading < Line_Threshold))
    {
        TunnelRoutine();
    }
    Serial.print("Left_MotorSpeed:");
    Serial.println(Left_Motor_Speed);
    Serial.print("Right_MotorSpeed:");
    Serial.println(Right_Motor_Speed);
    CountJunctions(); // Count single intersections and double intersections
    ReadLeftIR();
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
    if (grabbed && intersection == Block_Dropoff_Location)
    {
        // Dropping Routine
        Move_Stop();
        JunctionRoutine();
        DropRoutine();
    }
    if (!grabbed && JunctionDetected && intersection == 2)
    {
        ReadFrontIR();
        if (BlockDetected_Front)
        {
            //forward grab routine
            while (IR_Front_Reading > 2)
            {
                ReadFrontIR();
                Move_Straight();
                Move_Stop();
                Grabber_Servo.write(85); // grab
                grabbed = true;
            }
        }
    }
    if (!grabbed && BlockDetected_Left)
    {
        if (intersection == 2)
        {
            ReadFrontIR();
            if (BlockDetected_Front)
            {
                //forward grab routine
                while (IR_Front_Reading > 2)
                {
                    ReadFrontIR();
                    Move_Straight();
                    Move_Stop();
                    Grabber_Servo.write(85); // grab
                    grabbed = true;
                }
            }
        }
        else if (intersection == 3)
        {
            //side grab routine
            Move_Stop();
            JunctionRoutine();
            GrabRoutine();       // Goes forward, grabs the block, goes backwards. Also sets if grabber = true or false
            DetectDensityRoutine(); // Determines density and sets Block_Dropoff_Location
            JunctionRoutine();      // Turns back to track
        }
    }
    NormalRoutine();
}