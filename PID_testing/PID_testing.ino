#include <PID_AutoTune_v0.h>
#include <Adafruit_MotorShield.h>
#include <PID_v1.h>

double Setpoint, Input, Output;
double Kp=2, Ki=5, Kd=1;
PID myPIDController(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID_ATune PIDTuner (&Input, &Output);

const int Left_Line_Sensor = A3; //LEFT
const int Right_Line_Sensor = A2; //RIGHT
const int slow = 80;
const int fast = 220;
float Line_Left;
float Line_Right;
float offset = 120;
float threshold = 350.0; // edit this
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
  Left_Motor->run(FORWARD);
  Right_Motor->run(FORWARD);
  Setpoint = 0;
  myPIDController.SetMode(AUTOMATIC);
  myPIDController.SetSampleTime(25);
}

void loop() {
  
  //READ SENSORS
  Line_Left = analogRead(Left_Line_Sensor);
  Line_Right = analogRead(Right_Line_Sensor) + offset;
  Input = Line_Left - Line_Right;
  myPIDController.Compute();
  Serial.print("Left_Sensor:");
  Serial.print(Line_Left);
  Serial.print(",");
  Serial.print("Right_Sensor:");
  Serial.print(Line_Right);
  Serial.print(",");
  Serial.print("input:");
  Serial.print(Input);
  Serial.print(",");
  Serial.print("Output:");
  Serial.println(Output);
}