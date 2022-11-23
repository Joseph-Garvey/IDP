#define Line_Left_Sensor A1
#define Line_Right_Sensor A2
float Line_Left;
float Line_Right;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Line_Right = analogRead(Line_Right_Sensor);
  Line_Left = analogRead(Line_Left_Sensor);
  Serial.print("Left_Sensor:");
  Serial.print(Line_Left);
  Serial.print(",");
  Serial.print("Right_Sensor:");
  Serial.print(Line_Right);
  Serial.print(",");
  Serial.print("Line_Threshold:");
  Serial.println(100);
}