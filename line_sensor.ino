const int linereflect2 = A2;
const int linereflect3 = A3;
float Line_Right;
float linevolt3;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Line_Right = analogRead(linereflect2);
  linevolt3 = analogRead(linereflect3);
  Serial.print("Line sensor 2 reading: ");
  Serial.print(Line_Right);
  Serial.print(" Line sensor 3 reading: ");
  Serial.println(linevolt3);
  delay(1000);
}