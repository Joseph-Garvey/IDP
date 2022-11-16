// Define model and input pin:
#define IRPin A0

float reflection;
float distance_cm;

void setup() {
  Serial.begin(9600);
  pinMode(IRPin,INPUT);
}

void loop() {
reflection = analogRead(IRPin);
Serial.println(reflection);
delay(1000);
}
