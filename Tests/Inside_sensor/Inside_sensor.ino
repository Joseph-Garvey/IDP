int inside;
void setup() {
  Serial.begin(9600);
  pinMode(2,INPUT);
}

void loop() {
  inside = digitalRead(2);
  if (inside == HIGH){
    Serial.println("Inside");
  }
  else{
    Serial.println("Outside");
  }
  delay(750);
}
