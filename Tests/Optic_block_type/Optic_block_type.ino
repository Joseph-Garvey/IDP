const int block_sensor = 2;
const int green_LED = 6;
const int red_LED = 7;
int block_type;

void setup() {
  Serial.begin(9600);
  pinMode(block_sensor,INPUT_PULLUP);
}

void loop() {
  block_type = digitalRead(block_sensor);
  Serial.print("Block type:  ");
  if(block_type == HIGH){
    digitalWrite(green_LED, HIGH);
    digitalWrite(red_LED, LOW);
    Serial.println("LOW DENSITY");
  }
  else{
    digitalWrite(green_LED, LOW);
    digitalWrite(red_LED, HIGH);
    Serial.println("HIGH DENSITY");
  }
  delay(2000);
}
