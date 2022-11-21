const int block_sensor = 2;
int block_type;
void setup() {
  Serial.begin(9600);
  pinMode(block_sensor,INPUT_PULLUP);
}

void loop() {
  block_type = digitalRead(block_sensor);
  Serial.print("Block type:  ");
  if(block_type == HIGH){
    Serial.println("LOW DENSITY");
  }
  else{
    Serial.println("HIGH DENSITY");
  }
  delay(2000);
}
