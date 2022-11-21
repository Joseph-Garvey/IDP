const int line_sensor = 4;
int prev_line_stat = LOW;
int line_stat;
int counter = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(line_sensor,INPUT_PULLUP);
}

void loop() {
  line_stat = digitalRead(line_sensor);
  if(line_stat != prev_line_stat){
    counter += 1;
  }
  prev_line_stat = line_stat;
  Serial.println(counter);
  }
