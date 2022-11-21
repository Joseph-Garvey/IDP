const int rline = 4;
const int lline = 3;
int right_line;
int left_line;
void setup() {
  Serial.begin(9600);
  pinMode(rline, INPUT_PULLUP);
  pinMode(lline, INPUT_PULLUP);
}

void loop() {
  right_line = digitalRead(rline);
  left_line = digitalRead(lline);
  Serial.print("Right line status   ");
  Serial.println(right_line);
  Serial.print("Left line status    ");
  Serial.println(left_line);
  delay(1000);
}
