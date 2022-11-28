// Pin Definitions
#define IR_Front_Sensor A0 // Pin for block sensor on front (Analog).
#define Line_Left_Sensor A1 // Pin for Left Line Sensor (Analog).
#define Line_Right_Sensor A2 // Pin for Right Sensor (Analog).
#define IR_Left_Sensor A3 // Pin for block sensor on Left side (Analog).
#define LDR_Tunnel_Sensor 2 // Pin for detecting if the robot is entering the tunnel.
const int button_enable = 3;
const int LED_HighDensity = 6; // RED LED.
#define LED_Moving 7 // Flashing LED.
const int LDR_Grabber = 8; // Sensor for block detection.
const int Grabber = 9; // Pin for Grabber Servo
const int LED_LowDensity = 11; // GREEN LED.
#define Junction_Left_Sensor 12 // Pin for Junction on Left (Digital).
#define Junction_Right_Sensor 13 // Pin for Junction on Right (Digital).

int block_type;

void setup() {
  Serial.begin(9600);
  pinMode(LDR_Grabber,INPUT_PULLUP);
  pinMode(LED_HighDensity, OUTPUT);
  pinMode(LED_LowDensity, OUTPUT);
  digitalWrite(LED_HighDensity, LOW);
  digitalWrite(LED_LowDensity, HIGH);
}

void loop() {
    bool enabled = digitalRead(button_enable);
    if(!digitalRead(button_enable)){
        block_type = digitalRead(LDR_Grabber);
        Serial.print("Block type:  ");
        if(block_type == HIGH){
            // LOW DENSITY
            digitalWrite(LED_LowDensity, HIGH);
            digitalWrite(LED_HighDensity, LOW);
            Serial.println("LOW DENSITY");
        }
        else{
            // HIGH DENSITY
            digitalWrite(LED_LowDensity, LOW);
            digitalWrite(LED_HighDensity, HIGH);
            Serial.println("HIGH DENSITY");
        }
  }
  else{
    digitalWrite(LED_HighDensity, LOW);
    digitalWrite(LED_LowDensity, LOW);
  }
}
