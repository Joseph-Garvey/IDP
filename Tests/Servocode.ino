// Include the Servo library 
#include <Servo.h> 
// Declare the Servo pin 
#define currentservoPin A0
int servoPin = 3; 
float servocurrent;
// Create a servo object 
Servo Grabber_Servo; 
void setup() { 
  Serial.begin(9600);
  pinMode(currentservoPin, INPUT);
   // We need to attach the servo to the used pin number 
   Grabber_Servo.attach(servoPin); 
}
void loop(){ 
   // Make servo go to open  
   Grabber_Servo.write(40); 
   for (int i = 0; i < 5; i++) {
     servocurrent = analogRead(currentservoPin);
     Serial.print("open: ");
     Serial.println(servocurrent);
     delay(1000);
   } 
  // Make servo go to close
   Grabber_Servo.write(85); 
   for (int i = 0; i < 5; i++) {
    servocurrent = analogRead(currentservoPin);
    Serial.print("close: ");
    Serial.println(servocurrent);
    delay(1500); 
