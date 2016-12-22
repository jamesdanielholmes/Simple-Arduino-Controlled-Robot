// Simple demo to blink and LED on pins 9 and 10, 
// This sketch can also be used to pulse two motors via an L298N motor controller

void setup() {  // Setup runs once per reset

  // set all pins connecting to the motor as outputs
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);

  // when controlling the motor, these pins tell the motors to go forwards
  digitalWrite(2, HIGH);
  digitalWrite(3, LOW);
  digitalWrite(4, HIGH);
  digitalWrite(5, LOW);      

}

void loop() {
  digitalWrite(9, HIGH);    // set both motors to run!
  digitalWrite(10, HIGH);   

  delay(500);               // wait 0.5 seconds
  
  digitalWrite(9, LOW);     // stop both motors
  digitalWrite(10, LOW);         

  delay(500);               // wait 0.5 seconds
}

