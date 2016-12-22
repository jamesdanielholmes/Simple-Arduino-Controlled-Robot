/* This code allows the Simple Arduino controlled robot to be controlled via an analogue joystick.
 *  
 * Sadly, the code needs to be refactored as it isn't very easy to understand in its current state.
 */

// definitions for motor control
#define LEFT_MOTOR  0
#define RIGHT_MOTOR 1

#define STOP      0
#define FORWARDS  1
#define BACKWARDS 2

// Left Motor (A)
int dir1PinA = 2;
int dir2PinA = 3;
int speedPinA = 9;  // Needs to be a PWM pin to be able to control motor speed

// Right Motor (B)
int dir1PinB = 4;
int dir2PinB = 5;
int speedPinB = 10; // Needs to be a PWM pin to be able to control motor speed

// Joystick connection pin definitions
int joystickXPin = A0;
int joystickYPin = A1;
int joystickButton = A2;

void setup() {  // Setup runs once per reset

  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // prints title with ending line break
  Serial.println("Motor control demo using a joystick");

  //Define L298N Dual H-Bridge Motor Controller Pins
  pinMode(dir1PinA,OUTPUT);
  pinMode(dir2PinA,OUTPUT);
  pinMode(speedPinA,OUTPUT);
  pinMode(dir1PinB,OUTPUT);
  pinMode(dir2PinB,OUTPUT);
  pinMode(speedPinB,OUTPUT);

  // define Joystick connections
  pinMode(joystickXPin, INPUT);
  pinMode(joystickYPin, INPUT);   

}

void loop() {
  int leftMotSpeed = 0;
  int rightMotSpeed = 0;  
  
  int motSpeed = analogRead(joystickXPin);       // read the position of the joystick X axis
  motSpeed = motSpeed - 512;                     // the joystick rests at the half way point which is equal to 512.
                                                 // by subtracting 512, we get a positive number when the stick moves forwards, and
                                                 // a negative number when the stick moves backwards.

  int motDirection = analogRead(joystickYPin);       // read the position of the joystick Y axis
  motDirection = motDirection - 512;                 // subtract the centre location to provide a value where positive numbers are one direction, negative numbers are the other direction...

  // calculate how much each motor should be slowed by to make the robot turn based on the joystick position
  float leftMotorBrake = 0;
  float rightMotorBrake = 0;  
  if (motDirection < 0) {
    rightMotorBrake = 1.0+(float)(motDirection)/512;    
    leftMotorBrake = 1.0;
  } else {
    // turning left, so slow down the left motor
    leftMotorBrake = 1.0-(float)(motDirection)/512;
    rightMotorBrake = 1.0;    
  }
  //leftMotorBrake = (leftMotorBrake + 1)/2;      // mean average the brake value with 1.0 to prevent it ever reaching 0
  //rightMotorBrake = (rightMotorBrake + 1)/2;    // these lines make the steering less twitchy...

  
  if (motSpeed > 0) {
    motSpeed = (motSpeed + abs(motDirection));    // We want the speed of the motor to relate to how far the joystick is pressed, regardless of whether it is pushed forward or right
    if (motSpeed > 512) motSpeed = 512;           // However, the maximum speed must be capped at a known value (512)
    
    // if the speed is a positive number, make the motors run forwards
    leftMotSpeed = (motSpeed * rightMotorBrake)/4;    // apply the brake and rescale - analogue inputs are 0-1023, motor control values are 0-255 
    rightMotSpeed = (motSpeed * leftMotorBrake)/4;
    motorControl(LEFT_MOTOR,  FORWARDS, leftMotSpeed);
    motorControl(RIGHT_MOTOR, FORWARDS, rightMotSpeed);    
  } else {
    // the speed is a negative number, so make the motors run backwards
    motSpeed = -motSpeed;

    motSpeed = (motSpeed + abs(motDirection));    // We want the speed of the motor to relate to how far the joystick is pressed, regardless of whether it is pushed forward or right
    if (motSpeed > 512) motSpeed = 512;           // However, the maximum speed must be capped at a known value (512)
        
    leftMotSpeed = (motSpeed * rightMotorBrake)/4;  // apply the brake and rescale - analogue inputs are 0-1023, motor control values are 0-255
    rightMotSpeed = (motSpeed * leftMotorBrake)/4;    
    motorControl(LEFT_MOTOR,  BACKWARDS, leftMotSpeed);
    motorControl(RIGHT_MOTOR, BACKWARDS, rightMotSpeed);
  }
  
  Serial.print(leftMotorBrake);
  Serial.print(" ");
  Serial.print(rightMotorBrake);  
  Serial.print(" ");  
  Serial.print(leftMotSpeed);
  Serial.print(" ");
  Serial.println(rightMotSpeed);  
  
  delay(100); // wait 0.1 seconds, then run through this again.
  
}

void motorControl(int motor, int motorDirection, int motorSpeed) {
  // this function commands one of the motors to spin in a given direction and tells it how fast to go
  // motor speed values are 0-255
  
  int dir1Pin = 0;  // variable to hold which pin on the Arduino controls the forwards direction for this motor
  int dir2Pin = 0;  // variable to hold which pin on the Arduino controls the backwards direction for this motor
  int speedPin = 0; // variable to hold which pin on the Arduino controls the motor speed for this motor
  
  // first check which motor we will be controlling..
  
  if (motor == LEFT_MOTOR) {
    // set everything up to control the left motor
    dir1Pin = dir1PinA;
    dir2Pin = dir2PinA;    
    speedPin = speedPinA;    
  } 
  else if (motor == RIGHT_MOTOR)
  {
    // set everything up to control the right motor  
    dir1Pin = dir1PinB;
    dir2Pin = dir2PinB;    
    speedPin = speedPinB;        
  } 
  else {
    // looks like we don't recognise that motor
    return;
  }

  if (motorSpeed < 15) {
    // the motor speed is so slow, the motor probably won't be able to turn, so just set the speed to 0.
    motorSpeed = 0;
  }  

  switch (motorDirection) {
  case FORWARDS:
      //motor forward
      digitalWrite(dir1Pin, HIGH);
      digitalWrite(dir2Pin, LOW);
      analogWrite(speedPin, motorSpeed);      
      break;
      
  case BACKWARDS:
      //motor forward
      digitalWrite(dir1Pin, LOW);
      digitalWrite(dir2Pin, HIGH);
      analogWrite(speedPin, motorSpeed);            
      break;      
      
  case STOP:
      analogWrite(speedPin, 0);            // stop the motor!
      break;        

  default:      
      // we didn't recognise the motor direction given, stop the motor anyway
      analogWrite(speedPin, 0);            // stop the motor!  
  }  
}


