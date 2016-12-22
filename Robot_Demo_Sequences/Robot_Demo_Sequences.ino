/* This code drives the Simple Arduino controlled robot through a sequence of moves causing it to dance around
 *  
 *  The robot is controlled with the "motorControl()" function which accepts a motor, direction and speed as inputs    
 *  For example,  motorControl(RIGHT_MOTOR, FORWARDS, 255);  will make the right motor spin forwards at full speed  
 *  
 *  Speed values are between 0 (stopped) and 255 (full speed).
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

void setup() {  // Setup runs once per reset

    //Define L298N Dual H-Bridge Motor Controller Pins
    pinMode(dir1PinA,OUTPUT);
    pinMode(dir2PinA,OUTPUT);
    pinMode(speedPinA,OUTPUT);
    pinMode(dir1PinB,OUTPUT);
    pinMode(dir2PinB,OUTPUT);
    pinMode(speedPinB,OUTPUT);

    // stop the motor.
    motorControl(RIGHT_MOTOR, STOP, 0);
    motorControl(LEFT_MOTOR,  STOP,  0);   
}

void loop() {

    // spin around clockwise
    motorControl(LEFT_MOTOR,  FORWARDS,  255);    
    motorControl(RIGHT_MOTOR, BACKWARDS, 255);        
    delay(1200);

    // brief pause
    motorControl(LEFT_MOTOR,  STOP, 0);    
    motorControl(RIGHT_MOTOR, STOP, 0);        
    delay(100);    

    // drive forward
    motorControl(LEFT_MOTOR,  FORWARDS,  255);    
    motorControl(RIGHT_MOTOR, FORWARDS, 255);        
    delay(1000);

    // brief pause
    motorControl(LEFT_MOTOR,  STOP, 0);    
    motorControl(RIGHT_MOTOR, STOP, 0);        
    delay(100);        

    // spin around anticlockwise
    motorControl(LEFT_MOTOR,  BACKWARDS,  255);    
    motorControl(RIGHT_MOTOR, FORWARDS, 255);        
    delay(1200);

    // brief pause
    motorControl(LEFT_MOTOR,  STOP, 0);    
    motorControl(RIGHT_MOTOR, STOP, 0);        
    delay(100);    

    // drive backwards
    motorControl(LEFT_MOTOR,  BACKWARDS,  255);    
    motorControl(RIGHT_MOTOR, BACKWARDS, 255);        
    delay(1000);    

    // brief pause
    motorControl(LEFT_MOTOR,  STOP, 0);    
    motorControl(RIGHT_MOTOR, STOP, 0);        
    delay(100);          
    
}

void motorControl(int motor, int motorDirection, int motorSpeed) {
  // this function commands one of the motors to spin in a given direction and tells it how fast to go

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


