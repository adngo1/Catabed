#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Servo.h>

// Servo
Servo mainServo;

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Pins
const int trigPin = 9;
const int echoPin = 10;
const int buttonPin = 7;
const int servoPin = 6;
const int potPin = A0;

// Functions
double getDistance(int trigPin, int echoPin) {
  /* Get distance from ultrasonic sensor in meters
    * @param trigPin: Trig pin number on ultrasonic sensor
    * @param echoPin: Echo pin number on ultrasonic sensor
    * @return: Meters detected from ultrasonic sensor
  */

  // Setup variables
  double duration;
  
  // Startup trig pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Find duration from echo pin
  duration = pulseIn(echoPin, HIGH);

  // Do calculation to find distance in meters
  return (duration * 0.000343) / 2.0;
}

bool checkObstruction(int trigPin, int echoPin, double distance) {
  /* Check if there is anything blocking the wheelchair
    * @param trigPin: Trig pin number on ultrasonic sensor
    * @param echoPin: Echo pin number on ultrasonic sensor
    * @param distance: Minimum distance in meters that the wheelchair has to be in range for
    * @return: True if there is an obstruction, false otherwise
  */
 
  return getDistance(trigPin, echoPin) <= distance;
}

void slowlyRotate(int inputAngle, Servo *servo, bool obstructionCheck = false) {
  /* Slowly rotate the motor, checking if the obstruction is still there if asked
    * @param inputAngle: Angle to move to
    * @param servo: Pointer to servo module
    * @param obstructionCheck: Optional check to see if there is still an obstruction
  */

  // Get current angle from motor
  int currentAngle = (*servo).read();
  
  // If the current angle is less than the input angle
  // Move the servo up one degree at a time and check if the obstruction persists if
  // the option is selected, if it is, stop moving the servo
  if(currentAngle < inputAngle) {
    for(int i = 1; i <= (inputAngle - currentAngle); i++) {
      (*servo).write(currentAngle + i);
      if(checkObstruction(trigPin, echoPin, .10) == 0 && obstructionCheck) break;
      delay(30);
    }

  // If the current angle is greater than the input angle
  // Move the servo down one degree at a time and check if the obstruction persists if
  // the option is selected, if it is, stop moving the servo
  } else if(currentAngle > inputAngle) {
    for(int i = 1; i <= (currentAngle - inputAngle); i++) {
      (*servo).write(currentAngle - i);
      if(checkObstruction(trigPin, echoPin, .10) == 0 && obstructionCheck) break;
      delay(30);
    }
  }
  
  return;
}

void updateLCD(int setAngle, LiquidCrystal_I2C *lcd, Servo *servo) {
  /* Update LCD module
    * @param setAngle: Number to set to set angle section
    * @param lcd: Pointer to lcd module
    * @param servo: Pointer to servo module
  */

  // Clear current LCD
  (*lcd).clear();

  // Set current angle number
  (*lcd).setCursor(0, 0);
  (*lcd).print("Current: ");
  (*lcd).print((*servo).read());

  // Set set angle number
  (*lcd).setCursor(0, 1);
  (*lcd).print("Set: ");
  (*lcd).print(setAngle);

  return;
}

// Main
void setup() {
  // Pin Setup
  pinMode(trigPin, OUTPUT);  
  pinMode(echoPin, INPUT);
  pinMode(buttonPin, INPUT);

  // Attach motor
  mainServo.attach(servoPin);

  // Turn on LCD
  lcd.init();       
  lcd.backlight();  
  updateLCD(map(analogRead(potPin), 0, 1023, 45, 90), &lcd, &mainServo);

  // Turn on serial
  Serial.begin(9600);    
}

void loop() {
  // Setup static variables to not have to update LCD frequently
  static int prevSetAngle = -1;
  static int setAngle = map(analogRead(potPin), 0, 1023, 45, 90);
  
  // Find set angle from potentiometer
  setAngle = map(analogRead(potPin), 0, 1023, 45, 90);

  if(digitalRead(buttonPin) == 1) {

    // If button is pressed and the angle is not 90 degrees
    // First check the set angle is greater than the current angle of the motor
    // and move to the set angle if it is
    // If the set angle is lower than the current angle
    // Check for an obstruction, and if there is an obstruction
    // move the motor to the set angle
    // Otherwise, move to 90 degrees
    if(mainServo.read() != 90) {
      if(setAngle > mainServo.read()) {
        slowlyRotate(setAngle, &mainServo); 
      } else if(setAngle < mainServo.read() && checkObstruction(trigPin, echoPin, .10)){
        slowlyRotate(setAngle, &mainServo); 
      } else {
        slowlyRotate(90, &mainServo); 
      }
      delay(500);
    } 

    // If button is pressed and the angle is 90 degrees
    // Check if the wheelchair is close enough to an object
    // If it is, then rotate the wheelchair to the set angle
    else {
      if(checkObstruction(trigPin, echoPin, .10)) {
        slowlyRotate(setAngle, &mainServo, true);
        delay(500);        
      }
    }

    // Update LCD after changing motor value
    updateLCD(setAngle, &lcd, &mainServo);
  }
  
  // Only LCD if the potentiometer value changed
  if(prevSetAngle != setAngle) updateLCD(setAngle, &lcd, &mainServo);
  prevSetAngle = setAngle;

  delay(10);
}
