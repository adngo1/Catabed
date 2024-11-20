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
  double duration;
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  return (duration * 0.000343) / 2.0;
}

bool checkObstruction(int trigPin, int echoPin, double distance) {
  return getDistance(trigPin, echoPin) <= distance;
}

void slowlyRotate(int inputAngle, int buttonPin, bool obstructionCheck, Servo *servo) {
  int currentAngle = (*servo).read();
  
  if(currentAngle < inputAngle) {
    for(int i = 1; i <= (inputAngle - currentAngle); i++) {
      (*servo).write(currentAngle + i);
      if(checkObstruction(trigPin, echoPin, .10) == 0 && obstructionCheck) break;
      delay(30);
    }
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
  static int prevSetAngle = -1;
  static int setAngle = map(analogRead(potPin), 0, 1023, 45, 90);
  
  setAngle = map(analogRead(potPin), 0, 1023, 45, 90);
  
  if(digitalRead(buttonPin) == 1) {
    if(mainServo.read() != 90) {
      if(setAngle != mainServo.read()) {
        slowlyRotate(setAngle, buttonPin, false, &mainServo); 
      } else {
        slowlyRotate(90, buttonPin, false, &mainServo); 
      }
      delay(500);
    } else if(checkObstruction(trigPin, echoPin, .10)) {
      if(mainServo.read() == 90) {
        slowlyRotate(setAngle, buttonPin, true, &mainServo);
        delay(500);
      }
    }
    updateLCD(setAngle, &lcd, &mainServo);
  }
  
  if(prevSetAngle != setAngle) updateLCD(setAngle, &lcd, &mainServo);
  prevSetAngle = setAngle;
  delay(10);
}
