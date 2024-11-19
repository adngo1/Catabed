#include <Servo.h>

// Servo
Servo mainServo;

// Pins
const int trigPin = 9;
const int echoPin = 10;
const int buttonPin = 7;
const int servoPin = 6;

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
      delay(10);
    }
  } else if(currentAngle > inputAngle) {
    for(int i = 1; i <= (currentAngle - inputAngle); i++) {
      (*servo).write(currentAngle - i);
      if(checkObstruction(trigPin, echoPin, .10) == 0 && obstructionCheck) break;
      delay(10);
    }
  }
  
  return;
}

// Main
void setup() {
  pinMode(trigPin, OUTPUT);  
  pinMode(echoPin, INPUT);
  pinMode(buttonPin, INPUT);
  mainServo.attach(servoPin);
  Serial.begin(9600);    
}

void loop() {
  if(digitalRead(buttonPin) == 1) {
    if(mainServo.read() != 90) {
      slowlyRotate(90, buttonPin, false, &mainServo);
      delay(500);
      Serial.println(mainServo.read());
    } else if(checkObstruction(trigPin, echoPin, .10)) {
      if(mainServo.read() == 90) {
        slowlyRotate(45, buttonPin, true, &mainServo);
        delay(500);
        Serial.println(mainServo.read());
      }
    }
  }
  
  delay(10);
}
