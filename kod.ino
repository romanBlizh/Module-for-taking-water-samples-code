#define SERVO 9
#define PUMP  5
#define SENSOR 4

#include <Servo.h>

Servo myserv;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(PUMP, OUTPUT);
  pinMode(SENSOR, INPUT_PULLUP);
  myserv.attach(SERVO);
  myserv.write(0);

}

void loop() {
  // put your main code here, to run repeatedly:
  bool data = digitalRead(SENSOR);
  if (data) {
    digitalWrite(PUMP, LOW);
    myserv.write(0);
  } else {
    myserv.write(90);
    digitalWrite(PUMP, HIGH);
  }
}
