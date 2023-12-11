#include "MotorEncoderOnePin.h"

const uint8_t pinENA = 6;
const uint8_t pinIN1 = 7;
const uint8_t pinIN2 = 8;
const uint8_t encoderPinA = 2;
const uint8_t encoderPinB = 8;

MotorEncoderOnePin motor;

void setup() {
  motor.attachPin(pinENA, pinIN1, pinIN2);
  motor.attachInt(2, func);
  motor.setLimitsPwm(0, 255);
  pinMode(13, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  digitalWrite(13, HIGH);
  motor.runTo(2000, 3000);
  while (motor.run());
  Serial.print("pos: ");
  Serial.println(motor.getPos());
  motor.stop();
  delay(2000);

  digitalWrite(13, LOW);
  motor.runTo(-10, 7000, true);
  while (motor.run())
    Serial.println(motor.getPos());
  motor.stop();
  delay(2000);
}

void func(void) {
  digitalWrite(13, HIGH - digitalRead(13));
  motor.pulso();
}
