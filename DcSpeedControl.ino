#include "MotorEncoderOnePin.h"


const uint8_t pinENA = 9;
const uint8_t pinIN1 = 10;
const uint8_t pinIN2 = 11;
const uint8_t encoderPin = 3;

/*
const uint8_t pinENA = 6;
const uint8_t pinIN1 = 7;
const uint8_t pinIN2 = 8;
const uint8_t encoderPin = 2;
*/

struct TConfigMotor motorAmarillo;
struct TConfigMotor motorEncoderReductora;

MotorEncoderOnePin motor;

void setup() {
  //Configurscion PID del motor Amarillo
  motorAmarillo.EsperaMaxMotorParado_Micros = 40000;
  motorAmarillo.DescartarPulse_Micros = 3500;
  motorAmarillo.PulseMaxVelocidad_Micros = 10000;
  motorAmarillo.PulseMinVelocidad_Micros = 25000;
  motorAmarillo.PulsosPorVuelta = 200;

  //Configurscion PID del motor con Encoder Metalico
  motorEncoderReductora.EsperaMaxMotorParado_Micros = 15000;
  motorEncoderReductora.DescartarPulse_Micros = 1;
  motorEncoderReductora.PulseMaxVelocidad_Micros = 1000;
  motorEncoderReductora.PulseMinVelocidad_Micros = 10000;
  motorEncoderReductora.PulsosPorVuelta = 900;

  motor.attachPin(pinENA, pinIN1, pinIN2);
  motor.attachInt(encoderPin, func);
  motor.setLimitsPwm(0, 255);
  motor.setConfigMotor(motorEncoderReductora);
  motor.setPos(0);
  pinMode(13, OUTPUT);
  Serial.begin(115200);
  //Serial.end();
}

void loop() {

  digitalWrite(13, HIGH);
  motor.runTo(2000, 10000);
  while (motor.run())
    ;
  Serial.print("Pos Final: ");
  Serial.println(motor.getPos());
  Serial.flush();

  motor.stop();
  delay(4000);

  digitalWrite(13, LOW);
  motor.runTo(-10, 1000, false);
  while (motor.run())
    ;
  Serial.print("Pos Final: ");
  Serial.println(motor.getPos());
  motor.stop();
  delay(4000);
}

void func(void) {
  digitalWrite(13, HIGH - digitalRead(13));
  motor.pulso();
}
