#ifndef _MotorEncoderOnePin_
#define _MotorEncoderOnePin_

#include <Arduino.h>

/* Portotipo de la funcion para el "Attach" */
typedef void (&InterrupcionFunction)();

enum TEstadoMotor { estadoStop, estadoForward, estadoBackward };

class MotorEncoderOnePin {

public:
  MotorEncoderOnePin();
  void attachPin(uint8_t, uint8_t, uint8_t);
  void attachInt(uint8_t, InterrupcionFunction);
  long getPos() { return posionEnc; };
  void setPos(long p) { posionEnc = p; };
  void pulso();
  void forwardPwd(int);
  void backwardPwd(int);
  void runSpeed();
  void stop();
  void runTo(long, long, bool = false);
  bool runTo(long);
  bool run();
  void setLimitsPwm(int, int);
  void setRpV(int rpv) { this->rpv = abs(rpv); };

private:
  uint8_t pinENA;
  uint8_t pinIN1;
  uint8_t pinIN2;
  uint8_t encoderPin;
  long posionEnc;
  int pwm;
  long speed;
  long target;
  unsigned long delayPulse, timePass, timeNow;
  bool newPulse;
  inline void chekPwmLimits();
  int minPwm, maxPwm;
  int rpv;
  TEstadoMotor estadoMotor;

};

#endif