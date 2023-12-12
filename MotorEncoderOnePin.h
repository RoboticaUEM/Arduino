#ifndef _MotorEncoderOnePin_
#define _MotorEncoderOnePin_

#include <Arduino.h>

struct TConfigMotor {
  unsigned int EsperaMaxMotorParado_Micros;
  unsigned int DescartarPulse_Micros;
  unsigned int PulseMaxVelocidad_Micros;
  unsigned int PulseMinVelocidad_Micros;
  unsigned int PulsosPorVuelta;
};

/* Portotipo de la funcion para el "Attach" */
typedef void (&InterrupcionFunction)();

enum TEstadoMotor { estadoStop,
                    estadoForward,
                    estadoBackward };

class MotorEncoderOnePin {

public:
  MotorEncoderOnePin();
  void attachPin(uint8_t pinENA, uint8_t pinIN1, uint8_t pinIN2);
  void attachInt(uint8_t encoderPin, InterrupcionFunction f);
  long getPos() {
    return posionEnc;
  };
  void setPos(long p) {
    posionEnc = p;
  };
  void pulso();
  void runSpeed();
  void stop();
  void runTo(long target, long vel, bool forever = false);
  bool newRunTo(long incTarget) ;
  bool run();
  void setLimitsPwm(int min, int max);
  void setRpV(int rpv) {
    this->rpv = abs(rpv);
  };
  void setConfigMotor(TConfigMotor cfgM) {
    configMotor = cfgM;
  };

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
  void forwardPwd(int pwm);
  void backwardPwd(int pwm);
  int minPwm, maxPwm;
  int rpv;
  TEstadoMotor estadoMotor;
  TConfigMotor configMotor;
};

#endif