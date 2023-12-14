#include "Arduino.h"
#include "MotorEncoderOnePin.h"

MotorEncoderOnePin::MotorEncoderOnePin() {
  posionEnc = 0;
  pwm = 0;
  speed = 0;
  target = 0;
  newPulse = false;
  minPwm = 0;
  maxPwm = 255;
  estadoMotor = estadoStop;
  rpv = 900;
}

void MotorEncoderOnePin::attachPin(uint8_t pinENA, uint8_t pinIN1, uint8_t pinIN2) {
  this->pinENA = pinENA;
  this->pinIN1 = pinIN1;
  this->pinIN2 = pinIN2;
  pinMode(pinENA, OUTPUT);
  analogWrite(pinIN2, 0);
  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2, LOW);
}

/* La funcion a para el encoder: Attach tiene que tener esta estructura:
    void func(void) {
      motor.pulso();
    }
  Dentro de la funcion hay que llamar el método "pulso" para que el encoder funcione.
  Solo se le pasa un pin, no detecta el sentido de giro. Esta informacion la saca de las llamadas
  a las funciones forward, backward y stop. */
void MotorEncoderOnePin::attachInt(uint8_t encoderPin, InterrupcionFunction f) {
  this->encoderPin = encoderPin;
  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), f, RISING);
}

void MotorEncoderOnePin::forwardPwd(int pwm) {
  this->pwm = pwm;
  analogWrite(pinENA, pwm);
  digitalWrite(pinIN1, HIGH);
  digitalWrite(pinIN2, LOW);
  estadoMotor = estadoForward;
}

void MotorEncoderOnePin::backwardPwd(int pwm) {
  this->pwm = pwm;
  analogWrite(pinENA, pwm);
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2, HIGH);
  estadoMotor = estadoBackward;
}

void MotorEncoderOnePin::stop() {
  pwm = 0;
  analogWrite(pinENA, pwm);
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2, LOW);
  estadoMotor = estadoStop;
}

/* Establece el valor de destino en numero de pulsos, con uina velocidad "vel" determinada.
    ojo: "vel" ahora mismo el el tiempo entre pulsos. Cuanto mas pequeño es te tiempo, mayor velocidad.
    Por ejemplo, para el motor del C222 es entre 3000 (fast) y 8000 (slow).
    Si quieres que gire infinito, entonces forever=true y Target >= 0 gura forware, target < 0 gira backword*/
void MotorEncoderOnePin::runTo(long target, long vel, bool forever) {
  if (forever) {
    if (target >= 0) target = 2147483647;
    else target = -2147483647;
  } 
  this->target = target;
  speed = vel;
  newPulse = false;
  pwm = map(vel, configMotor.PulseMaxVelocidad_Micros, configMotor.PulseMinVelocidad_Micros, 255, 100);
  chekPwmLimits();
  if (target > posionEnc)
    forwardPwd(pwm);
  else
    backwardPwd(pwm);
}

/* Aunque se parece a la anterior, es muy diferente.
    Esta solo modifica el Target con incerementos (positivos y negativos).
    Debuelve verdadero si el motor esta en movimieto, false, si el motor estaba parado, y
    por tanto, no se ha inicializado de nuevo.
    Es importante saber, que no inicia el movimiento, solo modifica el target.
    Por supuesto, si esta en "forever" no modifica el target.
*/
bool MotorEncoderOnePin::newRunTo(long incTarget){
  if (target != 2147483647 && target != -2147483647)
    target += incTarget;
  return (estadoMotor != estadoStop);
}

/* Ajusta la velocidad a la "vel" establecida en runTo. Si es lenta, acelera y viceversa. Si no hay pulsos nuevos
   acelera poco a poco. Vericica siempre que no me salgo de rango de pwm. */
bool MotorEncoderOnePin::run() {
  if (estadoMotor == estadoStop) return false;
  if (estadoMotor == estadoForward) {
    if (target <= posionEnc) {
      stop();
      return false;
    }
  }
  if (estadoMotor == estadoBackward) {
    if (target >= posionEnc) {
      stop();
      return false;
    }
  }
  if (newPulse) {
    if (delayPulse > speed)
      pwm++;
    else if (delayPulse < speed)
      pwm--;
    newPulse = false;
  } else {
    if (micros() - timePass > configMotor.EsperaMaxMotorParado_Micros) pwm++;
  }
  chekPwmLimits();
  analogWrite(pinENA, pwm);
  return true;
}

void MotorEncoderOnePin::setLimitsPwm(int min, int max) {
  if (min >= max) return;
  maxPwm = max;
  minPwm = min;
}

void MotorEncoderOnePin::pulso() {
  static long incremento = 0;
  timeNow = micros();
  delayPulse = timeNow - timePass;
  if(delayPulse < configMotor.DescartarPulse_Micros) return;
  newPulse = true;
  timePass = timeNow;
  if (estadoMotor == estadoForward) incremento = +1;
  if (estadoMotor == estadoBackward) incremento = -1;
  posionEnc += incremento;
}

void MotorEncoderOnePin::chekPwmLimits() {
  if (pwm < minPwm) pwm = minPwm;
  if (pwm > maxPwm) pwm = maxPwm;
}