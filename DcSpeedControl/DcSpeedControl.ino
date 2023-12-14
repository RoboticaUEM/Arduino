#include "MotorEncoderOnePin.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

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

float mapLong(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

struct TConfigMotor motorAmarillo;
struct TConfigMotor motorEncoderReductora;

String logOut;

MotorEncoderOnePin motor;

// Keep track of the number of wheel ticks
//std_msgs::Int16 right_wheel_tick_count;
//ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);

std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

std_msgs::String sLog;
ros::Publisher logMotor("logMotor", &sLog);

void getCmdVel(const geometry_msgs::Twist& cmdVel) {
  float x = cmdVel.linear.x;
  if (x == 0) {
    motor.stop();
    return;
  }
  float v = mapLong(abs(x), 0.0, 2.0, 10000, 1000);
  if (motor.getEstado() != estadoStop)
    motor.setSpeed((long)v);
  else
    motor.runTo(long(x*100), long(v), true);
}

// Set up ROS subscriber to the velocity command
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &getCmdVel);

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

  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  //nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(subCmdVel);
  nh.advertise(logMotor);
}

long tiempo = 0;
void loop() {
  motor.run();
  if (millis() - tiempo > 250) {
    logOut = String("pwm:") + motor.getPwm();
    sLog.data = logOut.c_str();
    logMotor.publish(&sLog);
    // Publish tick counts to topics
    left_wheel_tick_count.data = motor.getPos();
    leftPub.publish(&left_wheel_tick_count);
    tiempo = millis();
  }
  nh.spinOnce();
  delay(1);
}

void func(void) {
  motor.pulso();
}
