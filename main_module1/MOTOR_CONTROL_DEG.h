#ifndef MOTOR_CONTROL_DEG_H
#define MOTOR_CONTROL_DEG_H

#include <Arduino.h>

#define ENA 10
#define IN1 8
#define IN2 9
#define RW 12
#define LPWM 11
#define LW 13
#define Inversezero ((int)(880*Afterrate)) // 880*4.3
#define Afterrate 11.75
#define Inprate 0.7
#define Margin 207
#define Maxspeed 80
#define Minspeed 0
#define kp 2.0
#define kd 1.8

extern double elaT, error, lasterror, input, output, Derror;
extern const int EA, EB;
extern volatile double REP;
extern volatile int setEP;
extern double curvel;
extern volatile bool K;
extern unsigned long curT, preT;

void setupMotor();
void IEA();
void IEB();
void stopMotor();
void doMotor(double inp);
double PID(double input);
int Deg8_1(int setEP);
int inverse(int setEP);
void Imotor(int vel,int oc);

#endif