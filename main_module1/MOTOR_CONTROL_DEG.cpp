#include "MOTOR_CONTROL_DEG.h"


//double kp = 2.0;
//double kd = 1.8;
double elaT = 0, error = 0, lasterror = 0, input = 0, output = 0, Derror = 0;
const int EA = 2;  
const int EB = 3;
volatile double REP = 0;  
volatile int setEP = 0;   
double curvel = 0;  
volatile bool K = 0; 
unsigned long curT, preT;

void setupMotor(){
  pinMode(EA, INPUT_PULLUP);
  pinMode(EB, INPUT_PULLUP);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(RW, OUTPUT);
  pinMode(LW, OUTPUT);
  pinMode(LPWM, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(EA), IEA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EB), IEB, CHANGE);
}

void IEA() {
  if (digitalRead(EA) == digitalRead(EB)) {
    REP += 1;
  } else {
    REP -= 1;
  }
}

void IEB() {
  if (digitalRead(EA) == digitalRead(EB)) {
    REP -= 1;
  } else {
    REP += 1;
  }
}

void doMotor(double inp) {
  inp = inp*Inprate;
  if(REP > (Inversezero*0.5) && setEP == 0){
    setEP = Inversezero;
    Serial.print("역방향 영점 : ");
  }
  if (abs(REP - setEP) < Margin) {  // 목표 위치 근처에 도달하면 모터 정지
    stopMotor(); // 모터 완전 정지
    Serial.println("목표 위치 근처에서 모터 정지");
  } else if (setEP > REP) { //CW
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, abs(inp));
  } else if (setEP < REP){  // CCW
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(inp));
    Serial.println("모터 뒤로 회전");
  }
}

void stopMotor() {
  Serial.println("역상제동 적용");

  if (digitalRead(IN1) == HIGH && digitalRead(IN2) == LOW) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else if (digitalRead(IN1) == LOW && digitalRead(IN2) == HIGH) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, 50); 
  delay(10); 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  if(setEP == Inversezero){
    setEP = 0;
    REP -= Inversezero;
  }
  Serial.println("모터 완전 정지");
}

double PID(double input) {
  if(REP > (Inversezero*0.5) && setEP == 0){
    setEP = Inversezero;
  }
  curT = millis();
  elaT = (double)(curT - preT) / 1000.0; // 밀리초를 초로 변환

  error = setEP - input;  // 에러 계산
  Derror = (error - lasterror) / elaT;  // D 계산

  double output = kp * error + kd * Derror; // P와 D 제어 결합

  lasterror = error;
  preT = curT;

  return output;
}

int Deg8_1(int setEP){
  switch(setEP){
  case 0:
    setEP = 0;
  break;
  case 110:
    setEP = 0;
  break;
  case 220:
    setEP = 770;
  break;
  case 330:
    setEP = 550;
  break;
  case 440:
    setEP = 330;
  break;
  case 550:
    setEP = 220;
  break;
  case 660:
    setEP = 220;
  break;
  case 770:
    setEP = 110;
  break;
  case 880:
    setEP = 0;
  break;
  default:
    setEP = 0;
  break;
  }
  return setEP;
}

int inverse(int setEP){
  if(setEP == 770){
    setEP = -110;
  }else if(setEP == 660){
    setEP = -220;
  }else if(setEP == 550){
    setEP = -330;
  }
  return setEP;
}

void Imotor(int vel,int oc){
  // vel 로 속도 50? 정도로 만들고 oc 1 속도 줌 0 그대로 -1속도 늠
  switch(vel){
    case 1:
      K = 1;
    break;
    case 0:
      if(K == 1){
        curvel += 10;
        if(curvel >= Maxspeed) curvel = Maxspeed;
      }else{
        curvel = curvel;
      }
    break;
    case -1:
      K = 0;
      curvel = Minspeed;
    break;
  }
  oc = 0;
  /*
  switch(oc){
    case 1:
      curvel -= 1;
      if(curvel <= Minspeed)curvel = Minspeed;
    break;
    case 0:
      curvel = curvel;
    break;
    case -1:
      curvel += 1;
      if(curvel >= Maxspeed)curvel = Maxspeed;
    break;

  }
*/
  digitalWrite(RW, HIGH);
  digitalWrite(LW, HIGH);
  analogWrite(LPWM, curvel);

  Serial.println(curvel);
}
