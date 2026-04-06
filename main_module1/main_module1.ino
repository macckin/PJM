#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "MOTOR_CONTROL_DEG.h"

RF24 radio(49, 48);

const byte address[6] = "00001";

struct CtrlData{
  int vel;
  double setEP1;
  int oc;
};

CtrlData receiveddata;

void setup(){
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address); // 송신기 주소
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  setupMotor();

  preT = millis();
  receiveddata.vel = 0;
  receiveddata.setEP1 = 0;
}

void loop(){
  if(radio.available()){
    radio.read(&receiveddata, sizeof(receiveddata)); 
    setEP = inverse(Deg8_1(receiveddata.setEP1))*Afterrate;
    output = PID(REP);
    doMotor(output);
    Imotor(receiveddata.vel, receiveddata.oc);
    delay(5);
  }
}