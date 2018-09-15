#include "FSI6.h"
#include "Arduino.h"

FSI6::FSI6() {

}

void FSI6::init() {
  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH3_PIN, INPUT);
  pinMode(CH4_PIN, INPUT);
}

ChannelRaw FSI6::readChannelRaw() {

  ChannelRaw channelRaw;

  //channelRaw.ch1 = pulseIn(CH1_PIN, HIGH);
  //channelRaw.ch2 = pulseIn(CH2_PIN, HIGH);
  //channelRaw.ch3 = pulseIn(CH3_PIN, HIGH);
  //channelRaw.ch4 = pulseIn(CH4_PIN, HIGH);

  return channelRaw;
}


void FSI6::printChannels(ChannelRaw channelRaw) {

  Serial.print("Channel ch1: ");
  Serial.print(channelRaw.ch1);
  Serial.print(" | ch2: ");
  Serial.print(channelRaw.ch2);
  Serial.print(" | ch3: ");
  Serial.print(channelRaw.ch3);
  Serial.print(" | ch4: ");
  Serial.println(channelRaw.ch4);



  Serial.print("With interrupt \t");
  Serial.println(this->channelRaw.ch3);
}

