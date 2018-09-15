#ifndef FSI6_h
#define FSI6_h

#include "Arduino.h"

#define CH1_PIN 2
#define CH2_PIN 3
#define CH3_PIN 4
#define CH4_PIN 5

struct ChannelRaw{
  volatile int ch1;
  volatile int ch2;
  volatile int ch3;
  volatile int ch4;
};

class FSI6 {
  public:
    volatile ChannelRaw channelRaw;
    volatile ChannelRaw prevChannelRaw;
    volatile double prevCh1; 
    volatile double prevCh2; 
    volatile double prevCh3;
    volatile double prevCh4;
  
    FSI6();
    void init();
    ChannelRaw readChannelRaw();
    void printChannels(ChannelRaw channelRaw);
    void readChannel3Falling();
};

#endif
