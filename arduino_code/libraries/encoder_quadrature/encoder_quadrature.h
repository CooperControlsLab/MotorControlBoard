#ifndef ARDUINO_H
#define ARDUINO_H
#include <Arduino.h>
#endif

#ifndef CONTROL_BOARD_CONFIG_H
#include <control_board_config.h>
#define CONTROL_BOARD_CONFIG_H
#endif

class Encoder{

public:
    Encoder();
    Encoder(int);
    int ppr;
    static volatile long enc_count;

    static void pulseA();
    static void pulseB();

    void setup_interrupts();
};

extern void pa(); //Let's ISR call the Encoder::pulseA function
extern void pb(); //Let's ISR call the Encoder::pulseB function