#ifndef ENCODER_QUADRATURE_H
#define ENCODER_QUADRATURE_H
#include <encoder_quadrature.h>
#endif
#ifndef ARDUINO_H
#include <Arduino.h>
#define ARDUINO_H
#endif

// #ifndef PINCHANGEINTERRUPT_H
// #define PINCHANGEINTERRUPT_H
// #include <PinChangeInterrupt.h>
// #endif
// #include "WInterrupts.c"

//Initialize static class members so they are defined for the ISR
static volatile long Encoder::enc_count=0;

Encoder::Encoder(){};
Encoder::Encoder(int p)
{
  ppr = p;
}


void Encoder::setup_interrupts(){
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A), pa, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), pb, CHANGE);
    Serial.print("init finished");
}

void pa(){
    Encoder::pulseA();
}

void pb(){
    Encoder::pulseB();
}


//Encoder interrupts
void Encoder::pulseA(){
  int valA = digitalRead(ENC_A);
  int valB = digitalRead(ENC_B);

  if (valA == HIGH) { // A Rise
    if (valB == LOW) {
      enc_count++;  // CW
    }
    else {
      enc_count--;  // CCW
    }
  }
  else { // A fall
    if (valB == HIGH) {
      enc_count ++;  // CW
    }
    else {
      enc_count --;  //CCW
    }
  }
}

void Encoder::pulseB(){
  int valA = digitalRead(ENC_A);
  int valB = digitalRead(ENC_B);

  if (valB == HIGH) { // B rise
    if (valA == HIGH) {
      enc_count ++; // CW
    }
    else {
      enc_count --; // CCW
    }
  }
  else { //B fall
    if (valA == LOW) {
      enc_count ++; // CW
    }
    else {
      enc_count --; // CCW
    }
  }
}
