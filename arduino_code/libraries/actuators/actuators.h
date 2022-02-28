#ifndef Servo_h
#include <Servo.h>
#define Servo_h
#endif
#ifndef ENCODER_QUADRATURE_H
#define ENCODER_QUADRATURE_H
#include <encoder_quadrature.h>
#endif
#ifndef PID_H
#include <PID_beard.h>
#define PID_H
#endif
#ifndef ASYNCTIMER_H
#include <AsyncTimer.h>
#define ASYNCTIMER_H
#endif

//Actuator class interface
class Actuator
{
public:
    /* Takes a desired angle in radians and will output
    the proper control signal to the motor its implemented for
    */
    double control_position(double des_angle);
    bool is_ccw_positive;
    int signal_pin;
};

class HobbyServo : public Actuator{
public:
    HobbyServo(int sig_pin, bool ccw_positive);
    double control_position(double des_angle);
    Servo servo;
};

class DCMotorEncoder : public Actuator
{
public:
    DCMotorEncoder(PID_Initializer, int);
    AsyncTimer timer;
    Encoder encoder;
    PIDControl controller;
    double control_position(double des_angle);
    void set_motor_pins(double power);
    double current_position;
    double output;
};

class Stepper : public Actuator
{
public:
    Stepper(int, int, int, double);
    int step_pin;//digital pin for stepping
    int dir_pin; //digital pin for direction
    int spr; //Steps per revolution
    int current_steps;
    double current_position;
    int desired_steps;
    double desired_position;
    double kp;
    double current_velocity;
    bool step_pin_low;
    double dt;
    double upper_limit;
    double lower_limit;

    unsigned long t_prev;
    unsigned long t_now;

    void control_position(double);
    void step(double);
    double saturate(double);
};