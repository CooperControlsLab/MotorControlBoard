#ifndef ACTUATORS_H
#include <actuators.h>
#define ACTUATORS_H
#endif
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
#ifndef ARDUINO_H
#define ARDUINO_H
#include <Arduino.h>
#endif
#ifndef ASYNCTIMER_H
#include <AsyncTimer.h>
#define ASYNCTIMER_H
#endif


DCMotorEncoder::DCMotorEncoder(PID_Initializer pid_init, int ppr)
{   PID_Initializer p = pid_init;
    encoder = Encoder(ppr) ;
    controller = PIDControl(pid_init);
    timer = AsyncTimer(pid_init.Ts, 1,1);
    encoder.setup_interrupts();

    current_position=0;
    output=0;
}

double DCMotorEncoder::control_position(double des_angle)
{
    //Check timing variables
    timer.check_dt();
    if(timer.run_process[0])
    {   // Get position from count
        current_position = (2*3.14159*encoder.enc_count/encoder.ppr);
        
        //Check if we're within 2 degrees
        if(abs(des_angle - current_position) < (3.14159/90) )
        {
            set_motor_pins(0);
            return 0;
        }
        
        output = controller.PID(des_angle, current_position);
        set_motor_pins(output);
        return output;
    }

}

void DCMotorEncoder::set_motor_pins(double power)
{
    if(power >= 0)
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }
    else if(power < 0)
    {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    }

    analogWrite(ENA, abs(power));
}


Stepper::Stepper(int step, int dir, int steps_per_revolution, double proportional_gain)
{
    kp = proportional_gain;
    step_pin = step;
    dir_pin = dir;
    spr = steps_per_revolution;
    current_position = 0;
    current_steps = 0;
    t_prev = micros();

    //Set pins to outputs
    pinMode(step_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);

    //Make sure step pin is in low state
    digitalWrite(step_pin, LOW);
    step_pin_low = true;
}

//Control the position of a stepper motor (des_angle is in radians)
void Stepper::control_position(double des_angle)
{
    desired_position = des_angle;

    //Convert radian position into the nearest # of steps
    desired_steps = (int)(des_angle*spr / (2*3.14159));

    //P controller: desired_steps - current_steps = desired velocity (in steps / sec)
    int error = desired_steps - current_steps;
    current_velocity = kp * error;

    //Saturate current velocity
    current_velocity = saturate(current_velocity);

    step(current_velocity);
}

void Stepper::step(double vel)
{
    t_now = micros();
    //Check time passed since last call
    dt = (t_now - t_prev)/1000000.0;

    //Check and set direction of step
    if(vel >= 0)
    {
        digitalWrite(dir_pin, LOW);
    }
    else if (vel < 0)
    {
        digitalWrite(dir_pin, HIGH);
    }

    //Check if step pin is in high mode, if so set to low
    if (!step_pin_low)
    {
        digitalWrite(step_pin, LOW);
        step_pin_low = true;
    }

    //Check if enough time has passed to step the motor
    if(vel != 0) //Make sure vel is not zero prevent divide by zero
    {
        if((dt > abs(1/vel)) && step_pin_low)
        {
            //Step the motor
            digitalWrite(step_pin, HIGH);

            //Increment/decrement the step value
            if(vel > 0){current_steps ++;}
            if(vel < 0){current_steps --;}

            //Reset the t_prev variables
            t_prev = micros();

            //Set step pin as high
            step_pin_low = false;
        }
    }
}

double Stepper::saturate(double u){
    return max(min(upper_limit, u), lower_limit);
}