#ifndef ENCODER_QUADRATURE_H
#define ENCODER_QUADRATURE_H
#include <encoder_quadrature.h>
#endif
#ifndef CONTROL_BOARD_CONFIG_H
#include <control_board_config.h>
#define CONTROL_BOARD_CONFIG_h
#endif
#ifndef ACTUATORS_H
#include <actuators.h>
#define ACTUATORS_H
#endif
#ifndef PID_H
#include <PID_beard.h>
#define PID_H
#endif

#include <Servo.h>

struct PID_Initializer pid_init = {
  kp: 150,
  ki: 50,
  kd: 10,
  lowerLimit: -175,
  upperLimit: 175,
  sigma: 0.01,
  Ts: 0.01,
flag: true,
};

//Define actuators
Servo ser;
DCMotorEncoder dcmotor(pid_init, PPR);
Stepper stepper(STEPPER_STEP, STEPPER_DIR, 200, 5);

void setup()
{
  Serial.begin(115200);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  //Hobby servo motor attach
  ser.attach(6);

  //Dcmotor extra parameters
  dcmotor.controller.update_deadband_voltages(1.5*255.0/9, -1.5*255.0/9);
  dcmotor.controller.anti_windup_activated=true;

  //Stepper motor extra parameters
  stepper.lower_limit = -180;
  stepper.upper_limit = 180;
}

void loop()
{
  int pot = analogRead(A0);
  double pos = mapf(pot, 0, 1023, PI/2, -PI/2);
  
  //Control DC motor (radians)
  double out = dcmotor.control_position(pos);
  //Control stepper motor (radians)
  stepper.control_position(pos);
  //Control servo (degrees)
  int servo_degrees = 90 + (pos * 180/PI);
  ser.write(servo_degrees);
  
  Serial.print(stepper.dt);
  Serial.print(" ");
  Serial.println(stepper.current_velocity);
    
  if(dcmotor.timer.run_process[0])
  {
//    ser.servo.write(90);
    
  }
}

double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
