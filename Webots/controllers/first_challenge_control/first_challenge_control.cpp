// File:          first_challenge_control.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
void move_forward();
void move_back();
void turn_left();
void turn_right();
void reset_motors();
Motor* left_motor;
Motor* right_motor;
const double max_speed = 0.25;
const double magic_turn_num = 4.87;


int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  left_motor = robot->getMotor("leftMotor");
  right_motor = robot->getMotor("rightMotor");
  left_motor->setPosition(INFINITY);
  right_motor->setPosition(INFINITY);
  reset_motors();
  int turn_step_size = static_cast<int>(timeStep * magic_turn_num * (1 + max_speed));
  int progress_step_size = timeStep * 8;
  for (size_t i = 0; i < 4; i++)
  {
      move_forward();
      robot->step(progress_step_size);
      turn_right();
      robot->step(turn_step_size);
  }
  for (size_t i = 0; i < 4; i++)
  {
      move_back();
      robot->step(progress_step_size);
      turn_right();
      robot->step(turn_step_size);
  }

  reset_motors();

  delete robot;
  left_motor = nullptr;
  right_motor = nullptr;
  return 0;
}

void move_forward()
{
    left_motor->setVelocity(max_speed);
    right_motor->setVelocity(max_speed);
}

void move_back()
{
    left_motor->setVelocity(-max_speed);
    right_motor->setVelocity(-max_speed);
}

void turn_left()
{
    left_motor->setVelocity(-max_speed);
    right_motor->setVelocity(max_speed);
}

void turn_right()
{
    left_motor->setVelocity(max_speed);
    right_motor->setVelocity(-max_speed);
}

void reset_motors()
{
    left_motor->setVelocity(0);
    right_motor->setVelocity(0);
}
