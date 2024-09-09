// File:          second_challenge_control.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <numeric>


// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
Motor* left_motor = nullptr;
Motor* right_motor = nullptr;
double max_speed = 0.35;

void move_forward()
{
    left_motor->setVelocity(max_speed);
    right_motor->setVelocity(max_speed);
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

void move_back()
{
    left_motor->setVelocity(-max_speed);
    right_motor->setVelocity(-max_speed);
}

void drive_forward(double angle, bool inRadians = false)
{
    if (inRadians)
    {
        angle *= 57.295;
    }
    if (angle > 90 || angle < -90)
    {
        return;
    }
    else if (angle >= 0)
    {
        left_motor->setVelocity(max_speed);
        right_motor->setVelocity(max_speed * sin((angle + 45) / 28.6458));
    }
    else
    {
        left_motor->setVelocity(max_speed * sin((-angle + 45) / 28.6458));
        right_motor->setVelocity(max_speed);
    }
}

void drive_backward(double angle, bool inRadians = false)
{
    if (inRadians)
    {
        angle *= 57.295;
    }
    if (angle > 90 || angle < -90)
    {
        return;
    }
    else if (angle >= 0)
    {
        left_motor->setVelocity(-max_speed);
        right_motor->setVelocity(-max_speed * sin((angle + 45) / 28.6458));
    }
    else
    {
        left_motor->setVelocity(-max_speed * sin((-angle + 45) / 28.6458));
        right_motor->setVelocity(-max_speed);
    }
}

void reset_motors()
{
    left_motor->setVelocity(0);
    right_motor->setVelocity(0);
}


int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = 64;

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
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  //while (robot->step(timeStep) != -1) {
  //  // Read the sensors:
  //  // Enter here functions to read sensor data, like:
  //  //  double val = ds->getValue();

  //  // Process sensor data here.

  //  // Enter here functions to send actuator commands, like:
  //  //  motor->setPosition(10.0);
  //};
      drive_forward(0);
      robot->step(timeStep * 10);
      drive_forward(90);
      robot->step(timeStep * 5);
      drive_forward(0);
      robot->step(timeStep * 10);
      drive_forward(-90);
      robot->step(timeStep * 5);
      drive_forward(10);
      robot->step(timeStep * 3);
      drive_forward(0);
      robot->step(timeStep * 11);
      drive_forward(-90);
      robot->step(timeStep * 5);
      drive_forward(0);
      robot->step(timeStep * 15);
      drive_forward(-10);
      robot->step(timeStep * 5);
      drive_forward(45);
      robot->step(int(timeStep * 2.5));
      drive_backward(0);
      robot->step(int(timeStep * 11));
      reset_motors();
      robot->step(timeStep * 3);
      turn_left();
      robot->step(int(timeStep * 2));
      reset_motors();
      robot->step(timeStep * 3);
      drive_forward(0);
      robot->step(timeStep * 3);
      turn_right();
      robot->step(timeStep);
      drive_backward(0);
      robot->step(int(timeStep * 3.5));
      turn_left();
      robot->step(timeStep);
      drive_forward(0);
      robot->step(timeStep * 1);
      drive_forward(-10);
      robot->step(timeStep * 2);
      drive_backward(-45);
      robot->step(int(timeStep * 3.5));
      drive_backward(0);
      robot->step(int(timeStep * 0.5));
      drive_forward(-45);
      robot->step(int(timeStep * 3));
      reset_motors();
  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
