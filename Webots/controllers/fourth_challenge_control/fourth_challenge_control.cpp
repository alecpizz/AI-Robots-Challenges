// File:          fourth_challenge_control.cpp
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

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
Motor* left_motor;
Motor* right_motor;
Camera* camera;
Motor* camera_motor;
const double max_speed = 0.65;

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
		right_motor->setVelocity(max_speed * sin((angle + 45) / 28.648));
	}
	else
	{
		right_motor->setVelocity(max_speed);
		left_motor->setVelocity(max_speed * sin((-angle + 45) / 28.648));
	}
}


int main(int argc, char** argv) {
	// create the Robot instance.
	Robot* robot = new Robot();

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

	double camera_min_position = 0.65;
	double camera_max_position = -1;
	double camera_default_position = 0;

	camera = robot->getCamera("CameraArm_camera");
	camera->enable(timeStep);
	int width = camera->getWidth();
	int height = camera->getHeight();

	camera_motor = robot->getMotor("CameraArm_rotational_motor");
	camera_motor->setPosition(camera_min_position);
	// Main loop:
	// - perform simulation steps until Webots is stopping the controller
	while (robot->step(timeStep) != -1) {
		// Read the sensors:
		// Enter here functions to read sensor data, like:
		//  double val = ds->getValue();

		// Process sensor data here.

		// Enter here functions to send actuator commands, like:
		//  motor->setPosition(10.0);

		int red = camera->imageGetRed(camera->getImage(), width, width / 2, height / 2);
		int green = camera->imageGetGreen(camera->getImage(), width, width / 2, height / 2);
		int blue = camera->imageGetBlue(camera->getImage(), width, width / 2, height / 2);
		if (green > 110 && blue > 90 && blue < 110)
		{
			drive_forward(-30);
		}
		else if (green > 200 && blue > 190 && red > 200)
		{
			drive_forward(30);
		}
		else
		{
			drive_forward(0);
		}
	};

	// Enter here exit cleanup code.

	delete robot;
	return 0;
}
