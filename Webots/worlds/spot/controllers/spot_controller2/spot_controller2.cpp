// File:          spot_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Device.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>


#define NUMBER_OF_LEDS 8
#define NUMBER_OF_JOINTS 12
#define NUMBER_OF_CAMERAS 5

using namespace webots;

static Motor* motors[NUMBER_OF_JOINTS];
static const char* motor_names[NUMBER_OF_JOINTS] = {
  "front left shoulder abduction motor",  "front left shoulder rotation motor",  "front left elbow motor",
  "front right shoulder abduction motor", "front right shoulder rotation motor", "front right elbow motor",
  "rear left shoulder abduction motor",   "rear left shoulder rotation motor",   "rear left elbow motor",
  "rear right shoulder abduction motor",  "rear right shoulder rotation motor",  "rear right elbow motor" };
static Camera* cameras[NUMBER_OF_CAMERAS];
static const char* camera_names[NUMBER_OF_CAMERAS] = { "left head camera", "right head camera", "left flank camera",
													  "right flank camera", "rear camera" };
static LED* leds[NUMBER_OF_LEDS];
static const char* led_names[NUMBER_OF_LEDS] = { "left top led",          "left middle up led", "left middle down led",
												"left bottom led",       "right top led",      "right middle up led",
												"right middle down led", "right bottom led" };

static void movement_decomposition(const double* target, double duration);
static void lie_down(double duration);
static void stand_up(double duration);
static void sit_down(double duration);
static void left_forward(double duration);
static void right_forward(double duration);
static void left_backward(double duration);
static void right_backward(double duration);
static void move_forward(double duration);
static void move_backward(double duration);
static void turn_left(double duration);
static void turn_right(double duration);
static void give_paw();
static void step();

// All the webots classes are defined in the "webots" namespace

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
Robot* robot = nullptr;

int main(int argc, char** argv) {
	// create the Robot instance.
	robot = new Robot();

	int time_step = (int)robot->getBasicTimeStep();

	for (int i = 0; i < NUMBER_OF_CAMERAS; ++i)
	{
		cameras[i] = robot->getCamera(camera_names[i]);
		cameras[i]->enable(2 * time_step);
	}

	// enable the two front cameras

	// Get the LEDs and turn them on
	for (int i = 0; i < NUMBER_OF_LEDS; ++i)
	{
		leds[i] = robot->getLED(led_names[i]);
		leds[i]->set(1);
	}

	// Get the motors (joints) and set initial target position to 0
	for (int i = 0; i < NUMBER_OF_JOINTS; ++i)
	{
		motors[i] = robot->getMotor(motor_names[i]);
	}

	//while (true)
	//{
	//	//move_forward(1.0);
	//	//move_forward(1.0);
	//	move_backward(1.0);
	//	//move_backward(1.0);
	//}
	/*	lie_down(4.0);
		stand_up(4.0);
		sit_down(4.0);
		give_paw();
		stand_up(4.0);
		lie_down(3.0);
		stand_up(3.0);
		lie_down(2.0);
		stand_up(2.0);
		lie_down(1.0);
		stand_up(1.0);
		lie_down(0.75);
		stand_up(0.75);
		lie_down(0.5);
		stand_up(0.5);*/


		// get the time step of the current world

		// You should insert a getDevice-like function in order to get the
		// instance of a device of the robot. Something like:
		//  Motor *motor = robot->getMotor("motorname");
		//  DistanceSensor *ds = robot->getDistanceSensor("dsname");
		//  ds->enable(timeStep);

		// Main loop:
		// - perform simulation steps until Webots is stopping the controller


		// Enter here exit cleanup code.

	delete robot;
	return 0;
}

void movement_decomposition(const double* target, double duration)
{
	int timeStep = (int)robot->getBasicTimeStep();
	const int n_steps_to_achieve_target = duration * 1000 / timeStep;
	double step_difference[NUMBER_OF_JOINTS];
	double current_position[NUMBER_OF_JOINTS];

	for (int i = 0; i < NUMBER_OF_JOINTS; ++i)
	{
		current_position[i] = motors[i]->getTargetPosition();
		step_difference[i] = (target[i] - current_position[i]) / n_steps_to_achieve_target;
	}

	for (int i = 0; i < n_steps_to_achieve_target; ++i)
	{
		for (int j = 0; j < NUMBER_OF_JOINTS; ++j)
		{
			current_position[j] += step_difference[j];
			motors[j]->setPosition(current_position[j]);
		}
		step();
	}
}

void lie_down(double duration)
{
	const double motors_target_pos[NUMBER_OF_JOINTS] = { -0.40, -0.99, 1.59,   // Front left leg
													  0.40,  -0.99, 1.59,   // Front right leg
													  -0.40, -0.99, 1.59,   // Rear left leg
													  0.40,  -0.99, 1.59 };  // Rear right leg
	movement_decomposition(motors_target_pos, duration);
}

void stand_up(double duration)
{
	const double motors_target_pos[NUMBER_OF_JOINTS] = { -0.1, 0.0, 0.0,   // Front left leg
													  0.1,  0.0, 0.0,   // Front right leg
													  -0.1, 0.0, 0.0,   // Rear left leg
													  0.1,  0.0, 0.0 };  // Rear right leg

	movement_decomposition(motors_target_pos, duration);
}

void sit_down(double duration)
{
	const double motors_target_pos[NUMBER_OF_JOINTS] = { -0.20, -0.40, -0.19,  // Front left leg
													  0.20,  -0.40, -0.19,  // Front right leg
													  -0.40, -0.90, 1.18,   // Rear left leg
													  0.40,  -0.90, 1.18 };  // Rear right leg

	movement_decomposition(motors_target_pos, duration);
}

void left_forward(double duration)
{
	const double motors_target_pos[NUMBER_OF_JOINTS] = { 
		0, 0.25, 0.0,   // Front left leg
		0,  0.0, 0.0,   // Front right leg
		0, 0.25, 0.0,   // Rear left leg
		0,  0.0, 0.0 };  // Rear right leg
	movement_decomposition(motors_target_pos, duration);
}

void right_forward(double duration)
{
	const double motors_target_pos[NUMBER_OF_JOINTS] = {
		0, 0.00, 0.0,   // Front left leg
		0,  0.25, 0.0,   // Front right leg
		0, 0.00, 0.0,   // Rear left leg
		0,  0.25, 0.0 };  // Rear right leg
	movement_decomposition(motors_target_pos, duration);
}

void left_backward(double duration)
{
	const double motors_target_pos[NUMBER_OF_JOINTS] = {
		0, -0.25, 0.0,   // Front left leg
		0,  0.0, 0.0,   // Front right leg
		0, -0.25, 0.0,   // Rear left leg
		0,  0.0, 0.0 };  // Rear right leg
	movement_decomposition(motors_target_pos, duration);
}

void right_backward(double duration)
{
	const double motors_target_pos[NUMBER_OF_JOINTS] = {
		0, 0.00, 0.0,   // Front left leg
		0,  -0.25, 0.0,   // Front right leg
		0, 0.00, 0.0,   // Rear left leg
		0,  -0.25, 0.0 };  // Rear right leg
	movement_decomposition(motors_target_pos, duration);
}

void move_forward(double duration)
{
	left_forward(duration * 0.5);
	right_forward(duration * 0.5);
}

void move_backward(double duration)
{
	left_backward(duration * 0.5);
	right_backward(duration * 0.5);
}

void turn_left(double duration)
{
	const double motors_target_pos[NUMBER_OF_JOINTS] = {
		0.25, 0.25, 0.0,   // Front left leg
		0,  0.0, 0.0,   // Front right leg
		0, 0.0, 0.0,   // Rear left leg
		0.25,  0.25, 0.0 };  // Rear right leg
	movement_decomposition(motors_target_pos, duration);
}

void turn_right(double duration)
{
}

void give_paw()
{
	const double motors_target_pos_1[NUMBER_OF_JOINTS] = { -0.20, -0.30, 0.05,   // Front left leg
														0.20,  -0.40, -0.19,  // Front right leg
														-0.40, -0.90, 1.18,   // Rear left leg
														0.49,  -0.90, 0.80 };  // Rear right leg

	movement_decomposition(motors_target_pos_1, 4);

	const double initial_time = robot->getTime();
	while (robot->getTime() - initial_time < 8)
	{
		motors[4]->setPosition(0.2 * sin(2 * robot->getTime() + 0.6));
		motors[5]->setPosition(0.4 * sin(2 * robot->getTime()));
		step();
	}
	// Get back in sitting posture
	const double motors_target_pos_2[NUMBER_OF_JOINTS] = { -0.20, -0.40, -0.19,  // Front left leg
														  0.20,  -0.40, -0.19,  // Front right leg
														  -0.40, -0.90, 1.18,   // Rear left leg
														  0.40,  -0.90, 1.18 };  // Rear right leg

	movement_decomposition(motors_target_pos_2, 4);
}

void step()
{
	int timeStep = (int)robot->getBasicTimeStep();
	if (robot->step(timeStep) == -1)
	{
		exit(0);
	}
}
