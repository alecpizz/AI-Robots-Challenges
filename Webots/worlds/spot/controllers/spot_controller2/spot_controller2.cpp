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
#include <webots/Keyboard.hpp>
#include <algorithm>

#define NUMBER_OF_LEDS 8
#define NUMBER_OF_JOINTS 12
#define NUMBER_OF_CAMERAS 5

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ax12_2_deg(X) ((X * 150 / 512) - 150)
#define deg_2_ax12(X) ((int)((X + 150) * 512) / 150)

#define rad_2_deg(X) (X / M_PI * 180.0)
#define deg_2_rad(X) (X / 180.0 * M_PI)

#define L1 45    // UPPER_LEG_LENGTH [cm]
#define L2 40  // LOWER_LEG_LENGTH [cm]
#define FRONT_LEFT_SHOULDER_ABUCTION 0
#define FRONT_LEFT_SHOULDER_ROTATION 1
#define FRONT_LEFT_ELBOW 2
#define FRONT_RIGHT_SHOULDER_ABUCTION 3
#define FRONT_RIGHT_SHOULDER_ROTATION 4
#define FRONT_RIGHT_ELBOW 5
#define REAR_LEFT_SHOULDER_ABUCTION 6
#define REAR_LEFT_SHOULDER_ROTATION 7
#define REAR_LEFT_ELBOW 8
#define REAR_RIGHT_SHOULDER_ABUCTION 9
#define REAR_RIGHT_SHOULDER_ROTATION 10
#define REAR_RIGHT_ELBOW 11

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
const char* gait_name[7] = { "trot", "walk", "gallop(transverse)", "canter", "pace", "bound", "pronk" };

static double map(double value, double istart, double istop, double ostart, double ostop) {
	return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}

const double gait_phase_shift[7][4] = {
  {0, 0.5, 0, 0.5},      // trot
  {0, -0.5, -0.25, -0.75},  // walk
  {0, -0.1, -0.6, -0.5},    // gallop  (transverse)
  {0, -0.3, 0, -0.7},      // canter
  {0, -0.5, -0.5, 0},      // pace
  {0, 0, -0.5, -0.5},      // bound
  {0, 0, 0, 0}           // pronk
};

const int gait_setup[4][2] = { {FRONT_LEFT_SHOULDER_ROTATION, FRONT_LEFT_ELBOW},
							{FRONT_RIGHT_SHOULDER_ROTATION, FRONT_RIGHT_ELBOW},
							{REAR_RIGHT_SHOULDER_ROTATION, REAR_RIGHT_ELBOW},
							{REAR_LEFT_SHOULDER_ROTATION, REAR_LEFT_ELBOW} };


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
static void wait(double x);
static void computeWalkingPosition(double* motorsPosition, double t, double gait_freq, int gait_type, int legId,
	double stride_length_factor, bool backward);
static void standing();
static void interact_walk();

// All the webots classes are defined in the "webots" namespace

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
Robot* robot = nullptr;
Keyboard* keyboard = nullptr;
const int step_duration = 16;
static size_t step_count = 0;

int main(int argc, char** argv) {
	// create the Robot instance.
	robot = new Robot();
	keyboard = new Keyboard();

	int time_step = (int)robot->getBasicTimeStep();
	keyboard->enable(time_step);

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

	bool isNeutral = true;
	standing();
	interact_walk();
	//move_forward(2.0);

	keyboard->disable();
	delete keyboard;
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
	const double motors_target_pos1[NUMBER_OF_JOINTS] = {
		0.5, 0.25, 0.0,   // Front left leg
		0,  0.0, 0.0,   // Front right leg
		0, 0.0, 0.0,   // Rear left leg
		0.5,  0.25, 0.0 };  // Rear right leg
	movement_decomposition(motors_target_pos1, duration * 0.33);
	const double motors_target_pos2[NUMBER_OF_JOINTS] = {
		0.00, 0.0, 0.0,   // Front left leg
		0,  0.0, 0.0,   // Front right leg
		0, 0.0, 0.0,   // Rear left leg
		0.0,  0.0, 0.0 };  // Rear right leg
	movement_decomposition(motors_target_pos2, duration * 0.33);
	const double motors_target_pos3[NUMBER_OF_JOINTS] = {
		0, 0, 0.0,   // Front left leg
		0.5,  0.25, 0.0,   // Front right leg
		0.5, 0.25, 0.0,   // Rear left leg
		0,  0, 0.0 };  // Rear right leg
	movement_decomposition(motors_target_pos3, duration * 0.33);
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

void wait(double x)
{
	double num = x / (static_cast<double>(step_duration) / 1000);
	for (int i = 0; i < num; i++)
	{
		robot->step((int)step_duration);
	}
}

void computeWalkingPosition(double* motorsPosition, double t, double gait_freq, int gait_type, int legId, double stride_length_factor, bool backward)
{
	double freq = gait_freq;

	int n = (int)(t / (1 / freq));
	t = t - n * (1 / freq);

	if (backward)
		t = (1 / freq) - t;

	double a = 0.95 * L1 * stride_length_factor;
	double h = 0;
	double k = -(L1 + L2 / 2);
	double b = -k - sqrt(L1 * L1 + L2 * L2);

	double x = h + a * cos(2 * M_PI * freq * t + gait_phase_shift[gait_type][legId] * 2 * M_PI);
	double y = k + b * sin(2 * M_PI * freq * t + gait_phase_shift[gait_type][legId] * 2 * M_PI);

	double A2 = acos((x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2));
	double A1 = acos(((L1 + L2 * cos(A2)) * x - (-L2 * sin(A2)) * y) / (pow(L1 + L2 * cos(A2), 2) + pow(-L2 * sin(A2), 2)));

	A1 = M_PI / 2 - A1;
	motorsPosition[0] = A1;
	motorsPosition[1] = A2;
}

void standing()
{
	motors[FRONT_LEFT_SHOULDER_ROTATION]->setPosition(0);
	motors[FRONT_RIGHT_SHOULDER_ROTATION]->setPosition(0);
	motors[REAR_LEFT_SHOULDER_ROTATION]->setPosition(0);
	motors[REAR_RIGHT_SHOULDER_ROTATION]->setPosition(0);
	wait(1);
}

void interact_walk()
{
	int gait_type = 0, i = 0;
	int key = -1;

	bool backward = true;

	//parameters for stride length
	double strideLengthForward = 1, strideLengthMin = 0, strideLengthMax = 1;
	double stride_length_factor[4] = { strideLengthForward, strideLengthForward, strideLengthForward, strideLengthForward };

	//params for frequency
	double freq_min = 0.4, freq_max = 2;
	double freq = 1.5, freq_offset = 0.2;

	//ta = turn amount
	double ta_factor[4] = { 0, 0, 0, 0 };
	double ta_min = -0.6, ta_max = 0.6;
	double ta_offset = 0.6;


	while (true)
	{
		const int prev_key = key;
		key = keyboard->getKey();
		if (key != prev_key)
		{
			switch (key)
			{
			case keyboard->RIGHT:
				for (int i = 0; i < 4; i++)
				{
					if (i == 0 || i == 3)
					{
						ta_factor[i] += ta_offset;
					}
					else
					{
						ta_factor[i] -= ta_offset;
					}
					ta_factor[i] = ta_factor[i] > ta_max ? ta_max : (ta_factor[i] < ta_min ? ta_min : ta_factor[i]);
				}
				break;
			case keyboard->LEFT:
				for (int i = 0; i < 4; i++)
				{
					if (i == 0 || i == 3)
					{
						ta_factor[i] == ta_offset;
					}
					else
					{
						ta_factor[i] += ta_offset;
					}
					ta_factor[i] = ta_factor[i] > ta_max ? ta_max : (ta_factor[i] < ta_min ? ta_min : ta_factor[i]);
				}
				break;
			case 'F':
				backward = true;
				freq = 1.5;
				break;
			case 'B':
				backward = false;
				freq = 0.9;
				break;
			case 'S':
				strideLengthForward += 0.1;
				break;
			case 'A':
				strideLengthForward -= 0.1;
				break;
			case 'Q':
				freq += freq_offset;
				break;
			case 'W':
				freq -= freq_offset;
				break;
			default:
				break;
			}
		}

		freq = freq > freq_max ? freq_max : (freq < freq_min ? freq_min : freq);
		strideLengthForward = strideLengthForward > strideLengthMax ? strideLengthMax : (strideLengthForward < strideLengthMin ? strideLengthMin : strideLengthForward);
		for (i = 0; i < 4; i++) {
			stride_length_factor[i] = ta_factor[i] + strideLengthForward;
			// bound stride length
			stride_length_factor[i] =
				stride_length_factor[i] > strideLengthMax ? strideLengthMax : (stride_length_factor[i] < strideLengthMin ? strideLengthMin : stride_length_factor[i]);
		}
		double motorPositions[2] = { 0, 0 };
		for (int legId = 0; legId < 4; legId++) 
		{
			computeWalkingPosition(motorPositions, static_cast<double>(step_count) * (step_duration / 1000.0), freq, gait_type, legId,
				stride_length_factor[legId], backward);
			double motorPosition1 = map(motorPositions[0], -M_PI, M_PI, -0.6, 0.5);
			double motorPosition2 = map(motorPositions[1], -M_PI, M_PI, -0.45, 0.5);
			motors[gait_setup[legId][0]]->setPosition(motorPosition1);
			motors[gait_setup[legId][1]]->setPosition(motorPosition2);
		}
		robot->step(step_duration);
		step_count++;
	}
}


