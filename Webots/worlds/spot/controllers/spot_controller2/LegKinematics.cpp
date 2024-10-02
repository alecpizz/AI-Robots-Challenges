#include "LegKinematics.h"

double LegKinematics::get_domain(double xDist, double yDist, double zDist) const
{
	double d = 0.0;
	d = (pow(yDist, 2) + pow(-zDist, 2) -
		pow(_shoulder_length, 2) -
		pow(_wrist_length, 2)) / (2 * _wrist_length * _elbow_length);

	if (d > 1.0 || d < -1.0)
	{
		d = fmin(1.0, fmax(d, -1.0));
	}
	return d;
}

std::vector<double> LegKinematics::left_IK(double xDist, double yDist, double zDist, double domain) const
{
	double wrist_angle = atan2(-sqrt(1 - pow(domain, 2)), domain);
	double sqrt_component = pow(yDist, 2) + pow(-zDist, 2) - pow(_shoulder_length, 2);
	if (sqrt_component < 0.0)
	{
		sqrt_component = 0.0;
	}
	double shoulder_angle = -atan2(zDist, yDist) - atan2(sqrt(sqrt_component), _shoulder_length);
	double elbow_angle = atan2(-xDist, sqrt(sqrt_component)) -
		atan2(_wrist_length * sin(wrist_angle),
			_elbow_length + _wrist_length * cos(wrist_angle));
	std::vector<double> result = { -shoulder_angle, elbow_angle, wrist_angle };
	return result;
}

std::vector<double> LegKinematics::right_IK(double xDist, double yDist, double zDist, double domain) const
{
	double wrist_angle = atan2(-sqrt(1 - pow(domain, 2)), domain);
	double sqrt_component = pow(yDist, 2) + pow(-zDist, 2) - pow(_shoulder_length, 2);
	if (sqrt_component < 0.0)
	{
		sqrt_component = 0.0;
	}
	double shoulder_angle = atan2(zDist, yDist) - atan2(sqrt(sqrt_component), -_shoulder_length);
	double elbow_angle = atan2(-xDist, sqrt(sqrt_component)) - 
		atan2(_wrist_length * sin(wrist_angle), 
			_elbow_length + _wrist_length * cos(wrist_angle));
	std::vector<double> result = { -shoulder_angle, elbow_angle, wrist_angle };
	return result;
}

std::vector<double> LegKinematics::solve(double xDist, double yDist, double zDist)
{
	double d = get_domain(xDist, yDist, zDist);
	if (_is_left)
	{
		return left_IK(xDist, yDist, zDist, d);
	}
	else
	{
		return right_IK(xDist, yDist, zDist, d);
	}
}
