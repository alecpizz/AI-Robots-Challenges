#pragma once
#include <vector>
class LegKinematics
{
private:
	bool _is_left;
	const double _shoulder_length = 0.113;
	const double _elbow_length = 0.368;
	const double _wrist_length = 0.352 + 0.035094;
	double get_domain(double xDist, double yDist, double zDist) const;
	std::vector<double> left_IK(double xDist, double yDist,
		double zDist, double domain) const;
	std::vector<double> right_IK(double xDist, double yDist,
		double zDist, double domain) const;
public:
	LegKinematics(bool left) : _is_left(left)
	{

	}

	std::vector<double> solve(double xDist, double yDist, double zDist);
};

