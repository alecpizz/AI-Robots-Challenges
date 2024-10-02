#pragma once
#include "LegKinematics.h"
#include <unordered_map>
#include <string>
class SpotKinematics
{
private:
	LegKinematics* _left;
	LegKinematics* _right;
	const double _com_offset = -0.023;
	const double hip_x = (0.364 + 0.308);
	const double hip_y = 2 * 0.053;
	const double foot_x = (0.364 + 0.308);
	const double foot_y = (2 * 0.166);
	const double height = 0.52;
	const double _hip_lower_limit = -0.6;
	const double _hip_upper_limit = 0.5;
	const double _shoulder_lower_limit = -1.7;
	const double _shoulder_upper_limit = 1.7;
	const double _leg_lower_limit = -0.45;
	const double _leg_upper_limit = 1.6;
	std::unordered_map<std::string, LegKinematics*> _legs;
public:
	SpotKinematics();
	~SpotKinematics();
};

