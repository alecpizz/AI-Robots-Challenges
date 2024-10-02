#include "SpotKinematics.h"
#include <string>
#include <unordered_map>
SpotKinematics::SpotKinematics()
{
	_left = new LegKinematics(true);
	_right = new LegKinematics(false);
	_legs["FL"] = _left;
	_legs["FR"] = _right;
	_legs["BL"] = _left;
	_legs["BR"] = _right;
}

SpotKinematics::~SpotKinematics()
{
	delete _left;
	delete _right;
}
