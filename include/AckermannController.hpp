#pragma once

#include <iostream>
#include "Robot.hpp"
#include "PID.hpp"

class AckermannController {
public:
	AckermannController(Robot robot, PID vel_PID, PID heading_PID);
	void SetDesiredSpeed(double speed);
	void SetDesiredHeading(double heading);
	void Solve();
private:
	Robot car;
	PID velocity_control;
	PID heading_control;
	double desired_speed;
	double desired_heading;
};
