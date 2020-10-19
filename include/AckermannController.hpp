#pragma once

#include <iostream>
#include "Robot.hpp"
#include "PID.hpp"

class AckermannController {
public:
	AckermannController();
	AckermannController(Robot robot, PID vel_PID, PID heading_PID);
	void SetDesiredSpeed(double speed);
	void SetDesiredHeading(double heading);
	double GetDesiredSpeed();
	double GetDesiredHeading();
	void Solve();
	Robot car;
	PID velocity_control;
	PID heading_control;
private:

	double desired_speed;
	double desired_heading;
};