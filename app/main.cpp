#include <iostream>
#include "../include/Robot.hpp"
#include "../include/PID.hpp"
#include "../include/AckermannController.hpp"

int main()
{
	Robot robo(.3, 1, .15, 0, 0, 0, 0);
	PID vel_pid(.1, 0, .1);
	PID head_pid(.1, 0, .1);
	AckermannController control(robo,vel_pid, head_pid);
	control.SetDesiredHeading(80);
	control.SetDesiredSpeed(10);
	control.Solve();

}
