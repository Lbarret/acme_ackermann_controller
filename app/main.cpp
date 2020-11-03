/**
 * @file       main.cpp
 * @version    1.0
 * @brief      This file is the main function which instantiate the object and implemets the function.
 * @created on 20th Oct 2020
 * @copyright  Copyright 2020. All rights reserved
 * @Author :   Divyam Garg (Driver), Loic Barret (Navigator), Aditya Goswami (Design Keeper),
 */

// user defined header files for PID Controller, ackermann controller and robot state
#include "../include/Robot.hpp"
#include "../include/PID.hpp"
#include "../include/AckermannController.hpp"

// c++ header file
#include <iostream>

int main() {
	// Robot parameterized constructor creates object and sets values
	Robot robo(.3, 1, .15, 0, 0, 0, 0);
	// Setter function is called to set the target velocity and actual velocity
	PID vel_pid(.25, 0, .1);
	// Setter function is called to set the target velocity and actual velocity
	PID head_pid(.25, 0, .1);
	//control object is created for ackermann controller
	AckermannController control(robo,vel_pid, head_pid);
	//In control object heading is set
	control.SetDesiredHeading(80);
	//In control object speed is set
	control.SetDesiredSpeed(20);
	//solver is called to converge the set speed and angles to desired output
	control.Solve();
}
