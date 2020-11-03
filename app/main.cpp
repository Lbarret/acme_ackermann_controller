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

	float des_heading = 0, des_speed = 0;
	// Robot parameterized constructor creates object and sets values
	Robot robo(.3, 1, .15, 0, 0, 0, 0);
	// Setter function is called to set the target velocity and actual velocity
	PID vel_pid(.25, 0, .1);
	// Setter function is called to set the target velocity and actual velocity
	PID head_pid(.25, 0, .1);
	//control object is created for ackermann controller
	AckermannController control(robo,vel_pid, head_pid);

	std::cout << "Input Desired Heading [deg]: ";
	std::cin >> des_heading;
	//In control object heading is set
	control.SetDesiredHeading(des_heading);

	std::cout << "Input Desired Speed [m/s]: ";
	std::cin >> des_speed;
	//In control object speed is set
	control.SetDesiredSpeed(des_speed);
	//solver is called to converge the set speed and angles to desired output
	control.Solve();
}
