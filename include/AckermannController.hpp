/**
 * @file       AckermannController.hpp
 * @version    1.0
 * @brief      Header file for AckermannController class.
 * @created on 20th Oct 2020
 * @copyright  Copyright 2020. All rights reserved
 * @Author :   Divyam Garg (Driver), Loic Barret (Navigator), Aditya Goswami (Design Keeper),
 */
#pragma once

// user defined header file for PID Controller and robot state
#include "Robot.hpp"
#include "PID.hpp"

// c++ header file
#include <iostream>
#include <vector> 
#include <string>

/**
 * The AckermannController class has variables for each of the term as well as time
 * interval and error variables. It has the method to implement Ackermann controller to achieve the
 * desired goal and methods to get the error variables and time interval.
 */
class AckermannController {
	// Contains public data members
public:
	AckermannController();
	AckermannController(Robot robot, PID vel_PID, PID heading_PID);
	void SetDesiredSpeed(double speed);
	void SetDesiredHeading(double heading);
	double GetDesiredSpeed();
	double GetDesiredHeading();
	void CalculateVehicleSpeed();
	void CalculateVehicleHeading();
	void CalculateWheelVelocities(double req_speed);
	void CalculateWheelAngles(double req_heading);
	void plot(std::vector<double> x, std::vector<double> y, std::string name);
	void Solve();
	Robot car;
	PID velocity_control;
	PID heading_control;
	double timestamp;
	// Contains private data members
private:

	double desired_speed;
	double desired_heading;
};
