/**
 * @file       PID.hpp
 * @version    1.0
 * @brief      Header file for PID class.
 * @created on 20th Oct 2020
 * @copyright  Copyright 2020. All rights reserved
 * @Author :   Divyam Garg (Driver), Loic Barret (Navigator), Aditya Goswami (Design Keeper),
 */
#pragma once

// c++ header file
#include <iostream>

/**
 * The PIDController class has variables for each of the term as well as time
 * interval and error variables. It has the method to implement PID controller to achieve the
 * desired goal and methods to get the error variables and time interval.
 */
class PID {
	// Contains public data members
public:
	PID();
	PID(double prop_gain, double int_gain, double diff_gain);
	double GetKp();
	double GetKi();
	double GetKd();
	// Contains private data members
private:
	double kp, ki, kd;
};
