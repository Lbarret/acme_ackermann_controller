/**
 * @file       pid_controller.cpp
 * @version    1.0
 * @brief      This file is the main function which instantiate the object and implemets the function.
 * @created on 20th Oct 2020
 * @copyright  Copyright 2020. All rights reserved
 * @Author :   Divyam Garg (Driver), Loic Barret (Navigator), Aditya Goswami (Design Keeper),
 */

// user defined header files for PID Controller
#include "../include/PID.hpp"

// c++ header file
#include <iostream>

/**
 * @brief PID default constructor
 * @param none
 * @return none
 */
PID::PID() { }

/**
 * @brief    PID parameterized constructor sets values of gains
 * @param[1] prop_gain, which is the proportional gain
 * @param[2] int_gain, which is the intgral gain
 * @param[3] diff_gain, which is the differential gain
 * @return 	 none
 */
PID::PID(double prop_gain, double int_gain, double diff_gain){
	kp = prop_gain;
	ki = int_gain;
	kd = diff_gain;
}
/**
 * @brief      Next three functions return values of kp, ki and kd
 * @param      none
 * @return     The propotional gain.
 * @return     The integral gain
 * @return     The differential gain
 */
double PID::GetKp() {
	return kp;
}
double PID::GetKi() {
	return ki;
}
double PID::GetKd() {
	return kd;
}
