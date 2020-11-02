/**
 * @file       robot.hpp
 * @version    1.0
 * @brief      This file is the main function which instantiate the object and implemets the function.
 * @created on 20th Oct 2020
 * @copyright  Copyright 2020. All rights reserved
 * @Author :   Divyam Garg (Driver), Loic Barret (Navigator), Aditya Goswami (Design Keeper),
 */

#pragma once

// c++ header file
#include <iostream>

/**
 * The robot class has variables for each of the term as well as time
 * interval and error variables. It has the method to implement PID controller to achieve the
 * desired goal and methods to get the error variables and time interval.
 */
class Robot {
public:
 Robot();
 Robot(double width, double base, double size, double left_angle, double right_angle, double left_vel, double right_vel);
 double GetTrackWidth();
 double GetWheelBase();
 double GetWheelSize();
 double GetSpeed();
 double GetHeading();
 double GetLeftVel();
 void SetLeftVel(double left_wheel_vel);
 double GetRightVel();
 void SetRightVel(double right_wheel_vel);
 double GetLeftAngle();
 void SetLeftAngle(double left_angle);
 double GetRightAngle();
 void SetRightAngle(double right_angle);
 void SetVehicleSpeed(double S);
 void SetVehicleHeading(double H);
private:
 double track_width;
 double wheelbase;
 double wheel_size;
 double vehicle_speed;
 double vehicle_heading;
 double left_wheel_angle;
 double right_wheel_angle;
 double left_wheel_velocity;
 double right_wheel_velocity;
};
