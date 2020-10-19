/**
 * @file Robot.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2020-10-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#pragma once

#include <iostream>

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
