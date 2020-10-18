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
 Robot(double width, double base, double size, double speed, double heading, double left_angle, double right_angle, double left_vel, double right_vel);
 double GetTrackWidth();
 double GetWheelBase();
 double GetWheelSize();
 double GetSpeed();
 double GetHeading();
 double GetLeftVel();
 double GetRightVel();
 double GetLeftAngle();
 double GetRightAngle();
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
