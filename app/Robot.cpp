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
#include "../include/Robot.hpp"
#include <iostream>
#include <cmath>

Robot::Robot(){}
Robot::Robot(double width, double base, double size, double left_angle, double right_angle,
             double left_vel, double right_vel) {
  track_width = width;
  wheelbase = base;
  wheel_size = size;
  left_wheel_angle = left_angle;
  right_wheel_angle = right_angle;
  left_wheel_velocity = left_vel;
  right_wheel_velocity = right_vel;
  vehicle_speed = 0;
  vehicle_heading = 0;
}
double Robot::GetTrackWidth() { return track_width; }
double Robot::GetWheelBase() { return wheelbase; }
double Robot::GetWheelSize() { return wheel_size; }
double Robot::GetSpeed() { return vehicle_speed; }
double Robot::GetHeading() { return vehicle_heading; }
double Robot::GetLeftVel() { return left_wheel_velocity; }
void Robot::SetLeftVel(double left_wheel_vel){
	left_wheel_velocity = left_wheel_vel;
}
double Robot::GetRightVel() { return right_wheel_velocity; }
void Robot::SetRightVel(double right_wheel_vel){
	right_wheel_velocity = right_wheel_vel;
}
double Robot::GetLeftAngle() { return left_wheel_angle; }
void Robot::SetLeftAngle(double left_angle){
	left_wheel_angle = left_angle;
}
double Robot::GetRightAngle() { return right_wheel_angle; }
void Robot::SetRightAngle(double right_angle){
	right_wheel_angle = right_angle;
}
void Robot::SetVehicleSpeed(double S){
	vehicle_speed = S;
}
void Robot::SetVehicleHeading(double H){
	vehicle_heading = H;
}
