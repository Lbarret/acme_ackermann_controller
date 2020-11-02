/**
 * @file       robot_state.cpp
 * @version    1.0
 * @brief      This file is the main function which instantiate the object and implemets the function.
 * @created on 20th Oct 2020
 * @copyright  Copyright 2020. All rights reserved
 * @Author :   Divyam Garg (Driver), Loic Barret (Navigator), Aditya Goswami (Design Keeper),
 */

// user defined header files for robot state
#include "../include/Robot.hpp"
#include <cmath>

// c++ header file
#include <iostream>

/**
 * @brief Robot default constructor
 * @param none
 * @return none
 */
Robot::Robot() { }

/**
 * @brief    Robot parameterized constructor sets values of states of robot
 * @param[1] width of track
 * @param[2] base of robot
 * @param[3] size of robot
 * @param[4] left_angle of robot's left wheel
 * @param[5] right_angle of robot's right wheel
 * @param[6] left_vel of robot's left wheel
 * @param[7] right_vel of robot's right wheel
 * @return   none
 */
Robot::Robot(double width, double base, double size, double left_angle, double right_angle,
             double left_vel, double right_vel) {
  track_width          = width;
  wheelbase            = base;
  wheel_size           = size;
  left_wheel_angle     = left_angle;
  right_wheel_angle    = right_angle;
  left_wheel_velocity  = left_vel;
  right_wheel_velocity = right_vel;
  vehicle_speed        = 0;
  vehicle_heading      = 0;
}

/**
 * @brief      Next nine functions return values of variables of parameterized constructor
 * @param      none
 * @return     The width of track
 * @return     The base of robot
 * @return     The size of robot
 * @return     The speed of the vehicle
 * @return     The heading (direction) of vehicle
 * @return     The velocity of robot's left wheel
 * @return     The velocity of robot's right wheel
 * @return     The angle of robot's left wheel
 * @return     The angle of robot's right wheel
 */
double Robot::GetTrackWidth() {
  return track_width;
}
double Robot::GetWheelBase() {
  return wheelbase;
}
double Robot::GetWheelSize() {
  return wheel_size;
}
double Robot::GetSpeed() {
  return vehicle_speed;
}
double Robot::GetHeading() {
  return vehicle_heading;
}
double Robot::GetLeftVel() {
  return left_wheel_velocity;
}
double Robot::GetRightVel() {
  return right_wheel_velocity;
}
double Robot::GetLeftAngle() {
  return left_wheel_angle;
}
double Robot::GetRightAngle() {
  return right_wheel_angle;
}

/**
 * @brief This function sets the velocity of left wheel.
 * @param left_wheel_vel is the velocity of left wheel.
 * @return none
 */
void Robot::SetLeftVel(double left_wheel_vel){
	left_wheel_velocity = left_wheel_vel;
}

/**
 * @brief This function sets the velocity of right wheel.
 * @param right_wheel_vel is the velocity of right wheel.
 * @return none
 */
void Robot::SetRightVel(double right_wheel_vel){
	right_wheel_velocity = right_wheel_vel;
}

/**
 * @brief This function sets the angle of left wheel.
 * @param left_angle is the velocity of left wheel.
 * @return none
 */
void Robot::SetLeftAngle(double left_angle){
	left_wheel_angle = left_angle;
}

/**
 * @brief This function sets the angle of right wheel.
 * @param right_angle is the velocity of right wheel.
 * @return none
 */
void Robot::SetRightAngle(double right_angle){
	right_wheel_angle = right_angle;
}

/**
 * @brief This function sets the speed of the vehicle.
 * @param S is the vehicle's speed.
 * @return none
 */
void Robot::SetVehicleSpeed(double S){
	vehicle_speed = S;
}

/**
 * @brief This function sets the heading (direction) of the vehicle.
 * @param H is the vehicle's heading.
 * @return none
 */
void Robot::SetVehicleHeading(double H){
	vehicle_heading = H;
}
