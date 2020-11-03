/**
 * @file       ackermann_controller.cpp
 * @version    1.0
 * @brief      This file is the main function which instantiate the object and implemets the function.initialising robot class and pid class: pid_speed, pid_heading
 * @created on 20th Oct 2020
 * @copyright  Copyright 2020. All rights reserved
 * @Author :   Divyam Garg (Driver), Loic Barret (Navigator), Aditya Goswami (Design Keeper),
 */

// user defined header files for ackermann controller, robot state and pid control.
#include "../include/AckermannController.hpp"
#include "../include/Robot.hpp"
#include "../include/PID.hpp"
#include <cmath>
#include <ratio>
#include <chrono>
#include <unistd.h>
#include "../include/supportLib.hpp"
#include "../include/pbPlots.hpp"
#include <vector>


// c++ header file
#include <iostream>

/**
 * @brief AckermannController default constructor
 * @param none
 * @return none
 */
AckermannController::AckermannController() { }

/**
 * @brief    AckermannController parameterized constructor sets values of six variable defined in it
 * @param[1] robot object from Robot class
 * @param[2] vel_PID object from PID class
 * @param[3] heading object from PID class
 * @return   none
 */
AckermannController::AckermannController(Robot robot, PID vel_PID, PID heading_PID) {
	car = robot;
	velocity_control = vel_PID;
	heading_control = heading_PID;
	desired_speed = 0;
	desired_heading = 0;
	timestamp = .01;
}

/**
 * @brief      Next two functions return values of desired_speed and desired_heading
 * @param      none
 * @return     Desired Speed
 * @return     Desired Heading
 */
double AckermannController::GetDesiredSpeed() {
	return desired_speed;
}
//getter
double AckermannController::GetDesiredHeading() {
	return desired_heading;
}

/**
 * @brief This function sets the desired speed.
 * @param speed of vehicle
 * @return none
 */
void AckermannController::SetDesiredSpeed(double speed) {
desired_speed = speed;
}

/**
 * @brief This function sets the desired heading.
 * @param heading of vehicle
 * @return none
 */
void AckermannController::SetDesiredHeading(double heading) {
desired_heading = heading;
}

/**
 * @brief This function calculates the vehicle speed.
 * @param none
 * @return none
 */
void AckermannController::CalculateVehicleSpeed() {
	car.SetVehicleSpeed((car.GetLeftVel()+car.GetRightVel())/2);
}

/**
 * @brief This function calculates the vehicle heading.
 * @param none
 * @return none
 */
void AckermannController::CalculateVehicleHeading() {

	car.SetVehicleHeading(car.GetHeading() + car.GetSpeed()/car.GetWheelBase()*(2*car.GetWheelBase()+car.GetTrackWidth() * atan(car.GetLeftAngle()*M_PI/180))/(2*car.GetWheelBase()*atan(car.GetLeftAngle()*M_PI/180))*timestamp);
	/*if(desired_heading > 0){
		car.SetVehicleHeading(-car.GetHeading());
	}*/
}

/**
 * @brief This function calculates the velocities of each wheel.
 * @param Required Speed
 * @return none
 */
void AckermannController::CalculateWheelVelocities(double req_speed) {
	double vehicle_angular_vel;
	//std::cout << "req_speed: " << req_speed << std::endl;
	//car.SetVehicleSpeed(req_speed);
	double r = (car.GetWheelBase()/sin(car.GetHeading()*M_PI/180));
	vehicle_angular_vel = req_speed/r;
	//std::cout << "vehicle_angular_vel: " << vehicle_angular_vel << std::endl;
	if(car.GetHeading() == 0){

	  car.SetLeftVel(req_speed);
	  car.SetRightVel(req_speed);


	} else if(desired_heading < 0){

		car.SetLeftVel(vehicle_angular_vel * (car.GetWheelBase())/(sin(car.GetLeftAngle()*M_PI/180)));
		car.SetRightVel(vehicle_angular_vel * (car.GetWheelBase())/(sin(car.GetRightAngle()*M_PI/180)));
    
	}else{
		car.SetLeftVel(vehicle_angular_vel * (car.GetWheelBase())/(sin(car.GetLeftAngle()*M_PI/180)));
		car.SetRightVel(vehicle_angular_vel * (car.GetWheelBase())/(sin(car.GetRightAngle()*M_PI/180)));
	}
	//std::cout << "left " << car.GetLeftVel() << " right " << car.GetRightVel() << std::endl; 
}

/**
 * @brief This function calculates the angles of each wheel.
 * @param Required heading (direction)
 * @return none
 */
void AckermannController::CalculateWheelAngles(double req_heading) {
	//car.SetVehicleHeading(req_heading);
	//std::cout << "req_heading " << req_heading << std::endl; 
	

	double inner = atan2(2*car.GetWheelBase()*sin(req_heading*M_PI/180), 2*car.GetWheelBase()*cos(req_heading*M_PI/180)-car.GetTrackWidth()*sin(req_heading*M_PI/180));
	inner = inner*180/M_PI;
	if (inner > 45){
		inner = 45;
	}
	//std::cout << "inner = " << inner << std::endl;
	double outer = atan2(2*car.GetWheelBase()*sin(req_heading*M_PI/180),2*car.GetWheelBase()*cos(req_heading*M_PI/180)+car.GetTrackWidth()*sin(req_heading*M_PI/180));
	outer = outer*180/M_PI;
	if (outer > 45){
		outer = 45;
	}
	//std::cout << "outer = " << outer << std::endl;

	if (desired_heading > 0) {
		car.SetLeftAngle(outer);
		car.SetRightAngle(inner);
	} else {
		car.SetLeftAngle(inner);
		car.SetRightAngle(outer);
	}
}

/**
 * @brief This function plots the outputs.
 * @param x, y, name
 * @return none
 */
void AckermannController::plot(std::vector<double> x, std::vector<double> y, std::string name){
	RGBABitmapImageReference *imageReference = CreateRGBABitmapImageReference();

	DrawScatterPlot(imageReference, 600, 400, &x, &y);
    
    vector<double> *pngdata = ConvertToPNG(imageReference->image);
    WriteToFile(pngdata, name);
    DeleteImage(imageReference->image);
}

/**
 * @brief This function helps in converging the speed and angles to the desired outputs.
 * @param none
 * @return none
 */
void AckermannController::Solve() {
	double prev_error_vel = 0;
	double current_error_vel = 0;
	double prev_error_heading = 0;
	double current_error_heading = 0;
	double req_heading = 0;
	double req_vel = 0;
	bool flag = true;
	int iteration = 0;
	std::vector<double> heading_y;
	std::vector<double> speed_y;
	std::vector<double> time_x;
	
    std::chrono::high_resolution_clock::time_point beginning = std::chrono::high_resolution_clock::now();
	
	while (flag) {
        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
		prev_error_heading = current_error_heading;
		current_error_heading = desired_heading - car.GetHeading();
		req_heading = car.GetHeading() + heading_control.GetKp()*current_error_heading + heading_control.GetKd()*(current_error_heading-prev_error_heading);
		CalculateWheelAngles(req_heading);

		prev_error_vel = current_error_vel;
		current_error_vel = desired_speed - car.GetSpeed();
		//std::cout << "speed1: " << car.GetSpeed() << std::endl;
		req_vel = car.GetSpeed() + velocity_control.GetKp()*current_error_vel + velocity_control.GetKd()*(current_error_vel-prev_error_vel);
		CalculateWheelVelocities(req_vel);

		CalculateVehicleSpeed();
		if (car.GetSpeed() > desired_speed) {
			car.SetVehicleSpeed(desired_speed);
		}
		CalculateVehicleHeading();
		
		//std::cout << "Desired Heading: " << desired_heading << std::endl;
		std::cout << "Heading: "<< car.GetHeading() << std::endl;
		std::cout << "Speed: "<< car.GetSpeed() << std::endl;

		if (std::abs(desired_heading - car.GetHeading()) < 1 && std::abs(desired_speed - car.GetSpeed()) < 1) {
			flag = false;
		}
		std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
		//std::cout << time_span.count() << std::endl;
		usleep(10000 - time_span.count()*1000000);
		std::chrono::high_resolution_clock::time_point total = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> time_span_total = std::chrono::duration_cast<std::chrono::duration<double>>(total - beginning);
		std::cout<< "Time = " << time_span_total.count() << std::endl << std::endl;
		iteration++;
		//std::cout << iteration << endl;

		if (iteration % 10 == 0) {
			time_x.push_back(time_span_total.count());
			heading_y.push_back(car.GetHeading());
			speed_y.push_back(car.GetSpeed());
		}

	}
	std::string heading_plot = "heading_plot.png";
	std::string speed_plot = "speed_plot.png";

	plot(time_x, heading_y, heading_plot);
	plot(time_x, speed_y, speed_plot);

}

