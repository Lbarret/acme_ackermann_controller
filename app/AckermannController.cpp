#include <iostream>
#include <cmath>
#include "../include/AckermannController.hpp"
#include "../include/Robot.hpp"
#include "../include/PID.hpp"

AckermannController::AckermannController(){

}

AckermannController::AckermannController(Robot robot, PID vel_PID, PID heading_PID) {
	car = robot;
	velocity_control = vel_PID;
	heading_control = heading_PID;
	desired_speed = 0;
	desired_heading = 0;
	timestamp = .01;
}
void AckermannController::SetDesiredSpeed(double speed){
desired_speed = speed;
}
void AckermannController::SetDesiredHeading(double heading){
desired_heading = heading;
}

double AckermannController::GetDesiredSpeed(){
	return desired_speed;
}

double AckermannController::GetDesiredHeading(){
	return desired_heading;
}

void AckermannController::CalculateVehicleSpeed(){
	car.SetVehicleSpeed((car.GetLeftVel()+car.GetRightVel())*car.GetWheelSize()*M_PI/2);
}

void AckermannController::CalculateVehicleHeading(){

	car.SetVehicleHeading(car.GetHeading() + car.GetSpeed()/car.GetWheelBase()*(2*car.GetWheelBase()+car.GetTrackWidth() * atan(car.GetLeftAngle()*M_PI/180))/(2*car.GetWheelBase()*atan(car.GetLeftAngle()*M_PI/180))*timestamp);
	/*if(desired_heading > 0){
		car.SetVehicleHeading(-car.GetHeading());
	}*/

}

void AckermannController::CalculateWheelVelocities(double req_speed){
	double vehicle_angular_vel;
	car.SetVehicleSpeed(req_speed);
	vehicle_angular_vel = (req_speed/car.GetWheelBase())*tan(car.GetHeading()*M_PI/180);
	double a = sin(car.GetLeftAngle()*M_PI/180);
	double b = sin(car.GetRightAngle()*M_PI/180);
	if(desired_heading < 0){
		car.SetLeftVel(vehicle_angular_vel * (car.GetWheelBase()/tan(car.GetHeading()*M_PI/180) - car.GetTrackWidth()/2)/(asin(a))*180/M_PI);
		car.SetRightVel(vehicle_angular_vel * (car.GetWheelBase()/tan(car.GetHeading()*M_PI/180) + car.GetTrackWidth()/2)/(asin(b))*180/M_PI);

	} else{
		car.SetLeftVel(vehicle_angular_vel * (car.GetWheelBase()/tan(car.GetHeading()*M_PI/180) + car.GetTrackWidth()/2)/(asin(a))*180/M_PI);
		car.SetRightVel(vehicle_angular_vel * (car.GetWheelBase()/tan(car.GetHeading()*M_PI/180) - car.GetTrackWidth()/2)/(asin(b))*180/M_PI);
	}
}

void AckermannController::CalculateWheelAngles(double req_heading){
	car.SetVehicleHeading(req_heading);
	if(req_heading>45){ req_heading = 45; }
	double inner = atan(2*car.GetWheelBase()*sin(req_heading*M_PI/180)/(2*car.GetWheelBase()*cos(req_heading*M_PI/180)-car.GetTrackWidth()*sin(req_heading*M_PI/180)));
	inner = inner*180/M_PI;
	double outer = atan(2*car.GetWheelBase()*sin(req_heading*M_PI/180)/(2*car.GetWheelBase()*cos(req_heading*M_PI/180)+car.GetTrackWidth()*sin(req_heading*M_PI/180)));
	outer = outer*180/M_PI;
	if(desired_heading > 0){
		car.SetLeftAngle(outer);
		car.SetRightAngle(inner);
	} else {
		car.SetLeftAngle(inner);
		car.SetRightAngle(outer);
	}

}

void AckermannController::Solve(){
	double prev_error_vel = 0;
	double current_error_vel = 0;
	double prev_error_heading = 0;
	double current_error_heading = 0;
	double req_heading = 0;
	double req_vel = 0;
	bool flag = true;
	while(flag){

		prev_error_heading = current_error_heading;
		current_error_heading = desired_heading - car.GetHeading();
		req_heading = car.GetHeading() + heading_control.GetKp()*current_error_heading + heading_control.GetKd()*(current_error_heading-prev_error_heading);
		CalculateWheelAngles(req_heading);

		prev_error_vel = current_error_vel;
		current_error_vel = desired_speed - car.GetSpeed();
		req_vel = car.GetSpeed() + velocity_control.GetKp()*current_error_vel + heading_control.GetKd()*(current_error_vel-prev_error_vel);
		CalculateWheelVelocities(req_vel);

		CalculateVehicleSpeed();
		if (car.GetSpeed() > desired_speed){
			car.SetVehicleSpeed(desired_speed);
		}
		CalculateVehicleHeading();
		std::cout << "Heading: " << car.GetHeading() << std::endl;
		std::cout << "Speed: " << car.GetSpeed() << std::endl;

		if (desired_heading - car.GetHeading() < 5 ){
			flag = false;
		}
	}
}
