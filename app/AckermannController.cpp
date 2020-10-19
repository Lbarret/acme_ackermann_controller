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
}
void AckermannController::SetDesiredSpeed(double speed){

}
void AckermannController::SetDesiredHeading(double heading){

}
void AckermannController::Solve(){

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
	if(desired_heading > 0){
		car.SetVehicleHeading(-car.GetHeading());
	}

}
