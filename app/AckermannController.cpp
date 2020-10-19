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

void AckermannController::CalculateWheelVelocities(double req_speed){
	double vehicle_angular_vel, inner, outer;
	vehicle_angular_vel = req_speed/car.GetWheelBase()*tan(car.GetHeading()*M_PI/180);
	if(desired_heading < 0){
		car.SetLeftVel(vehicle_angular_vel * (car.GetWheelBase()/tan(car.GetHeading()*M_PI/180) - car.GetTrackWidth()/2)/(asin(car.GetLeftAngle()*M_PI/180))*180/M_PI);
		car.SetRightVel(vehicle_angular_vel * (car.GetWheelBase()/tan(car.GetHeading()*M_PI/180) + car.GetTrackWidth()/2)/(asin(car.GetRightAngle()*M_PI/180))*180/M_PI);

	} else{
		car.SetLeftVel(vehicle_angular_vel * (car.GetWheelBase()/tan(car.GetHeading()*M_PI/180) + car.GetTrackWidth()/2)/(asin(car.GetLeftAngle()*M_PI/180))*180/M_PI);
		car.SetRightVel(vehicle_angular_vel * (car.GetWheelBase()/tan(car.GetHeading()*M_PI/180) - car.GetTrackWidth()/2)/(asin(car.GetRightAngle()*M_PI/180))*180/M_PI);
	}
}

void AckermannController::CalculateWheelAngles(double req_heading){
	if(req_heading>45){ req_heading = 45 }
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
	double current_error_vel;


}

