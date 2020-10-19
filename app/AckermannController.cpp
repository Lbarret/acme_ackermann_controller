#include <iostream>
#include "../include/AckermannController.hpp"
#include "../include/Robot.hpp"
#include "../include/PID.hpp"

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
