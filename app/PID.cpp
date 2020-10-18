#include <iostream>
#include "../include/PID.hpp"

PID::PID(double prop_gain, double int_gain, double diff_gain){
	kp = prop_gain;
	ki = int_gain;
	kd = diff_gain;
}
double PID::GetKp(){
	return kp;
}
double PID::GetKi(){
	return ki;
}
double PID::GetKd(){
	return kd;
}

