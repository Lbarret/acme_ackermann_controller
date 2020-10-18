#pragma once

#include <iostream>

class PID {
public:
	PID(double prop_gain, double int_gain, double diff_gain);
	double GetKp();
	double GetKi();
	double GetKd();
private:
	double kp, ki, kd;
};
