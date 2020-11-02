#include <gtest/gtest.h>
#include "../include/AckermannController.hpp"
#include "../include/PID.hpp"
#include "../include/Robot.hpp"

PID vel_pid(0.1, 0.2, 0.3);
PID head_pid(0.4, 0, 0.6);
Robot robot2(1, 2, 3, 60, 70, 4, 5);
AckermannController control(robot2, vel_pid, head_pid);

TEST(AckermannController_test, AckermannController_constructor_test) {
	EXPECT_DOUBLE_EQ(1, control.car.GetTrackWidth());
	EXPECT_DOUBLE_EQ(0.3, control.velocity_control.GetKd());
	EXPECT_DOUBLE_EQ(0.6, control.heading_control.GetKd());
}

TEST(AckermannController_test, AckermannController_GetDesiredSpeed_test){

	EXPECT_DOUBLE_EQ(0, control.GetDesiredSpeed());
}

TEST(AckermannController_test, AckermannController_SetDesiredSpeed_test){
	control.SetDesiredSpeed(3);
	EXPECT_DOUBLE_EQ(3, control.GetDesiredSpeed());
}

TEST(AckermannController_test, AckermannController_GetDesiredHeading_test){

	EXPECT_DOUBLE_EQ(0, control.GetDesiredHeading());
}

TEST(AckermannController_test, AckermannController_SetDesiredHeading_test){
	control.SetDesiredHeading(40);
	EXPECT_DOUBLE_EQ(40, control.GetDesiredHeading());
}

TEST(AckermannController_test, AckermannController_Solve_test){
	control.Solve();
	EXPECT_NEAR(control.GetDesiredHeading(), control.car.GetHeading(), 5.0);
	EXPECT_NEAR(control.GetDesiredSpeed(), control.car.GetSpeed(), 5.0);
}
