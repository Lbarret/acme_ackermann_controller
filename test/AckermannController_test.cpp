/**
 * @file       AckermannController_test.cpp
 * @version    Iteration 2
 * @brief      The AckermannController class is tested here.
 * @created on 3rd Nov 2020
 * @copyright  Copyright 2020. All rights reserved
 * @Author :   Loic Barret (Driver), Aditya Goswami (Navigator), Divyam Garg (Design Keeper),
 */
#include <gtest/gtest.h>
#include "../include/AckermannController.hpp"
#include "../include/PID.hpp"
#include "../include/Robot.hpp"


PID vel_pid(0.25, 0, 0.1);
PID head_pid(0.25, 0, 0.1);
Robot robo(0.3, 1, 0.15, 0, 0, 0, 0);
AckermannController control(robo, vel_pid, head_pid);

/**
   * @brief AckermannController constructor test
   * @param AckermannController_test, AckermannController_constructor_test
   * @return none
   */
TEST(AckermannController_test, AckermannController_constructor_test) {
	EXPECT_DOUBLE_EQ(.3, control.car.GetTrackWidth());
	EXPECT_DOUBLE_EQ(0.1, control.velocity_control.GetKd());
	EXPECT_DOUBLE_EQ(0.1, control.heading_control.GetKd());
}

/**
   * @brief AckermannController getDesiredSpeed function test
   * @param AckermannController_test, AckermannController_GetDesiredSpeed_test
   * @return none
   */
TEST(AckermannController_test, AckermannController_GetDesiredSpeed_test){

	EXPECT_DOUBLE_EQ(0, control.GetDesiredSpeed());
}

/**
   * @brief AckermannController setDesiredSpeed function test
   * @param AckermannController_test, AckermannController_SetDesiredSpeed_test
   * @return none
   */
TEST(AckermannController_test, AckermannController_SetDesiredSpeed_test){
	control.SetDesiredSpeed(3);
	EXPECT_DOUBLE_EQ(3, control.GetDesiredSpeed());
}

/**
   * @brief AckermannController getDesiredHeading function test
   * @param AckermannController_test, AckermannController_GetDesiredHeading_test
   * @return none
   */
TEST(AckermannController_test, AckermannController_GetDesiredHeading_test){

	EXPECT_DOUBLE_EQ(0, control.GetDesiredHeading());
}

/**
   * @brief AckermannController setDesiredHeading function test
   * @param AckermannController_test, AckermannController_SetDesiredHeading_test
   * @return none
   */
TEST(AckermannController_test, AckermannController_SetDesiredHeading_test){
	control.SetDesiredHeading(40);
	EXPECT_DOUBLE_EQ(40, control.GetDesiredHeading());
}

/**
   * @brief AckermannController Solver function test
   * @param AckermannController_test, AckermannController_Solve_test
   * @return none
   */
TEST(AckermannController_test, AckermannController_Solve_test){
	control.SetDesiredHeading(80);
	control.SetDesiredSpeed(20);
	control.Solve();
	EXPECT_NEAR(control.GetDesiredHeading(), control.car.GetHeading(), 2.0);
	EXPECT_NEAR(control.GetDesiredSpeed(), control.car.GetSpeed(), 2.0);
}
