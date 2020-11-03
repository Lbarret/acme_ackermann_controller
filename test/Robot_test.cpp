/**
 * @file       Robot_test.cpp
 * @version    Iteration 2
 * @brief      The Robot class is tested here.
 * @created on 3rd Nov 2020
 * @copyright  Copyright 2020. All rights reserved
 * @Author :   Loic Barret (Driver), Aditya Goswami (Navigator), Divyam Garg (Design Keeper),
 */

// included gtest header file
#include <gtest/gtest.h>
#include "../include/Robot.hpp"

Robot robot1(1, 2, 3, 60, 70, 4, 5);

/**
   * @brief Robot Constructor test
   * @param Robot_test, Robot_constructor_test
   * @return none
   */
TEST(Robot_test, robot_constructor_test) {
  EXPECT_DOUBLE_EQ(1, robot1.GetTrackWidth());
  EXPECT_DOUBLE_EQ(2, robot1.GetWheelBase());
  EXPECT_DOUBLE_EQ(3, robot1.GetWheelSize());
  EXPECT_DOUBLE_EQ(60, robot1.GetLeftAngle());
  EXPECT_DOUBLE_EQ(70, robot1.GetRightAngle());
  EXPECT_DOUBLE_EQ(4, robot1.GetLeftVel());
  EXPECT_DOUBLE_EQ(5, robot1.GetRightVel());
}
