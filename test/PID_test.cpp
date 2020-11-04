/**
 * @file       PID_test.cpp
 * @version    Iteration 2
 * @brief      The PID class is tested here.
 * @created on 3rd Nov 2020
 * @copyright  Copyright 2020. All rights reserved
 * @Author :   Loic Barret (Driver), Aditya Goswami (Navigator), Divyam Garg (Design Keeper),
 */

// included gtest header file
#include <gtest/gtest.h>
#include "../include/PID.hpp"

PID pid(0.1, 0.2, 0.3);

/**
   * @brief PID constructor test
   * @param PID_test, PID_constructor_test
   * @return none
   */
TEST(PID_test, PID_constructor_test) {
	EXPECT_DOUBLE_EQ(0.1, pid.GetKp());
	EXPECT_DOUBLE_EQ(0.2, pid.GetKi());
	EXPECT_DOUBLE_EQ(0.3, pid.GetKd());
}
