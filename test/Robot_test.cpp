#include <gtest/gtest.h>
#include "../include/Robot.hpp"

Robot robot1(1, 2, 3, 60, 70, 4, 5);

TEST(Robot_test, robot_constructor_test) {
  EXPECT_DOUBLE_EQ(1, robot1.GetTrackWidth());
  EXPECT_DOUBLE_EQ(2, robot1.GetWheelBase());
  EXPECT_DOUBLE_EQ(3, robot1.GetWheelSize());
  EXPECT_DOUBLE_EQ(60, robot1.GetLeftAngle());
  EXPECT_DOUBLE_EQ(70, robot1.GetRightAngle());
  EXPECT_DOUBLE_EQ(4, robot1.GetLeftVel());
  EXPECT_DOUBLE_EQ(5, robot1.GetRightVel());
}

TEST(Robot_test, robot_CalculateSpeed){
  robot1.CalculateVehicleSpeed();
  EXPECT_DOUBLE_EQ(0, robot1.GetSpeed());
}

TEST(Robot_test, robot_CalculateHeading){
  robot1.CalculateVehicleHeading();
  EXPECT_DOUBLE_EQ(0, robot1.GetHeading());
}
