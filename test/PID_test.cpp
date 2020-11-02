#include <gtest/gtest.h>
#include "../include/PID.hpp"

PID pid(0.1, 0.2, 0.3);

TEST(PID_test, PID_constructor_test) {
	EXPECT_DOUBLE_EQ(0.1, pid.GetKp());
	EXPECT_DOUBLE_EQ(0.2, pid.GetKi());
	EXPECT_DOUBLE_EQ(0.3, pid.GetKd());
}
