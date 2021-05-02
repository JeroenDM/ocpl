#include <ocpl_planning/planner_base.h>

#include <iostream>

#include <gtest/gtest.h>

TEST(Testbasics, Test1)
{
    ASSERT_TRUE(true);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
