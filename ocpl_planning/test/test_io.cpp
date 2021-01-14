#include <ocpl_planning/io.h>

#include <gtest/gtest.h>

TEST(TestFileParser, TestParseVariable)
{

    ocpl::parsePose("0.18 0 0.02 0 135 90");
    ocpl::parsePose(" 0.18 0 0.02 0 135 90");
    ocpl::parsePose("0.18 0 0.02 0 135 90   ");

    EXPECT_TRUE(true);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
