#include <ocpl_planning/io.h>

#include <string>
#include <gtest/gtest.h>

const std::string L_PROFILE {R"irl(
variables
config home 0 -1.5708 1.5708 0 0 0
pose work 0.8 -0.5 0 0 0 0
set-reference work
pose P1 0.18 0 0.02 0 135 90
pose P2 0.18 1 0.02 0 135 90
pose Papp 0.16 0 0.04 0 135 90
pose Pret 0.16 1 0.04 0 135 90
set-reference default

commands
movej home
movep Papp constraints C1
movel P1 constraints C1 approach 0 0 0.1
movel P2 constraints C1 retract 0 0 -0.1
movel Pret constraints C1
movej home

constraints
rpy C1 0 0 3.1415 0 0 3.1415
)irl"};


TEST(TestFileParser, TestParseVariable)
{

    ocpl::parsePose("0.18 0 0.02 0 135 90");
    ocpl::parsePose(" 0.18 0 0.02 0 135 90");
    ocpl::parsePose("0.18 0 0.02 0 135 90   ");

    EXPECT_TRUE(true);
}

TEST(TestCompleteFile, LProfile)
{
    ocpl::parseIrlTask(L_PROFILE);

    EXPECT_TRUE(true);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
