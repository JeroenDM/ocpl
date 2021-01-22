#include <ocpl_planning/oriolo.h>

#include <iostream>

#include <gtest/gtest.h>

using namespace oriolo;

const double TOLERANCE = 1e-8;
void compareVectors(const std::vector<double>& v1, const std::vector<double>& v2);
std::ostream& operator<<(std::ostream& os, const JointPositions& q);

TEST(TestRandConf, TestIk)
{
    // create dummy ik function
    auto ik_fun = [](const ocpl::Transform& tf, const JointPositions& q_red) {
        JointPositions q = q_red;
        for (size_t i = 0; i < 3; ++i)
            q.push_back(0.0);
        IKSolution sol{ q };

        for (size_t i = 0; i < 3; ++i)
            q[q_red.size() + i] = 1.5;
        sol.push_back(q);

        return sol;
    };

    // create dummy TSR
    ocpl::TSRBounds bnds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, M_PI } };
    ocpl::TSR tsr(ocpl::Transform::Identity(), bnds);

    // test joint positions
    JointPositions q_red{ 1.0, -0.5, 0.2 };
    JointPositions q_bias{ 1.0, -0.5, 0.2, 0.01, 0.01, 0.01 };

    std::vector<ocpl::Bounds> tsr_bnds{ { 0.0, 0.0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, M_PI } };
    std::vector<ocpl::Bounds> jls{ { -1.5, 1.5 }, { -1.5, 1.5 }, { -1.5, 1.5 },
                                   { -1.5, 1.5 }, { -1.5, 1.5 }, { -1.5, 1.5 } };

    Planner p(ik_fun, jls, tsr_bnds);

    compareVectors(p.invKin(tsr, q_red, q_bias), { 1.0, -0.5, 0.2, 0.0, 0.0, 0.0 });

    // the ik solution is too far off from the biased solution
    JointPositions q_bias2{ 1.0, -0.5, 0.2, 0.01, 0.01, 0.01 + magic::D };
    ASSERT_TRUE(p.invKin(tsr, q_red, q_bias2).empty());

    ASSERT_TRUE(true);

    for (int j={0}; j < 20; ++j)
        std::cout << p.invKin(tsr, q_red) << "\n";
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

void compareVectors(const std::vector<double>& v1, const std::vector<double>& v2)
{
    ASSERT_EQ(v1.size(), v2.size());
    for (size_t i{ 0 }; i < v1.size(); ++i)
        ASSERT_NEAR(v1[i], v2[i], TOLERANCE);
}

std::ostream& operator<<(std::ostream& os, const JointPositions& q)
{
    for (auto qi : q)
        os << qi << ", ";
    return os;
}
