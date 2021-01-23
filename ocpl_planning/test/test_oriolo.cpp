#include <ocpl_planning/oriolo.h>

#include <iostream>
#include <string>

#include <gtest/gtest.h>

using namespace oriolo;

const double TOLERANCE = 1e-8;
void compareVectors(const std::vector<double>& v1, const std::vector<double>& v2, const std::string& info);
std::ostream& operator<<(std::ostream& os, const JointPositions& q);

class TestPlanner : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        ik_ = [](const ocpl::Transform& tf, const JointPositions& q_red) {
            JointPositions q = q_red;
            for (size_t i = 0; i < 3; ++i)
                q.push_back(0.0);
            IKSolution sol{ q };

            for (size_t i = 0; i < 3; ++i)
                q[q_red.size() + i] = 1.5;
            sol.push_back(q);

            return sol;
        };

        isValid_ = [](const JointPositions& /* q */) { return true; };

        for (int i = 0; i < 6; ++i)
        {
            joint_limits_.push_back({ -1.5, 1.5 });
            tsr_bounds_.push_back({ 0.0, 0.0 });
        }
        tsr_bounds_.back() = ocpl::Bounds{ 0.0, M_PI };
    }

    // TSR has no default constructor
    ocpl::TSR getTSR()
    {
        // create dummy TSR
        ocpl::TSRBounds bnds;
        bnds.fromVector(tsr_bounds_);
        return ocpl::TSR(ocpl::Transform::Identity(), bnds);
    }

    const size_t num_red_dof_{ 3 };
    const size_t num_dof_{ 6 };

    IKFun ik_;
    IsValidFun isValid_;

    std::vector<ocpl::Bounds> joint_limits_;
    std::vector<ocpl::Bounds> tsr_bounds_;
};

TEST_F(TestPlanner, TestInvKin)
{
    // test joint positions
    JointPositions q_red{ 1.0, -0.5, 0.2 };
    JointPositions q_bias{ 1.0, -0.5, 0.2, 0.01, 0.01, 0.01 };

    Planner p(ik_, isValid_, joint_limits_, tsr_bounds_);
    ocpl::TSR tsr = getTSR();

    compareVectors(p.invKin(tsr, q_red, q_bias), { 1.0, -0.5, 0.2, 0.0, 0.0, 0.0 }, "invKind()");

    // the ik solution is too far off from the biased solution
    JointPositions q_bias2{ 1.0, -0.5, 0.2, 0.01, 0.01, 0.01 + magic::D };
    ASSERT_TRUE(p.invKin(tsr, q_red, q_bias2).empty());

    ASSERT_TRUE(true);

    for (int j = { 0 }; j < 20; ++j)
        std::cout << p.invKin(tsr, q_red) << "\n";
}

TEST_F(TestPlanner, TestRandConf)
{
    // test joint positions
    JointPositions q_red{ 1.0, -0.5, 0.2 };
    JointPositions q_bias{ 1.0, -0.5, 0.2, 0.1, 0.1, 0.1 };
    JointPositions q_red_bias(q_bias.begin(), q_bias.begin() + num_red_dof_);

    // set deterministic ik funcion
    auto ik_fun = [](const ocpl::Transform& tf, const JointPositions& q_red) {
        JointPositions q = q_red;
        for (size_t i = 0; i < 3; ++i)
            q.push_back(0.12345);
        IKSolution sol{ q };
        return sol;
    };

    Planner p(ik_fun, isValid_, joint_limits_, tsr_bounds_);
    ocpl::TSR tsr = getTSR();

    auto q0 = p.randRed(q_red_bias);
    ASSERT_EQ(q0.size(), num_red_dof_);
    for (size_t i = 0; i < num_red_dof_; ++i)
    {
        ASSERT_LE(q0[i], q_red_bias[i] + magic::D);
        ASSERT_GE(q0[i], q_red_bias[i] - magic::D);
    }

    auto q1 = p.randConf();
    ASSERT_EQ(q1.size(), num_dof_);
    for (size_t i = 0; i < num_dof_; ++i)
    {
        ASSERT_LE(q1[i], joint_limits_[i].upper);
        ASSERT_GE(q1[i], joint_limits_[i].lower);
    }

    auto q3 = p.randConf(tsr);
    ASSERT_EQ(q3.size(), num_dof_);
    // only the base joints are set by the ik solver
    // the redundant joints are random here
    for (size_t i = 3; i < num_dof_; ++i)
    {
        ASSERT_NEAR(q3[i], 0.12345, TOLERANCE);
    }

    auto q4 = p.randConf(tsr, q_bias);
    ASSERT_EQ(q4.size(), num_dof_);
    // joint should not deviate too much from bias
    for (size_t i = 0; i < num_dof_; ++i)
    {
        ASSERT_LE(q4[i], q_bias[i] + magic::D);
        ASSERT_GE(q4[i], q_bias[i] - magic::D);
    }
    // base joints are deterministic
    for (size_t i = 3; i < num_dof_; ++i)
    {
        ASSERT_NEAR(q4[i], 0.12345, TOLERANCE);
    }
}

TEST(TestCollision, TestCollision1D)
{
    auto ik = [](const ocpl::Transform& tf, const JointPositions& q_red) {
        JointPositions q{ 0.1234 };
        IKSolution sol{ q };
        return sol;
    };

    auto is_valid = [](const JointPositions& q) {
        if (q.at(0) < 0.5 || q.at(0) > 0.7)
            return true;
        else
            return false;
    };

    std::vector<ocpl::Bounds> jl{ { 0.0, 0.1 } };
    std::vector<ocpl::Bounds> bnds{ { 0.0, 0.0 } };
    Planner p(ik, is_valid, jl, bnds);

    ASSERT_TRUE(p.noColl({ 0.2 }));
    ASSERT_FALSE(p.noColl({ 0.6 }));
    ASSERT_TRUE(p.noColl({ 0.8 }));

    // the complete path is valid
    ASSERT_TRUE(p.noColl({ 0.0 }, { 0.4 }));
    // interpolated state should be 0.6, which is not valid
    ASSERT_FALSE(p.noColl({ 0.4 }, { 0.8 }));
    // interpolated state should be 1.0, and therefore interpolation should not catch path violation
    ASSERT_TRUE(p.noColl({ 0.4 }, { 1.6 }));
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

void compareVectors(const std::vector<double>& v1, const std::vector<double>& v2, const std::string& info)
{
    ASSERT_EQ(v1.size(), v2.size()) << info;
    for (size_t i{ 0 }; i < v1.size(); ++i)
        ASSERT_NEAR(v1[i], v2[i], TOLERANCE) << info;
}

std::ostream& operator<<(std::ostream& os, const JointPositions& q)
{
    for (auto qi : q)
        os << qi << ", ";
    return os;
}
