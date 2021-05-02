#include <ocpl_planning/oriolo.h>

#include <iostream>
#include <string>

#include <memory>

#include <gtest/gtest.h>

using namespace ocpl;
using namespace ocpl::oriolo;

const double TOLERANCE = 1e-8;
void compareVectors(const std::vector<double>& v1, const std::vector<double>& v2, const std::string& info);
std::ostream& operator<<(std::ostream& os, const JointPositions& q);

class TestPlanner : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        fk_ = [](const JointPositions& /* q */) { return ocpl::Transform::Identity(); };
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
        tsr_bounds_.back() = Bounds{ 0.0, M_PI };
    }

    // TSR has no default constructor
    TSR getTSR()
    {
        // create dummy TSR
        TSRBounds bounds;
        bounds.fromVector(tsr_bounds_);
        return TSR(Transform::Identity(), bounds);
    }

    const size_t num_red_dof_{ 3 };
    const size_t num_dof_{ 6 };

    FKFun fk_;
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

    Robot bot{num_dof_, num_red_dof_, joint_limits_, fk_, ik_, isValid_};
    PlannerSettings set;
    OrioloPlanner p(bot, set);
    TSR tsr = getTSR();
    p.initializeTaskSpaceSamplers(tsr.bounds.asVector());

    compareVectors(p.invKin(tsr, q_red, q_bias), { 1.0, -0.5, 0.2, 0.0, 0.0, 0.0 }, "invKind()");

    // the ik solution is too far off from the biased solution
    JointPositions q_bias2{ 1.0, -0.5, 0.2, 0.01, 0.01, 0.01 + set.cspace_delta };
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

    Robot bot{num_dof_, num_red_dof_, joint_limits_, fk_, ik_fun, isValid_};
    PlannerSettings set;
    OrioloPlanner p("oriolo", bot, set);
    ocpl::TSR tsr = getTSR();
    p.initializeTaskSpaceSamplers(tsr.bounds.asVector());
 
    auto q0 = p.randRed(q_red_bias);
    ASSERT_EQ(q0.size(), num_red_dof_);
    for (size_t i = 0; i < num_red_dof_; ++i)
    {
        ASSERT_LE(q0[i], q_red_bias[i] + set.cspace_delta);
        ASSERT_GE(q0[i], q_red_bias[i] - set.cspace_delta);
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
        ASSERT_NEAR(q4[i], q_bias[i], set.cspace_delta);
    }
    // base joints are deterministic
    for (size_t i = 3; i < num_dof_; ++i)
    {
        ASSERT_NEAR(q4[i], 0.12345, TOLERANCE);
    }
}

class TestRobot1D : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        fk_ = [](const JointPositions& q) {
            auto tf = ocpl::Transform::Identity();
            tf.translation().x() = q.at(0);
            return tf;
        };
        ik_ = [](const ocpl::Transform& tf, const JointPositions& /* q_red */) {
            // the x position of the robot will be samples between 0 and 1
            // this is also the joint value for this simple robot
            JointPositions q{ tf.translation().x() };
            IKSolution sol{ q };
            return sol;
        };

        isValid_ = [](const JointPositions& q) {
            if (q.at(0) < 0.5 || q.at(0) > 0.7)
                return true;
            else
                return false;
        };

        jl_.push_back({ 0.0, 1.0 });
        bnds_.push_back({ 0.0, 1.0 });

        for (int i = 0; i < 6; ++i)
            bnds_6d_.push_back({ 0.0, 0.0 });
        bnds_6d_[0] = ocpl::Bounds{ 0.0, 0.02 };
    }

    FKFun fk_;
    IKFun ik_;
    IsValidFun isValid_;
    std::vector<ocpl::Bounds> jl_;
    std::vector<ocpl::Bounds> bnds_;
    std::vector<ocpl::Bounds> bnds_6d_;
};

TEST_F(TestRobot1D, TestCollision1D)
{
    Robot bot{1, 0, jl_, fk_, ik_, isValid_};
    OrioloSpecificSettings set;
    OrioloPlanner p("oriolo", bot, set);
    p.initializeTaskSpaceSamplers(bnds_);

    // the test robot position is valid outside the interval [0.5, 0.7]
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

TEST_F(TestRobot1D, TestStep1D)
{
    Robot bot{1, 0, jl_, fk_, ik_, isValid_};
    OrioloSpecificSettings set;
    OrioloPlanner p("oriolo", bot, set);
    p.initializeTaskSpaceSamplers(bnds_6d_);

    // create the task
    std::vector<double> x_positions{ 0.0, 0.01, 0.02 };
    std::vector<ocpl::TSR> task;
    for (double xi : x_positions)
    {
        ocpl::TSRBounds b;
        b.fromVector(bnds_6d_);
        ocpl::Transform tfi = ocpl::Transform::Identity();
        tfi.translation().x() = xi;
        task.push_back(ocpl::TSR(tfi, b));
    }

    auto path = p.step(0, 2, { 0.0 }, task);
    ASSERT_EQ(path.size(), task.size());
    // for (size_t i = 0; i < path.size(); ++i)
    //     ASSERT_NEAR(path[i][0], x_positions[i], magic::D) << "point: " << i;
}

TEST_F(TestRobot1D, TestGreedy1D)
{
    Robot bot{1, 0, jl_, fk_, ik_, isValid_};
    OrioloSpecificSettings set;
    OrioloPlanner p("oriolo", bot, set);
    p.initializeTaskSpaceSamplers(bnds_6d_);

    // create the task
    std::vector<double> x_positions{ 0.0, 0.01, 0.02 };
    std::vector<ocpl::TSR> task;
    for (double xi : x_positions)
    {
        ocpl::TSRBounds b;
        b.fromVector(bnds_6d_);
        ocpl::Transform tfi = ocpl::Transform::Identity();
        tfi.translation().x() = xi;
        task.push_back(ocpl::TSR(tfi, b));
    }

    auto path = p.greedy(task);
    ASSERT_EQ(path.size(), task.size());
    // for (size_t i = 0; i < path.size(); ++i)
    //     ASSERT_NEAR(path[i][0], x_positions[i], magic::D) << "point: " << i;
}

TEST_F(TestRobot1D, TestExtend)
{
    Robot bot{1, 0, jl_, fk_, ik_, isValid_};
    OrioloSpecificSettings set;
    OrioloPlanner p("oriolo", bot, set);
    p.initializeTaskSpaceSamplers(bnds_6d_);

    JointPositions q0{ 0.0 };
    JointPositions q1{ 2.0 };
    auto n0 = std::make_shared<graph::Node>(graph::NodeData{ q0, 0 });
    auto n1 = std::make_shared<graph::Node>(graph::NodeData{ q1, 1 });

    graph::Tree tree;
    tree[n0] = {};
    tree[n1] = {};

    auto res0 = p.getNear({ 0.01 }, tree)->data;
    ASSERT_EQ(res0.q.at(0), q0.at(0));
    ASSERT_EQ(res0.waypoint, 0);

    auto res1 = p.getNear({ 1.8 }, tree)->data;
    ASSERT_EQ(res1.q.at(0), q1.at(0));
    ASSERT_EQ(res1.waypoint, 1);

    // create the task
    std::vector<double> x_positions{ 0.0, 0.01, 0.02 };
    std::vector<ocpl::TSR> task;
    for (double xi : x_positions)
    {
        ocpl::TSRBounds b;
        b.fromVector(bnds_6d_);
        ocpl::Transform tfi = ocpl::Transform::Identity();
        tfi.translation().x() = xi;
        task.push_back(ocpl::TSR(tfi, b));
    }

    auto res = p.extend(task, tree);
    std::cout << "Extend: " << res.first << ", " << res.second.waypoint << "\n";

    auto path = p.rrtLike(task);
    std::cout << "rrtLike: " << path.size() << "\n";
    std::cout << "path: ";
    for (auto q : path)
        std::cout << q[0] << ", ";
    std::cout << "\n";
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
