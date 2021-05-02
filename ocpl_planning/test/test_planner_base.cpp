#include <ocpl_planning/planner_base.h>
#include <ocpl_planning/acro_planner.h>
#include <ocpl_planning/io.h>

#include <iostream>

#include <gtest/gtest.h>

const double TOLERANCE = 1e-8;
void compareVectors(const std::vector<double>& v1, const std::vector<double>& v2, const std::string& info);

using namespace ocpl;

class TestRobot2D : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        fk_ = [](const JointPositions& q) {
            auto tf = ocpl::Transform::Identity();
            tf.translation().x() = q.at(0);
            tf.translation().y() = q.at(1);
            return tf;
        };
        ik_ = [](const ocpl::Transform& tf, const JointPositions& /* q_red */) {
            JointPositions q{ tf.translation().x(), tf.translation().y() };
            IKSolution sol{ q };
            return sol;
        };

        isValid_ = [](const JointPositions& q) {
            if (q.at(1) < 0.0|| q.at(1) > 0.2)
                return true;
            else
                return false;
        };

        jl_.push_back({ 0.0, 1.0 });
        jl_.push_back({ 0.0, 1.0 });

        for (int i = 0; i < 6; ++i)
            bnds_6d_.push_back({ 0.0, 0.0 });
        bnds_6d_[0] = ocpl::Bounds{ 0.0, 0.02 };
        bnds_6d_[1] = ocpl::Bounds{ -0.2, 0.3 };
    }

    FKFun fk_;
    IKFun ik_;
    IsValidFun isValid_;
    std::vector<ocpl::Bounds> jl_;
    std::vector<ocpl::Bounds> bnds_6d_;
};

TEST_F(TestRobot2D, TestUnifiedPlannerGlobal)
{
    // create the task
    std::vector<double> x_positions{ 0.0, 0.01, 0.02 };
    std::vector<ocpl::TSR> task;
    for (double xi : x_positions)
    {
        ocpl::TSRBounds b;
        b.fromVector(bnds_6d_);
        ocpl::Transform tfi = ocpl::Transform::Identity();
        tfi.translation().x() = xi;
        tfi.translation().y() = -0.2;
        task.push_back(ocpl::TSR(tfi, b));
    }

    PlannerSettings s;
    s.type = PlannerType::GLOBAL;
    s.sampler_type = SamplerType::HALTON;
    s.is_redundant = false;
    s.c_space_batch_size = 10;
    s.t_space_batch_size = 10;
    s.debug = true;
    // std::cout << s << "\n";

    Robot bot {2, 0, jl_, fk_, ik_, isValid_};

    UnifiedPlanner planner(bot, s);
    Solution sol = planner.solve(task);

    EXPECT_TRUE(sol.success);

}

TEST_F(TestRobot2D, TestUnifiedPlannerLocal)
{
    // create the task
    std::vector<double> x_positions{ 0.0, 0.01, 0.02 };
    std::vector<ocpl::TSR> task;
    for (double xi : x_positions)
    {
        ocpl::TSRBounds b;
        b.fromVector(bnds_6d_);
        ocpl::Transform tfi = ocpl::Transform::Identity();
        tfi.translation().x() = xi;
        tfi.translation().y() = 0.2;
        task.push_back(ocpl::TSR(tfi, b));
    }

    PlannerSettings s;
    s.type = PlannerType::LOCAL_DFS;
    s.sampler_type = SamplerType::HALTON;
    s.is_redundant = false;
    s.c_space_batch_size = 10;
    s.t_space_batch_size = 10;
    s.debug = true;
    // std::cout << s << "\n";

    Robot bot {2, 0, jl_, fk_, ik_, isValid_};

    UnifiedPlanner planner(bot, s);
    Solution sol = planner.solve(task);

    EXPECT_TRUE(sol.success);

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


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
