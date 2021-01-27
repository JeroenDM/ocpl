#include <ocpl_planning/generic_attempt.h>

#include <iostream>

#include <gtest/gtest.h>

using namespace planner;

TEST(Testbasics, Test1)
{
    // dummy path sampler
    LocalSampler sampler = [](size_t i, const JointPositions& q_bias) {
        std::vector<JointPositions> s;
        if (q_bias.x() == 2 && q_bias.y() == 2)
        {
            return s;
        }
        s.push_back(Eigen::Vector2d((double)(i), 1, q_bias.x() + 2, 0.0));
        s.push_back(Eigen::Vector2d((double)(i), 2, q_bias.x(), 0.0));
        return s;
    };

    // auto comp = [](const Vertice& a, const Vertice& b){ return a.q.back() > b.q.back();};
    auto cost_fun = [](const JointPositions& from, const JointPositions& to) { return 1.0; };

    auto samples = sampler(3, Eigen::Vector3d( 9, 8, 7 ));
    for (auto sample : samples)
        std::cout << sample << "\n";

    JointPositions q_start{ 0, 99, 99 };
    // StackContainer container;
    // QueueContainer container;
    // PriorityQueueContainer container(5, comp);
    PriorityStackContainer container(5);
    auto sol = search(q_start, sampler, 5, container, cost_fun);

    std::cout << "--- solution ---\n";
    for (auto q : sol)
        std::cout << q << "\n";

    EXPECT_TRUE(true);
}

TEST(Test2DGrid, TestGrid1)
{
    size_t N{ 6 }, M{ 3 };

    LocalSampler sampler = [N, M](size_t i, const JointPositions& q_bias) {
        std::vector<JointPositions> samples;

        if (i < M)
        {
            samples.push_back(Eigen::Vector2d((double)i, q_bias[1]));
            if (q_bias[1] > 0)
                samples.push_back(Eigen::Vector2d((double)i, q_bias[1] - 1));
            if (q_bias[1] < N - 1)
                samples.push_back(Eigen::Vector2d((double)i, q_bias[1] + 1));
        }
        return samples;
    };
    DistanceMetric cost_fun = [](const JointPositions& from, const JointPositions& to) { return 1.0; };

    JointPositions q_start;
    q_start << 0.0, 3.0;
    // StackContainer container;
    QueueContainer container;
    // PriorityQueueContainer container(M);
    // PriorityStackContainer container(M);
    auto sol = search(q_start, sampler, M, container, cost_fun);

    std::cout << "--- solution ---\n";
    for (auto q : sol)
        std::cout << q << "\n";

    EXPECT_TRUE(true);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
