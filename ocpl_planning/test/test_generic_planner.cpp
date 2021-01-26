#include <ocpl_planning/generic_attempt.h>

#include <iostream>

#include <gtest/gtest.h>

using namespace planner;

TEST(Testbasics, Test1)
{
    // dummy path sampler
    LocalSampler sampler = [](size_t i, const JointPositions& q_bias) {
        std::vector<JointPositions> s;
        if (q_bias.at(0) == 2 && q_bias.at(1) == 2)
        {
            return s;
        }
        s.push_back({ (double)(i), 1, q_bias.at(0) + 2 });
        s.push_back({ (double)(i), 2, q_bias.at(0) });
        return s;
    };

    // auto comp = [](const Vertice& a, const Vertice& b){ return a.q.back() > b.q.back();};
    auto cost_fun = [](const JointPositions& from, const JointPositions& to) { return to.back(); };

    auto samples = sampler(3, { 9, 8, 7 });
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
    std::vector<std::vector<JointPositions>> states;
    states.resize(M);

    for (size_t way_pt{ 0 }; way_pt < M; ++way_pt)
    {
        for (size_t i{ 0 }; i < N; ++i)
        {
            states[way_pt].push_back({ (double)way_pt, (double)i, 0.0 });
        }
    }

    states[1][0].back() = 3.0;

    LocalSampler sampler = [&states](size_t i, const JointPositions& q_bias) { return states.at(i); };
    DistanceMetric cost_fun = [](const JointPositions& from, const JointPositions& to) { return to.back(); };

    JointPositions q_start = sampler(0, { 0.0, 0.0, 0.0 }).at(0);
    // StackContainer container;
    QueueContainer container;
    // PriorityQueueContainer container(M);
    // PriorityStackContainer container(M);
    auto sol = search(q_start, sampler, M, container, cost_fun);

    for (auto row : states)
    {
        for (auto state : row)
        {
            std::cout << "|" << state << "|";
        }
        std::cout << "\n";
    }

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
