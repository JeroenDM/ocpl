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
        s.push_back({ (double)(i), 1, q_bias.at(0) });
        s.push_back({ (double)(i), 2, q_bias.at(0) });
        return s;
    };

    auto samples = sampler(3, { 9, 8, 7 });
    for (auto sample : samples)
        std::cout << sample << "\n";

    JointPositions q_start{ 0, 99, 99 };
    // StackContainer container;
    QueueContainer container;
    auto sol = search(q_start, sampler, 5, container);

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
