#include <ocpl_graph/containers.h>
#include <ocpl_graph/graph.h>

#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include <vector>

using namespace ocpl_graph;

TEST(TestContainers, TestStack)
{
    // (3, 5) - (3) - (3, 2) - (3) - ()
    StackContainer<int> stack;
    stack.push(3, 0);
    stack.push(5, 0);

    EXPECT_FALSE(stack.empty());
    EXPECT_EQ(stack.pop(), 5);

    stack.push(2, 0);

    EXPECT_EQ(stack.pop(), 2);
    EXPECT_EQ(stack.pop(), 3);
    EXPECT_TRUE(stack.empty());
}

TEST(TestContainers, TestQueue)
{
    // (3, 5) - (5) - (5, 2) - (2) - ()
    QueueContainer<int> queue;
    queue.push(3, 0);
    queue.push(5, 0);

    EXPECT_FALSE(queue.empty());
    EXPECT_EQ(queue.pop(), 3);

    queue.push(2, 0);

    EXPECT_EQ(queue.pop(), 5);
    EXPECT_EQ(queue.pop(), 2);
    EXPECT_TRUE(queue.empty());
}

TEST(TestContainers, TestPriorityStack)
{
    // we expect the smallest values are popped first
    // and in addition the values of the last waypoint that contains values
    // are popped first
    PriorityStackContainer<int> ps(3, std::greater<int>());
    ps.push(3, 0);
    ps.push(3, 0);
    ps.push(5, 0);
    ps.push(8, 1);
    ps.push(9, 1);

    EXPECT_FALSE(ps.empty());

    EXPECT_EQ(ps.pop(), 8);
    EXPECT_EQ(ps.pop(), 9);

    EXPECT_EQ(ps.pop(), 3);

    ps.push(9, 1);
    ps.push(2, 0);

    EXPECT_EQ(ps.pop(), 9);
    EXPECT_EQ(ps.pop(), 2);
    EXPECT_EQ(ps.pop(), 3);
    EXPECT_EQ(ps.pop(), 5);
    EXPECT_TRUE(ps.empty());

    ps.push(7, 1);
    EXPECT_EQ(ps.pop(), 7);
    EXPECT_TRUE(ps.empty());
}

TEST(TestContainers, TestPriorityQueue)
{
    // we expect the smallest values are popped first
    // and in addition the values of first waypoint that contains values
    // to pop first
    PriorityQueueContainer<int> pq(3, std::greater<int>());
    pq.push(3, 0);
    pq.push(5, 0);
    pq.push(8, 1);
    pq.push(9, 1);

    EXPECT_FALSE(pq.empty());

    EXPECT_EQ(pq.pop(), 3);
    EXPECT_EQ(pq.pop(), 5);
    EXPECT_EQ(pq.pop(), 8);

    pq.push(7, 1);
    pq.push(2, 0);

    EXPECT_EQ(pq.pop(), 2);
    EXPECT_EQ(pq.pop(), 7);
    EXPECT_EQ(pq.pop(), 9);
    EXPECT_TRUE(pq.empty());
}

TEST(TestContainers, TestWithDoubleVector)
{
    using Vectord = std::vector<double>;
    PriorityStackContainer<Vectord> ps(3, [](const Vectord& a, const Vectord& b) { return a[0] > b[0]; });

    ps.push({ 3, 8 }, 0);
    ps.push({ 3, 7 }, 0);
    ps.push({ 5, 6 }, 0);
    ps.push({ 8, 5 }, 1);
    ps.push({ 9, 4 }, 1);

    EXPECT_FALSE(ps.empty());

    EXPECT_EQ(ps.pop().at(0), 8);
    EXPECT_EQ(ps.pop().at(0), 9);

    EXPECT_EQ(ps.pop().at(0), 3);

    ps.push({ 9, 8 }, 1);
    ps.push({ 2, 8 }, 0);

    EXPECT_EQ(ps.pop().at(0), 9);
    EXPECT_EQ(ps.pop().at(0), 2);
    EXPECT_EQ(ps.pop().at(0), 3);
    EXPECT_EQ(ps.pop().at(0), 5);
    EXPECT_TRUE(ps.empty());

    ps.push({ 7, 99 }, 1);
    EXPECT_EQ(ps.pop().at(0), 7);
    EXPECT_TRUE(ps.empty());
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
