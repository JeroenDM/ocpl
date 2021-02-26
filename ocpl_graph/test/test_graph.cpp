#include <ocpl_graph/containers.h>
#include <ocpl_graph/graph.h>

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

inline double norm2Diff(const std::vector<double>& n1, const std::vector<double>& n2)
{
    assert(n1.size() == n2.size());

    double cost{ 0.0 };
    for (int i = 0; i < n1.size(); ++i)
    {
        cost += std::sqrt((n1[i] - n2[i]) * (n1[i] - n2[i]));
    }
    return cost;
}

using namespace ocpl;

TEST(TestContainers, TestStack)
{
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

TEST(TestGraph, TestGraphSearch)
{
    using Data = std::vector<double>;
    using MyNode = tree::Node<Data>;
    using MyNodePtr = tree::NodePtr<Data>;
    // create graph
    std::vector<Data> jp1 = { { 0, 0 }, { 0, 1 } };
    std::vector<Data> jp2 = { { 1, -1 }, { 1, 0 }, { 1, 1 } };
    std::vector<Data> jp3 = { { 0, 2 }, { 2, 2 } };

    std::vector<std::vector<tree::NodePtr<Data>>> graph(3);
    for (size_t k{ 0 }; k < 3; ++k)
    {
        std::transform(jp1.begin(), jp1.end(), std::back_inserter(graph.at(k)),
                       [k](const Data& d) { return std::make_shared<MyNode>(d, k); });
    }

    QueueContainer<tree::NodePtr<Data>> container;

    auto nb_fun = [&graph](const MyNodePtr& /* n */, const size_t k) { return graph.at(k + 1); };
    auto d_fun = [](const MyNodePtr& a, const MyNodePtr& b) { return norm2Diff(a->data, b->data); };
    // tree::search(container, nb_fun, graph.front(), d_fun);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
