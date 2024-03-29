#include <ocpl_graph/containers.h>
#include <ocpl_graph/graph.h>

#include <gtest/gtest.h>
#include <algorithm>

#include <iostream>

inline double norm1Diff(const std::vector<double>& n1, const std::vector<double>& n2)
{
    assert(n1.size() == n2.size());

    double cost{ 0.0 };
    for (int i = 0; i < n1.size(); ++i)
    {
        cost += std::abs(n1[i] - n2[i]);
    }
    return cost;
}

using namespace ocpl_graph;
using Data = std::vector<double>;

TEST(TestMiniGraph, BreathVsDepth)
{
    // [ 0 0 ]  [ n1 n2 ]
    // [ 0 x ]  [ n3 n4 ]

    std::vector<double> dummy_data;

    NodePtr n1 = std::make_shared<Node>(dummy_data, 0.0);
    NodePtr n2 = std::make_shared<Node>(dummy_data, 0.0);
    NodePtr n3 = std::make_shared<Node>(dummy_data, 0.0);
    NodePtr n4 = std::make_shared<Node>(dummy_data, 0.0);

    n3->waypoint_index = 1;
    n4->waypoint_index = 1;

    std::vector<std::vector<NodePtr>> nodes{ { n1, n2 }, { n3, n4 } };

    auto d_fun = [](const NodePtr& /*a*/, const NodePtr& /*b*/) { return 1.0; };
    auto nb_fun = [&nodes](const NodePtr& /*n*/) { return std::vector<NodePtr>{ nodes.at(1).at(0) }; };

    QueueContainer<NodePtr> queue;
    Graph graph1(nb_fun, 2, { n1, n2 });
    auto path_q = shortest_path_dag(graph1, d_fun, queue);

    ASSERT_EQ(path_q.size(), 2);
    EXPECT_EQ(path_q[0], n1);
    EXPECT_EQ(path_q[1], n3);

    n1->reset();
    n2->reset();
    n3->reset();
    n4->reset();

    StackContainer<NodePtr> stack;
    Graph graph2(nb_fun, 2, { n1, n2 });
    auto path_s = shortest_path_dag(graph2, d_fun, stack);

    ASSERT_EQ(path_s.size(), 2);
    EXPECT_EQ(path_s[0], n2);
    EXPECT_EQ(path_s[1], n3);
}

TEST(TestGraph, TestBreathFirst)
{
    // create some artificial graph data
    std::vector<std::vector<Data>> gd = { { { 0, 0 }, { 0, 1 } },
                                          { { 1, -1 }, { 1, 1 }, { 1, 0 } },
                                          { { 0, 2 }, { 2, 2 } } };

    // convert it to Node pointers
    std::vector<std::vector<NodePtr>> samples(3);
    for (size_t k{ 0 }; k < 3; ++k)
    {
        std::transform(gd.at(k).begin(), gd.at(k).end(), std::back_inserter(samples.at(k)), [k](const Data& d) {
            auto node = std::make_shared<Node>(d, 0.0);
            node->waypoint_index = k;
            return node;
        });
    }

    QueueContainer<NodePtr> container;

    auto nb_fun = [&samples](const NodePtr& n) { return samples.at(n->waypoint_index + 1); };
    Graph graph(nb_fun, 3, samples.at(0));

    auto d_fun = [](const NodePtr& a, const NodePtr& b) { return norm1Diff(a->data, b->data); };
    auto path = shortest_path_dag(graph, d_fun, container);

    std::vector<Data> solution;
    std::transform(path.begin(), path.end(), std::back_inserter(solution), [](const NodePtr& n) { return n->data; });

    // for (auto wp : path)
    // {
    //     std::cout << *wp << "\n";
    // }

    EXPECT_EQ(solution.at(0).at(0), 0);
    EXPECT_EQ(solution.at(0).at(1), 1);
    EXPECT_EQ(solution.at(1).at(0), 1);
    EXPECT_EQ(solution.at(1).at(1), 1);
    EXPECT_EQ(solution.at(2).at(0), 0);
    EXPECT_EQ(solution.at(2).at(1), 2);

    EXPECT_EQ(path.at(0)->dist, 0.0);
    EXPECT_EQ(path.at(1)->dist, 1.0);
    EXPECT_EQ(path.at(2)->dist, 3.0);
}

TEST(TestGraph, TestDepthFirst)
{
    // create some artificial graph data
    std::vector<std::vector<Data>> gd = { { { 0, 0 }, { 0, 1 } },
                                          { { 1, -1 }, { 1, 1 }, { 1, 0 } },
                                          { { 0, 2 }, { 2, 2 } } };

    // convert it to Node pointers
    std::vector<std::vector<NodePtr>> samples(3);
    for (size_t k{ 0 }; k < 3; ++k)
    {
        std::transform(gd.at(k).begin(), gd.at(k).end(), std::back_inserter(samples.at(k)), [k](const Data& d) {
            auto node = std::make_shared<Node>(d, 0.0);
            node->waypoint_index = k;
            return node;
        });
    }

    auto nb_fun = [&samples](const NodePtr& n) { return samples.at(n->waypoint_index + 1); };
    Graph graph(nb_fun, 3, samples.at(0));

    auto d_fun = [](const NodePtr& a, const NodePtr& b) { return norm1Diff(a->data, b->data); };

    // stack container results in depth first search,
    // but the nodes are accessed in what might appear as a reversed way
    // because they are added in order and the top of the stack is popped first of course
    StackContainer<NodePtr> stack;
    auto path = shortest_path_dag(graph, d_fun, stack);
    std::vector<Data> solution;
    std::transform(path.begin(), path.end(), std::back_inserter(solution), [](const NodePtr& n) { return n->data; });

    // for (auto wp : path)
    // {
    //     std::cout << *wp << "\n";
    // }

    EXPECT_EQ(solution.at(0).at(0), 0);
    EXPECT_EQ(solution.at(0).at(1), 1);
    EXPECT_EQ(solution.at(1).at(0), 1);
    EXPECT_EQ(solution.at(1).at(1), 0);
    EXPECT_EQ(solution.at(2).at(0), 2);
    EXPECT_EQ(solution.at(2).at(1), 2);

    EXPECT_EQ(path.at(0)->dist, 0.0);
    EXPECT_EQ(path.at(1)->dist, 2.0);
    EXPECT_EQ(path.at(2)->dist, 5.0);
}

TEST(TestGraph, TestDijkstra)
{
    // create some artificial graph data
    std::vector<std::vector<Data>> gd = { { { 0, 0 }, { 0, 1 } },
                                          { { 1, -1 }, { 1, 1 }, { 1, 0 } },
                                          { { 0, 2 }, { 2, 2 } } };

    // convert it to Node pointers
    std::vector<std::vector<NodePtr>> samples(3);
    for (size_t k{ 0 }; k < 3; ++k)
    {
        std::transform(gd.at(k).begin(), gd.at(k).end(), std::back_inserter(samples.at(k)), [k](const Data& d) {
            auto node = std::make_shared<Node>(d, 0.0);
            node->waypoint_index = k;
            return node;
        });
    }

    auto nb_fun = [&samples](const NodePtr& n) { return samples.at(n->waypoint_index + 1); };
    Graph graph(nb_fun, 3, samples.at(0));

    auto d_fun = [](const NodePtr& a, const NodePtr& b) { return norm1Diff(a->data, b->data); };

    // stack container results in depth first search,
    // but the nodes are accessed in what might appear as a reversed way
    // because they are added in order and the top of the stack is popped first of course
    PriorityQueueContainer<NodePtr> container(3, d_fun);
    auto path = shortest_path_dag(graph, d_fun, container);
    std::vector<Data> solution;
    std::transform(path.begin(), path.end(), std::back_inserter(solution), [](const NodePtr& n) { return n->data; });

    // for (auto wp : path2)
    // {
    //     std::cout << *wp << "\n";
    // }

    EXPECT_EQ(solution.at(0).at(0), 0);
    EXPECT_EQ(solution.at(0).at(1), 1);
    EXPECT_EQ(solution.at(1).at(0), 1);
    EXPECT_EQ(solution.at(1).at(1), 1);
    EXPECT_EQ(solution.at(2).at(0), 2);
    EXPECT_EQ(solution.at(2).at(1), 2);

    EXPECT_EQ(path.at(0)->dist, 0.0);
    EXPECT_EQ(path.at(1)->dist, 1.0);
    EXPECT_EQ(path.at(2)->dist, 3.0);
}

TEST(TestGraph, TestBestFirst)
{
    // create some artificial graph data
    std::vector<std::vector<Data>> gd = { { { 0, 0 }, { 0, 1 } },
                                          { { 1, -1 }, { 1, 1 }, { 1, 0 } },
                                          { { 0, 2 }, { 2, 2 } } };

    // convert it to Node pointers
    std::vector<std::vector<NodePtr>> samples(3);
    for (size_t k{ 0 }; k < 3; ++k)
    {
        std::transform(gd.at(k).begin(), gd.at(k).end(), std::back_inserter(samples.at(k)), [k](const Data& d) {
            auto node = std::make_shared<Node>(d, 0.0);
            node->waypoint_index = k;
            return node;
        });
    }

    auto nb_fun = [&samples](const NodePtr& n) { return samples.at(n->waypoint_index + 1); };
    Graph graph(nb_fun, 3, samples.at(0));

    auto d_fun = [](const NodePtr& a, const NodePtr& b) { return norm1Diff(a->data, b->data); };

    // stack container results in depth first search,
    // but the nodes are accessed in what might appear as a reversed way
    // because they are added in order and the top of the stack is popped first of course
    PriorityStackContainer<NodePtr> container(3, d_fun);
    auto path2 = shortest_path_dag(graph, d_fun, container);
    std::vector<Data> solution2;
    std::transform(path2.begin(), path2.end(), std::back_inserter(solution2), [](const NodePtr& n) { return n->data; });

    // for (auto wp : path2)
    // {
    //     std::cout << *wp << "\n";
    // }

    EXPECT_EQ(solution2.at(0).at(0), 0);
    EXPECT_EQ(solution2.at(0).at(1), 1);
    EXPECT_EQ(solution2.at(1).at(0), 1);
    EXPECT_EQ(solution2.at(1).at(1), 0);
    EXPECT_EQ(solution2.at(2).at(0), 2);
    EXPECT_EQ(solution2.at(2).at(1), 2);

    EXPECT_EQ(path2.at(0)->dist, 0.0);
    EXPECT_EQ(path2.at(1)->dist, 2.0);
    EXPECT_EQ(path2.at(2)->dist, 5.0);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
