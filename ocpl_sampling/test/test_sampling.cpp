#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_sampling/random_sampler.h>
#include <ocpl_sampling/halton_sampler.h>

#include <gtest/gtest.h>
#include <iostream>

const bool VERBOSE = false;

using Grid = std::vector<std::vector<double>>;
using namespace ocpl;

void printGrid(Grid& grid);
void compareGrids(Grid& actual, Grid& expected);

TEST(TestGridSampler, addDimensions)
{
    GridSampler s;
    s.addDimension(0, 0, 1);
    s.addDimension(1, 2, 2);
    s.addDimension(99, 99, 1);
    s.addDimension(5, 7, 3);
    Grid grid = s.getSamples();
    Grid expected = { { 0, 1, 99, 5 }, { 0, 1, 99, 6 }, { 0, 1, 99, 7 },
                      { 0, 2, 99, 5 }, { 0, 2, 99, 6 }, { 0, 2, 99, 7 } };
    compareGrids(grid, expected);

    if (VERBOSE)
        printGrid(grid);
    EXPECT_TRUE(true);
}

TEST(TestGridSampler, range)
{
    std::vector<double> r = ocpl::GridSampler::range(-1, 2, 4);
    ASSERT_EQ(r.size(), 4);
    EXPECT_DOUBLE_EQ(r[0], -1);
    EXPECT_DOUBLE_EQ(r[1], 0);
    EXPECT_DOUBLE_EQ(r[2], 1);
    EXPECT_DOUBLE_EQ(r[3], 2);
}

TEST(TestSampler, wrongInput)
{
    GridSampler s;
    EXPECT_ANY_THROW(s.addDimension(-1, 2, 1));
}

TEST(TestRandomSampler, TestEigenVector)
{
    Eigen::VectorXd lower(4), upper(4);
    lower << -3, -4, 1, 99;
    upper << -2, 1, 2, 99;

    auto s = ocpl::RandomSampler();
    for (int i = 0; i < lower.size(); ++i)
    {
        s.addDimension(lower[i], upper[i]);
    }

    auto samples = s.getSamplesV(10);

    // check that they are different
    for (int i{1}; i < samples.size(); ++i)
    {
      EXPECT_GT((samples[i] - samples[i-1]).norm(), 1e-16);
    }

    // check bounds
    for (auto sample : samples)
    {
        for (int k{0}; k < sample.size(); ++k)
        {
          EXPECT_LE(sample[k], upper[k]);
          EXPECT_GE(sample[k], lower[k]);
        }
    }
}

TEST(TestHaltonSampler, TestEigenVector)
{
    Eigen::VectorXd lower(4), upper(4);
    lower << -3, -4, 1, 99;
    upper << -2, 1, 2, 99;

    auto s = ocpl::HaltonSampler();
    for (int i = 0; i < lower.size(); ++i)
    {
        s.addDimension(lower[i], upper[i]);
    }

    auto samples = s.getSamplesV(10);

    for (auto sample : samples)
    {
      std::cout << sample.transpose() << "\n";
    }

    // check that they are different
    for (int i{1}; i < samples.size(); ++i)
    {
      EXPECT_GT((samples[i] - samples[i-1]).norm(), 1e-16);
    }

    // check bounds
    for (auto sample : samples)
    {
        for (int k{0}; k < sample.size(); ++k)
        {
          EXPECT_LE(sample[k], upper[k]);
          EXPECT_GE(sample[k], lower[k]);
        }
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

void printGrid(Grid& grid)
{
    if (grid.size() > 0)
    {
        std::cout << "===== GRID =======\n";
        for (auto jv : grid)
        {
            std::cout << "( ";
            for (auto val : jv)
            {
                std::cout << val << ", ";
            }
            std::cout << ")\n";
        }
    }
    else
    {
        std::cout << "===== NO GRID ======";
    }
    std::cout << std::endl;
}

void compareGrids(Grid& actual, Grid& expected)
{
    ASSERT_EQ(actual.size(), expected.size());
    std::size_t nrows = actual.size();
    for (std::size_t i = 0; i < nrows; ++i)
    {
        ASSERT_EQ(actual[i].size(), expected[i].size());
        std::size_t ncols = actual[i].size();
        for (std::size_t j = 0; j < ncols; ++j)
        {
            EXPECT_DOUBLE_EQ(expected[i][j], actual[i][j]);
        }
    }
}
