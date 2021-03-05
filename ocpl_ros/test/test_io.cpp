#include <ocpl_ros/io.h>

#include <gtest/gtest.h>

TEST(TestReadSettings, TestStringToVector)
{
    std::vector<int> v = ocpl::stringToVector<int>("1 3 2");
    EXPECT_EQ(v.at(0), 1);
    EXPECT_EQ(v.at(1), 3);
    EXPECT_EQ(v.at(2), 2);
}

TEST(TestReadSettings, TestProccessLIne)
{
    auto p = ocpl::impl::proccessLine("key: value");
    EXPECT_EQ(p.first, "key");
    EXPECT_EQ(p.second, "value");

    auto p2 = ocpl::impl::proccessLine("key: 0.1");
    EXPECT_EQ(p2.first, "key");
    EXPECT_EQ(p2.second, "0.1");
    EXPECT_EQ(std::stod(p2.second), 0.1);

    ASSERT_THROW(ocpl::impl::proccessLine("key: 0.1: kfe"), std::runtime_error);
    ASSERT_THROW(ocpl::impl::proccessLine(": "), std::runtime_error);
    ASSERT_THROW(ocpl::impl::proccessLine(": value"), std::runtime_error);
}

TEST(TestReadSettings, SimpleTest)
{
    auto s = ocpl::readSettingsFile("test_settings.txt");
    ASSERT_TRUE(true);

    ocpl::SettingsMap test_map{
        { "name", "settings1" }, { "method", "local" }, { "somedouble", "0.1" }, { "someint", "8" }
    };

    for (auto& p : test_map)
    {
        EXPECT_TRUE(s.find(p.first) != s.end());
        EXPECT_EQ(s.at(p.first), p.second);
    }

    ASSERT_THROW(ocpl::readSettingsFile("garbage.txt"), std::runtime_error);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
