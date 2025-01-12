#include <gtest/gtest.h>

TEST(package_name, a_first_test)
{
    ASSERT_EQ(4, 2 + 2);
    ASSERT_EQ(4, 6);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
