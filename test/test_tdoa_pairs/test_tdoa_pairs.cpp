#include <gtest/gtest.h>

#include "uwb/tdoa_pairs.hpp"

TEST(TDoAPairs, PairIndexMatchesExpectedFourAnchorOrder)
{
    EXPECT_EQ(tdoa::PairCount(4), 6);
    EXPECT_EQ(tdoa::PairIndex(0, 1, 4), 0);
    EXPECT_EQ(tdoa::PairIndex(0, 2, 4), 1);
    EXPECT_EQ(tdoa::PairIndex(0, 3, 4), 2);
    EXPECT_EQ(tdoa::PairIndex(1, 2, 4), 3);
    EXPECT_EQ(tdoa::PairIndex(1, 3, 4), 4);
    EXPECT_EQ(tdoa::PairIndex(2, 3, 4), 5);
}

TEST(TDoAPairs, PairIndexIsOrderIndependent)
{
    EXPECT_EQ(tdoa::PairIndex(1, 0, 4), 0);
    EXPECT_EQ(tdoa::PairIndex(3, 0, 4), 2);
    EXPECT_EQ(tdoa::PairIndex(3, 2, 4), 5);
}

TEST(TDoAPairs, CanonicalizeReportsReverseOrder)
{
    tdoa::AnchorPair pair;
    bool reversed = false;

    ASSERT_TRUE(tdoa::CanonicalizePair(3, 1, 4, pair, reversed));
    EXPECT_EQ(pair.a, 1);
    EXPECT_EQ(pair.b, 3);
    EXPECT_TRUE(reversed);

    ASSERT_TRUE(tdoa::CanonicalizePair(1, 3, 4, pair, reversed));
    EXPECT_EQ(pair.a, 1);
    EXPECT_EQ(pair.b, 3);
    EXPECT_FALSE(reversed);
}

TEST(TDoAPairs, RejectsInvalidPairs)
{
    tdoa::AnchorPair pair;
    bool reversed = false;

    EXPECT_FALSE(tdoa::CanonicalizePair(1, 1, 4, pair, reversed));
    EXPECT_EQ(tdoa::PairIndex(4, 1, 4), -1);
    EXPECT_EQ(tdoa::PairIndex(1, 4, 4), -1);
}

TEST(TDoAPairs, PairByIndexMatchesFourAnchorOrder)
{
    constexpr tdoa::AnchorPair p0 = tdoa::PairByIndex<4>(0);
    constexpr tdoa::AnchorPair p5 = tdoa::PairByIndex<4>(5);

    EXPECT_EQ(p0.a, 0);
    EXPECT_EQ(p0.b, 1);
    EXPECT_EQ(p5.a, 2);
    EXPECT_EQ(p5.b, 3);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    if (RUN_ALL_TESTS())
    ;
    return 0;
}
