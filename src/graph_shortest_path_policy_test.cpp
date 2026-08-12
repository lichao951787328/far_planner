#include <gtest/gtest.h>

#include "far_planner/graph_planner.h"

namespace {

NavNodePtr MakeNode(const std::size_t id) {
    NavNodePtr node(new NavNode());
    node->id = id;
    node->gscore = std::numeric_limits<float>::max();
    return node;
}

TEST(GraphShortestPathPolicy, LowerCostSnapshotPrecedesOldSnapshot) {
    const NavNodePtr node = MakeNode(7);
    std::priority_queue<GraphSearchQueueEntry,
                        std::vector<GraphSearchQueueEntry>,
                        GraphSearchQueueEntryGreater> queue;
    queue.push({10.0f, node->id, node});
    queue.push({3.0f, node->id, node});

    EXPECT_FLOAT_EQ(3.0f, queue.top().cost);
    node->gscore = 3.0f;
    queue.pop();
    EXPECT_TRUE(IsStaleGraphSearchEntry(queue.top(), node->gscore));
}

TEST(GraphShortestPathPolicy, EqualCostsUseStableParentId) {
    const NavNodePtr high_parent = MakeNode(9);
    const NavNodePtr low_parent = MakeNode(4);

    EXPECT_TRUE(ShouldRelaxGraphSearchEdge(5.0f, 5.0f,
                                           low_parent->id, high_parent));
    EXPECT_FALSE(ShouldRelaxGraphSearchEdge(5.0f, 5.0f,
                                            high_parent->id, low_parent));
}

TEST(GraphShortestPathPolicy, DiamondDecreaseKeyFindsShortBranch) {
    // This reproduces the old failure mode: B first enters the heap with 8,
    // then A improves it to 2.  The immutable 2-cost entry must be expanded
    // before the unrelated 4-cost node.
    const NavNodePtr start = MakeNode(1);
    const NavNodePtr a = MakeNode(2);
    const NavNodePtr b = MakeNode(3);
    const NavNodePtr unrelated = MakeNode(4);
    std::priority_queue<GraphSearchQueueEntry,
                        std::vector<GraphSearchQueueEntry>,
                        GraphSearchQueueEntryGreater> queue;

    start->gscore = 0.0f;
    b->gscore = 8.0f;
    queue.push({8.0f, b->id, b});
    unrelated->gscore = 4.0f;
    queue.push({4.0f, unrelated->id, unrelated});
    a->gscore = 1.0f;
    queue.push({1.0f, a->id, a});

    GraphSearchQueueEntry entry = queue.top();
    queue.pop();
    ASSERT_EQ(a->id, entry.node_id);
    ASSERT_TRUE(ShouldRelaxGraphSearchEdge(2.0f, b->gscore,
                                           a->id, b->parent));
    b->gscore = 2.0f;
    b->parent = a;
    queue.push({2.0f, b->id, b});

    EXPECT_EQ(b->id, queue.top().node_id);
    EXPECT_FLOAT_EQ(2.0f, queue.top().cost);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
