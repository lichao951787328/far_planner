#include <gtest/gtest.h>

#include "far_planner/local_observation_window.h"

namespace {

LocalObservationWindow2D MakeWindow() {
    LocalObservationWindow2D window;
    window.enabled = true;
    window.min_x = -4.0f;
    window.max_x = 8.0f;
    window.min_y = -5.0f;
    window.max_y = 5.0f;
    window.guard = 0.4f;
    window.origin = Point3D(10.0f, 20.0f, 0.0f);
    window.forward = Point3D(0.0f, 1.0f, 0.0f);
    return window;
}

}  // namespace

TEST(LocalObservationWindow, RotatesAsymmetricRobotBoxIntoWorld) {
    const LocalObservationWindow2D window = MakeWindow();
    EXPECT_TRUE(window.Contains(Point3D(10.0f, 27.0f, 0.0f)));
    EXPECT_FALSE(window.Contains(Point3D(10.0f, 28.0f, 0.0f)));
    EXPECT_TRUE(window.Contains(Point3D(14.0f, 20.0f, 0.0f)));
    EXPECT_FALSE(window.Contains(Point3D(15.0f, 20.0f, 0.0f)));
    EXPECT_TRUE(window.Contains(Point3D(10.0f, 17.0f, 0.0f)));
    EXPECT_FALSE(window.Contains(Point3D(10.0f, 16.0f, 0.0f)));
}

TEST(LocalObservationWindow, DistinguishesFullCoverageFromIntersection) {
    const LocalObservationWindow2D window = MakeWindow();
    const Point3D inside_a(10.0f, 18.0f, 0.0f);
    const Point3D inside_b(10.0f, 26.0f, 0.0f);
    const Point3D outside(10.0f, 29.0f, 0.0f);

    EXPECT_TRUE(window.SegmentFullyContained(inside_a, inside_b));
    EXPECT_TRUE(window.SegmentIntersects(inside_a, inside_b));
    EXPECT_FALSE(window.SegmentFullyContained(inside_a, outside));
    EXPECT_TRUE(window.SegmentIntersects(inside_a, outside));
    EXPECT_FALSE(window.SegmentIntersects(
        Point3D(16.0f, 20.0f, 0.0f),
        Point3D(16.0f, 25.0f, 0.0f)));
}

TEST(LocalObservationWindow, GuardAndContourHaloRemainCurrentOnly) {
    const LocalObservationWindow2D window = MakeWindow();
    // Robot forward is world +Y: local max_x=8 maps to world y=28.
    const Point3D guard_point(10.0f, 27.8f, 0.0f);
    const Point3D dilated_clip(10.0f, 28.3f, 0.0f);
    const Point3D unknown_outside(10.0f, 28.5f, 0.0f);
    EXPECT_FALSE(window.Contains(guard_point));
    EXPECT_TRUE(window.ContainsFull(guard_point));
    EXPECT_TRUE(window.ContainsFull(dilated_clip, 0.4f));
    EXPECT_FALSE(window.ContainsFull(unknown_outside, 0.4f));
    EXPECT_TRUE(window.SegmentFullyContainedFull(
        Point3D(10.0f, 20.0f, 0.0f), dilated_clip, 0.4f));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
