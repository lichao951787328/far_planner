#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "far_planner/terminal_visibility_policy.h"

namespace {

EdgeValidationResult Result(const bool valid,
                            const EdgeRejectReason reason,
                            const float projection) {
    EdgeValidationResult result;
    result.valid = valid;
    result.reason = reason;
    result.projection_distance = projection;
    return result;
}

struct Segment2D {
    Point3D first;
    Point3D second;
};

float Cross(const Point3D& first, const Point3D& second,
            const Point3D& third) {
    return (second.x - first.x) * (third.y - first.y) -
           (second.y - first.y) * (third.x - first.x);
}

bool Between(const float first, const float second, const float value) {
    constexpr float kTolerance = 1e-5f;
    return value >= std::min(first, second) - kTolerance &&
           value <= std::max(first, second) + kTolerance;
}

bool SegmentsIntersect(const Segment2D& lhs, const Segment2D& rhs) {
    const float lhs_first = Cross(lhs.first, lhs.second, rhs.first);
    const float lhs_second = Cross(lhs.first, lhs.second, rhs.second);
    const float rhs_first = Cross(rhs.first, rhs.second, lhs.first);
    const float rhs_second = Cross(rhs.first, rhs.second, lhs.second);
    constexpr float kTolerance = 1e-5f;
    if (((lhs_first > kTolerance && lhs_second < -kTolerance) ||
         (lhs_first < -kTolerance && lhs_second > kTolerance)) &&
        ((rhs_first > kTolerance && rhs_second < -kTolerance) ||
         (rhs_first < -kTolerance && rhs_second > kTolerance))) {
        return true;
    }
    const auto on_segment = [](const Point3D& point,
                               const Segment2D& segment) {
        return std::fabs(Cross(segment.first, segment.second, point)) <=
                   1e-5f &&
               Between(segment.first.x, segment.second.x, point.x) &&
               Between(segment.first.y, segment.second.y, point.y);
    };
    return on_segment(rhs.first, lhs) || on_segment(rhs.second, lhs) ||
           on_segment(lhs.first, rhs) || on_segment(lhs.second, rhs);
}

float DistancePointToSegment(const Point3D& point,
                             const Segment2D& segment) {
    const float dx = segment.second.x - segment.first.x;
    const float dy = segment.second.y - segment.first.y;
    const float length_squared = dx * dx + dy * dy;
    if (length_squared <= 1e-8f) {
        return std::hypot(point.x - segment.first.x,
                          point.y - segment.first.y);
    }
    const float projection = std::max(
        0.0f, std::min(
                  1.0f,
                  ((point.x - segment.first.x) * dx +
                   (point.y - segment.first.y) * dy) /
                      length_squared));
    const float closest_x = segment.first.x + projection * dx;
    const float closest_y = segment.first.y + projection * dy;
    return std::hypot(point.x - closest_x, point.y - closest_y);
}

float SegmentDistance(const Segment2D& first, const Segment2D& second) {
    if (SegmentsIntersect(first, second)) return 0.0f;
    return std::min(
        std::min(DistancePointToSegment(first.first, second),
                 DistancePointToSegment(first.second, second)),
        std::min(DistancePointToSegment(second.first, first),
                 DistancePointToSegment(second.second, first)));
}

TEST(TerminalVisibilityPolicy, DirectionDoesNotRejectSafeFartherProjection) {
    std::vector<float> checked;
    const TerminalProjectionSearchResult result =
        FindNearestSafeTerminalProjection(
            0.15f, 0.60f, 0.075f, true,
            [&checked](const float projection) {
                checked.push_back(projection);
                if (projection < 0.45f - 1e-4f) {
                    return Result(false,
                                  EdgeRejectReason::STATIC_CLOUD_BLOCKED,
                                  projection);
                }
                return Result(true, EdgeRejectReason::NONE, projection);
            });

    ASSERT_TRUE(result.validation.valid);
    EXPECT_NEAR(0.45f, result.validation.projection_distance, 1e-4f);
    EXPECT_EQ(5u, result.evaluated_candidates);
    ASSERT_EQ(result.evaluated_candidates, checked.size());
}

TEST(TerminalVisibilityPolicy, ProperWallCrossingRemainsRejected) {
    const TerminalProjectionSearchResult result =
        FindNearestSafeTerminalProjection(
            0.15f, 0.60f, 0.075f, true,
            [](const float projection) {
                return Result(false, EdgeRejectReason::POLYGON_BLOCKED,
                              projection);
            });

    EXPECT_FALSE(result.validation.valid);
    EXPECT_EQ(EdgeRejectReason::POLYGON_BLOCKED,
              result.validation.reason);
    EXPECT_NEAR(0.60f, result.validation.projection_distance, 1e-4f);
    EXPECT_EQ(7u, result.evaluated_candidates);
}

TEST(TerminalVisibilityPolicy, PillarWithoutDirectionIsCheckedOnce) {
    const TerminalProjectionSearchResult result =
        FindNearestSafeTerminalProjection(
            0.15f, 0.60f, 0.075f, false,
            [](const float projection) {
                EXPECT_FLOAT_EQ(0.0f, projection);
                return Result(true, EdgeRejectReason::NONE, projection);
            });

    EXPECT_TRUE(result.validation.valid);
    EXPECT_EQ(1u, result.evaluated_candidates);
}

TEST(TerminalVisibilityPolicy, CapturedGoalGeometryCanUseIntersectionCorner) {
    // Coordinates captured from the 2026-08-11 stuck simulation.  The test
    // models the outer vertical-wall corner and verifies that terminal policy
    // accepts it as soon as the caller's full geometry check reports free;
    // there is intentionally no historical direction gate in this policy.
    const Point3D corner(39.740f, -32.917f, 0.268f);
    const Point3D goal(41.0694f, -23.5600f, 0.0f);
    const Point3D free_bisector(0.70710678f, -0.70710678f, 0.0f);
    const std::array<Segment2D, 6> captured_contours = {{
        {{38.940f, -32.183f, 0.0f}, {14.988f, -32.142f, 0.0f}},
        {{38.986f, -9.776f, 0.0f}, {19.939f, -9.755f, 0.0f}},
        {{20.151f, -8.913f, 0.0f}, {39.744f, -8.948f, 0.0f}},
        {{39.744f, -8.948f, 0.0f}, {39.740f, -32.917f, 0.0f}},
        {{38.940f, -32.183f, 0.0f}, {38.986f, -9.776f, 0.0f}},
        {{39.740f, -32.917f, 0.0f}, {14.300f, -32.914f, 0.0f}}
    }};

    const TerminalProjectionSearchResult result =
        FindNearestSafeTerminalProjection(
            0.15f, 0.65f, 0.075f, true,
            [&](const float projection) {
                const Point3D projected =
                    corner + free_bisector * projection;
                const Segment2D route{projected, goal};
                bool collision_free = true;
                for (const Segment2D& contour : captured_contours) {
                    collision_free = collision_free &&
                                     SegmentDistance(route, contour) >=
                                         0.45f - 1e-5f;
                }
                return Result(collision_free,
                              collision_free ? EdgeRejectReason::NONE
                                             : EdgeRejectReason::POLYGON_BLOCKED,
                              projection);
            });

    ASSERT_TRUE(result.validation.valid);
    EXPECT_NEAR(0.60f, result.validation.projection_distance, 1e-4f);
    EXPECT_GT((goal - corner).norm_flat(), 9.0f);
}

TEST(TerminalVisibilityPolicy, CapturedIntersectionStillRejectsWallCrossing) {
    const Point3D corner(39.740f, -32.917f, 0.268f);
    const Point3D goal_inside_wall(30.0f, -23.5600f, 0.0f);
    const Point3D free_bisector(0.70710678f, -0.70710678f, 0.0f);
    const std::array<Segment2D, 2> corner_walls = {{
        {{39.744f, -8.948f, 0.0f}, {39.740f, -32.917f, 0.0f}},
        {{39.740f, -32.917f, 0.0f}, {14.300f, -32.914f, 0.0f}}
    }};

    const TerminalProjectionSearchResult result =
        FindNearestSafeTerminalProjection(
            0.15f, 0.65f, 0.075f, true,
            [&](const float projection) {
                const Point3D projected =
                    corner + free_bisector * projection;
                const Segment2D route{projected, goal_inside_wall};
                bool collision_free = true;
                for (const Segment2D& wall : corner_walls) {
                    collision_free = collision_free &&
                                     SegmentDistance(route, wall) >=
                                         0.45f - 1e-5f;
                }
                return Result(collision_free,
                              collision_free ? EdgeRejectReason::NONE
                                             : EdgeRejectReason::POLYGON_BLOCKED,
                              projection);
            });

    EXPECT_FALSE(result.validation.valid);
    EXPECT_EQ(EdgeRejectReason::POLYGON_BLOCKED,
              result.validation.reason);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
