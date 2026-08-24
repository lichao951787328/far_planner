#include <gtest/gtest.h>

#include "far_planner/contour_detector.h"

TEST(ContourSimplificationPolicy, RemovesVoxelJitterOnStraightWall) {
    PointStack contour = {
        Point3D(0.0f, 0.0f, 0.0f), Point3D(2.0f, 0.08f, 0.0f),
        Point3D(4.0f, 0.0f, 0.0f), Point3D(4.0f, 2.0f, 0.0f),
        Point3D(0.0f, 2.0f, 0.0f)};
    SimplifyClosedContourCollinearVertices(contour, 0.20f, 8.0f);
    EXPECT_EQ(4u, contour.size());
}

TEST(ContourSimplificationPolicy, PreservesDoorLipAndRightAngle) {
    PointStack contour = {
        Point3D(0.0f, 0.0f, 0.0f), Point3D(2.0f, 0.0f, 0.0f),
        Point3D(2.0f, 0.5f, 0.0f), Point3D(4.0f, 0.5f, 0.0f),
        Point3D(4.0f, 2.0f, 0.0f), Point3D(0.0f, 2.0f, 0.0f)};
    SimplifyClosedContourCollinearVertices(contour, 0.20f, 8.0f);
    EXPECT_EQ(6u, contour.size());
}

TEST(ContourSimplificationPolicy, AggressiveProfileCollapsesShortZigZagRun) {
    PointStack contour = {
        Point3D(0.0f, 0.0f, 0.0f), Point3D(1.0f, 0.10f, 0.0f),
        Point3D(2.0f, 0.0f, 0.0f), Point3D(3.0f, 0.10f, 0.0f),
        Point3D(4.0f, 0.0f, 0.0f), Point3D(4.0f, 2.0f, 0.0f),
        Point3D(0.0f, 2.0f, 0.0f)};
    SimplifyClosedContourCollinearVertices(contour, 0.30f, 15.0f);
    EXPECT_EQ(4u, contour.size());
}

TEST(ContourSimplificationPolicy, AggressiveProfileStillPreservesDoorLip) {
    PointStack contour = {
        Point3D(0.0f, 0.0f, 0.0f), Point3D(2.0f, 0.0f, 0.0f),
        Point3D(2.0f, 0.5f, 0.0f), Point3D(4.0f, 0.5f, 0.0f),
        Point3D(4.0f, 2.0f, 0.0f), Point3D(0.0f, 2.0f, 0.0f)};
    SimplifyClosedContourCollinearVertices(contour, 0.30f, 15.0f);
    EXPECT_EQ(6u, contour.size());
}

TEST(ContourRasterAlignment, SnapsPositiveAndNegativeWorldCoordinates) {
    EXPECT_NEAR(1.2f, AlignContourRasterCoordinate(1.13f, 0.2f), 1e-6f);
    EXPECT_NEAR(-1.2f, AlignContourRasterCoordinate(-1.13f, 0.2f), 1e-6f);
}

TEST(ContourRasterAlignment, ReconstructsTheSameWorldCellWhileRobotMoves) {
    constexpr float resolution = 0.2f;
    constexpr float obstacle_world_x = 2.4f;
    const float robot_positions[] = {0.03f, 0.08f, 0.12f, 0.31f, -0.11f};
    for (const float robot_x : robot_positions) {
        const float raster_center =
            AlignContourRasterCoordinate(robot_x, resolution);
        const int cell_offset = static_cast<int>(std::round(
            (obstacle_world_x - raster_center) / resolution));
        const float reconstructed =
            raster_center + static_cast<float>(cell_offset) * resolution;
        EXPECT_NEAR(obstacle_world_x, reconstructed, 1e-5f)
            << "robot_x=" << robot_x
            << " raster_center=" << raster_center;
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
