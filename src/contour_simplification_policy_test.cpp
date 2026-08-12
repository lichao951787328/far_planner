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

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
