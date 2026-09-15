#include <gtest/gtest.h>

#include "far_planner/external_angular_clearing.h"

namespace {

PCLPoint Point(const float x, const float y, const float z = 0.0f,
               const float intensity = 0.0f) {
  PCLPoint point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  return point;
}

ExternalAngularClearingRay Ray(const std::uint8_t layer,
                               const float endpoint_x = 1.05f) {
  ExternalAngularClearingRay ray;
  ray.origin = Point3D(0.05f, 0.05f, layer == 0u ? 0.4f : 0.0f);
  ray.endpoint = Point3D(endpoint_x, 0.05f,
                         layer == 0u ? 0.4f : 0.0f);
  ray.layer = layer;
  return ray;
}

struct FilterClouds {
  PointCloudPtr history{new PointCloud()};
  PointCloudPtr current{new PointCloud()};
  PointCloudPtr protected_static{new PointCloud()};
  PointCloudPtr candidates{new PointCloud()};
  PointCloudPtr confirmed{new PointCloud()};
  PointCloudPtr mask{new PointCloud()};
};

void Apply(ExternalAngularClearingFilter* filter,
           const std::vector<ExternalAngularClearingRay>& rays,
           FilterClouds* clouds) {
  filter->Filter(rays, clouds->history, clouds->current,
                 clouds->protected_static, clouds->candidates,
                 clouds->confirmed, clouds->mask);
}

TEST(ExternalAngularClearing, RequiresTwoConsecutiveTwoLayerFrames) {
  ExternalAngularClearingParams params;
  params.endpoint_margin = 0.1f;
  params.confirmations = 2;
  ExternalAngularClearingFilter filter;
  filter.Init(params);

  FilterClouds clouds;
  clouds.history->push_back(Point(0.56f, 0.04f, 0.31f, 77.0f));
  const std::vector<ExternalAngularClearingRay> rays = {Ray(0u), Ray(1u)};

  Apply(&filter, rays, &clouds);
  ASSERT_EQ(1u, clouds.candidates->size());
  EXPECT_TRUE(clouds.confirmed->empty());

  Apply(&filter, rays, &clouds);
  ASSERT_EQ(1u, clouds.confirmed->size());
  // Deletion must use the historical point itself, not a synthetic cell centre.
  EXPECT_FLOAT_EQ(0.56f, clouds.confirmed->front().x);
  EXPECT_FLOAT_EQ(0.04f, clouds.confirmed->front().y);
  EXPECT_FLOAT_EQ(0.31f, clouds.confirmed->front().z);
  EXPECT_FLOAT_EQ(77.0f, clouds.confirmed->front().intensity);
}

TEST(ExternalAngularClearing, RejectsEvidenceFromOnlyOneLayer) {
  ExternalAngularClearingParams params;
  params.endpoint_margin = 0.1f;
  params.confirmations = 1;
  params.require_both_layers = true;
  ExternalAngularClearingFilter filter;
  filter.Init(params);

  FilterClouds clouds;
  clouds.history->push_back(Point(0.55f, 0.05f));
  Apply(&filter, {Ray(0u)}, &clouds);

  EXPECT_TRUE(clouds.mask->empty());
  EXPECT_TRUE(clouds.candidates->empty());
  EXPECT_TRUE(clouds.confirmed->empty());
}

TEST(ExternalAngularClearing, CurrentObstacleProtectsHistoricalColumn) {
  ExternalAngularClearingParams params;
  params.endpoint_margin = 0.1f;
  params.confirmations = 1;
  ExternalAngularClearingFilter filter;
  filter.Init(params);

  FilterClouds clouds;
  clouds.history->push_back(Point(0.56f, 0.04f, 0.3f));
  clouds.current->push_back(Point(0.51f, 0.09f, 1.2f));
  Apply(&filter, {Ray(0u), Ray(1u)}, &clouds);

  EXPECT_TRUE(clouds.candidates->empty());
  EXPECT_TRUE(clouds.confirmed->empty());
}

TEST(ExternalAngularClearing, ProtectedSemanticColumnCannotBeCleared) {
  ExternalAngularClearingParams params;
  params.endpoint_margin = 0.1f;
  params.confirmations = 1;
  ExternalAngularClearingFilter filter;
  filter.Init(params);

  FilterClouds clouds;
  clouds.history->push_back(Point(0.56f, 0.04f, 0.3f));
  clouds.protected_static->push_back(Point(0.51f, 0.09f, 1.2f));
  Apply(&filter, {Ray(0u), Ray(1u)}, &clouds);

  EXPECT_TRUE(clouds.candidates->empty());
  EXPECT_TRUE(clouds.confirmed->empty());
}

TEST(ExternalAngularClearing, EndpointMarginProtectsBoundaryCells) {
  ExternalAngularClearingParams params;
  params.endpoint_margin = 0.2f;
  params.confirmations = 1;
  ExternalAngularClearingFilter filter;
  filter.Init(params);

  FilterClouds clouds;
  clouds.history->push_back(Point(0.95f, 0.05f));
  Apply(&filter, {Ray(0u), Ray(1u)}, &clouds);

  EXPECT_TRUE(clouds.candidates->empty());
  EXPECT_TRUE(clouds.confirmed->empty());
}

TEST(ExternalAngularClearing, MissingFrameBreaksConsecutiveVote) {
  ExternalAngularClearingParams params;
  params.endpoint_margin = 0.1f;
  params.confirmations = 2;
  ExternalAngularClearingFilter filter;
  filter.Init(params);

  FilterClouds clouds;
  clouds.history->push_back(Point(0.55f, 0.05f));
  const std::vector<ExternalAngularClearingRay> rays = {Ray(0u), Ray(1u)};

  Apply(&filter, rays, &clouds);
  EXPECT_TRUE(clouds.confirmed->empty());
  Apply(&filter, {}, &clouds);
  EXPECT_TRUE(clouds.confirmed->empty());
  Apply(&filter, rays, &clouds);
  EXPECT_TRUE(clouds.confirmed->empty());
  Apply(&filter, rays, &clouds);
  EXPECT_EQ(1u, clouds.confirmed->size());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
