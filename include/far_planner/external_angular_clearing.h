#ifndef FAR_EXTERNAL_ANGULAR_CLEARING_H
#define FAR_EXTERNAL_ANGULAR_CLEARING_H

#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "point_struct.h"

struct ExternalAngularClearingParams {
    float voxel_size = 0.1f;
    float endpoint_margin = 0.2f;
    int confirmations = 2;
    bool require_both_layers = true;
};

struct ExternalAngularClearingRay {
    Point3D origin;
    Point3D endpoint;
    bool endpoint_is_hit = false;
    std::uint8_t layer = 0u;
    std::uint16_t angle_bin = 0u;
};

/**
 * Convert the Local3D two-layer angular ray records into exact historical FAR
 * points that may be removed. The traversed ray interior is free evidence;
 * hit/no-hit endpoints retain a safety margin. The filter deliberately operates
 * on XY columns because
 * the configured FAR contour graph is 2-D.  When both-layer confirmation is
 * enabled, a column must be traversed in both producer layers and in consecutive
 * acquisition frames before it is returned as confirmed.
 */
class ExternalAngularClearingFilter {
public:
    ExternalAngularClearingFilter() = default;

    void Init(const ExternalAngularClearingParams& params);
    void Reset();

    void Filter(const std::vector<ExternalAngularClearingRay>& rays,
                const PointCloudPtr& historical_obstacles,
                const PointCloudPtr& current_obstacles,
                const PointCloudPtr& protected_static_obstacles,
                const PointCloudPtr& candidates_out,
                const PointCloudPtr& confirmed_out,
                const PointCloudPtr& common_free_mask_out);

private:
    struct VoteState {
        int count = 0;
        std::uint64_t last_frame = 0u;
    };

    ExternalAngularClearingParams params_;
    std::uint64_t frame_sequence_ = 0u;
    std::unordered_map<std::uint64_t, VoteState> votes_;

    std::uint64_t PointKey(const PCLPoint& point) const;
    std::uint64_t PositionKey(double x, double y) const;
    void RasterizeRay(const ExternalAngularClearingRay& ray,
                      std::unordered_set<std::uint64_t>* free_keys) const;
};

#endif
