#include "far_planner/external_angular_clearing.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace {

std::uint64_t PackKey(const std::int64_t x, const std::int64_t y) {
    const std::uint32_t packed_x = static_cast<std::uint32_t>(
        static_cast<std::int32_t>(x));
    const std::uint32_t packed_y = static_cast<std::uint32_t>(
        static_cast<std::int32_t>(y));
    return (static_cast<std::uint64_t>(packed_x) << 32u) | packed_y;
}

std::int32_t KeyX(const std::uint64_t key) {
    return static_cast<std::int32_t>(static_cast<std::uint32_t>(key >> 32u));
}

std::int32_t KeyY(const std::uint64_t key) {
    return static_cast<std::int32_t>(static_cast<std::uint32_t>(key));
}

}  // namespace

void ExternalAngularClearingFilter::Init(
    const ExternalAngularClearingParams& params) {
    if (!std::isfinite(params.voxel_size) || params.voxel_size <= 0.0f) {
        throw std::invalid_argument("external angular clearing voxel_size must be positive");
    }
    if (!std::isfinite(params.endpoint_margin) || params.endpoint_margin < 0.0f) {
        throw std::invalid_argument("external angular clearing endpoint_margin must be non-negative");
    }
    if (params.confirmations <= 0) {
        throw std::invalid_argument("external angular clearing confirmations must be positive");
    }
    params_ = params;
    Reset();
}

void ExternalAngularClearingFilter::Reset() {
    frame_sequence_ = 0u;
    votes_.clear();
}

std::uint64_t ExternalAngularClearingFilter::PositionKey(
    const double x, const double y) const {
    return PackKey(
        static_cast<std::int64_t>(std::floor(x / params_.voxel_size)),
        static_cast<std::int64_t>(std::floor(y / params_.voxel_size)));
}

std::uint64_t ExternalAngularClearingFilter::PointKey(
    const PCLPoint& point) const {
    return PositionKey(point.x, point.y);
}

void ExternalAngularClearingFilter::RasterizeRay(
    const ExternalAngularClearingRay& ray,
    std::unordered_set<std::uint64_t>* free_keys) const {
    const double dx = static_cast<double>(ray.endpoint.x) - ray.origin.x;
    const double dy = static_cast<double>(ray.endpoint.y) - ray.origin.y;
    const double distance = std::hypot(dx, dy);
    if (!std::isfinite(distance) || distance <= params_.endpoint_margin) return;

    // Back away from both hit and no-hit endpoints. For hit rays this protects
    // the measured surface; for no-hit rays it avoids clearing across the local
    // box boundary after discretisation or TF noise.
    const double usable_distance = distance - params_.endpoint_margin;
    const double end_x = ray.origin.x + dx * usable_distance / distance;
    const double end_y = ray.origin.y + dy * usable_distance / distance;
    std::int64_t cell_x = static_cast<std::int64_t>(
        std::floor(ray.origin.x / params_.voxel_size));
    std::int64_t cell_y = static_cast<std::int64_t>(
        std::floor(ray.origin.y / params_.voxel_size));
    const std::int64_t end_cell_x = static_cast<std::int64_t>(
        std::floor(end_x / params_.voxel_size));
    const std::int64_t end_cell_y = static_cast<std::int64_t>(
        std::floor(end_y / params_.voxel_size));
    const std::int64_t origin_cell_x = cell_x;
    const std::int64_t origin_cell_y = cell_y;

    const int step_x = dx > 0.0 ? 1 : (dx < 0.0 ? -1 : 0);
    const int step_y = dy > 0.0 ? 1 : (dy < 0.0 ? -1 : 0);
    const double infinity = std::numeric_limits<double>::infinity();
    const double delta_x = step_x == 0 ? infinity :
        params_.voxel_size / std::abs(dx);
    const double delta_y = step_y == 0 ? infinity :
        params_.voxel_size / std::abs(dy);
    const double next_x = step_x > 0 ?
        (static_cast<double>(cell_x) + 1.0) * params_.voxel_size :
        static_cast<double>(cell_x) * params_.voxel_size;
    const double next_y = step_y > 0 ?
        (static_cast<double>(cell_y) + 1.0) * params_.voxel_size :
        static_cast<double>(cell_y) * params_.voxel_size;
    double max_x = step_x == 0 ? infinity :
        (next_x - ray.origin.x) / dx;
    double max_y = step_y == 0 ? infinity :
        (next_y - ray.origin.y) / dy;

    // A valid local ray is short, but keep a hard bound so malformed input
    // cannot turn this callback into an unbounded traversal.
    const std::size_t max_steps = static_cast<std::size_t>(
        std::ceil(usable_distance / params_.voxel_size) * 3.0 + 8.0);
    for (std::size_t step = 0u; step < max_steps; ++step) {
        if (cell_x != origin_cell_x || cell_y != origin_cell_y) {
            free_keys->insert(PackKey(cell_x, cell_y));
        }
        if (cell_x == end_cell_x && cell_y == end_cell_y) break;
        if (max_x < max_y) {
            cell_x += step_x;
            max_x += delta_x;
        } else if (max_y < max_x) {
            cell_y += step_y;
            max_y += delta_y;
        } else {
            cell_x += step_x;
            cell_y += step_y;
            max_x += delta_x;
            max_y += delta_y;
        }
    }
}

void ExternalAngularClearingFilter::Filter(
    const std::vector<ExternalAngularClearingRay>& rays,
    const PointCloudPtr& historical_obstacles,
    const PointCloudPtr& current_obstacles,
    const PointCloudPtr& protected_static_obstacles,
    const PointCloudPtr& candidates_out,
    const PointCloudPtr& confirmed_out,
    const PointCloudPtr& common_free_mask_out) {
    candidates_out->clear();
    confirmed_out->clear();
    common_free_mask_out->clear();
    ++frame_sequence_;

    std::unordered_set<std::uint64_t> layer_free[2];
    bool has_layer[2] = {false, false};
    for (const auto& ray : rays) {
        if (ray.layer > 1u) continue;
        has_layer[ray.layer] = true;
        RasterizeRay(ray, &layer_free[ray.layer]);
    }

    std::unordered_set<std::uint64_t> common_free;
    if (params_.require_both_layers) {
        if (has_layer[0] && has_layer[1]) {
            const auto* smaller = &layer_free[0];
            const auto* larger = &layer_free[1];
            if (smaller->size() > larger->size()) std::swap(smaller, larger);
            common_free.reserve(smaller->size());
            for (const auto key : *smaller) {
                if (larger->count(key) != 0u) common_free.insert(key);
            }
        }
    } else {
        common_free = std::move(layer_free[0]);
        common_free.insert(layer_free[1].begin(), layer_free[1].end());
    }

    common_free_mask_out->reserve(common_free.size());
    for (const auto key : common_free) {
        PCLPoint point;
        point.x = (static_cast<double>(KeyX(key)) + 0.5) * params_.voxel_size;
        point.y = (static_cast<double>(KeyY(key)) + 0.5) * params_.voxel_size;
        point.z = 0.0f;
        point.intensity = 0.0f;
        common_free_mask_out->push_back(point);
    }

    std::unordered_set<std::uint64_t> occupied_now;
    occupied_now.reserve(current_obstacles->size() +
                         protected_static_obstacles->size());
    for (const auto& point : current_obstacles->points) {
        occupied_now.insert(PointKey(point));
    }
    for (const auto& point : protected_static_obstacles->points) {
        occupied_now.insert(PointKey(point));
    }

    std::unordered_set<std::uint64_t> candidate_keys;
    candidate_keys.reserve(historical_obstacles->size());
    for (const auto& point : historical_obstacles->points) {
        const std::uint64_t key = PointKey(point);
        if (common_free.count(key) == 0u || occupied_now.count(key) != 0u) {
            continue;
        }
        candidates_out->push_back(point);  // Preserve the exact historical coordinate.
        candidate_keys.insert(key);
    }

    std::unordered_set<std::uint64_t> confirmed_keys;
    confirmed_keys.reserve(candidate_keys.size());
    for (const auto key : candidate_keys) {
        VoteState& state = votes_[key];
        state.count = state.last_frame + 1u == frame_sequence_ ?
            state.count + 1 : 1;
        state.last_frame = frame_sequence_;
        if (state.count >= params_.confirmations) confirmed_keys.insert(key);
    }

    // Missing evidence breaks consecutiveness. This is observation-count based
    // clearing, not a wall-clock expiry policy.
    for (auto it = votes_.begin(); it != votes_.end();) {
        if (it->second.last_frame != frame_sequence_ ||
            confirmed_keys.count(it->first) != 0u) {
            it = votes_.erase(it);
        } else {
            ++it;
        }
    }

    confirmed_out->reserve(candidates_out->size());
    for (const auto& point : candidates_out->points) {
        if (confirmed_keys.count(PointKey(point)) != 0u) {
            confirmed_out->push_back(point);
        }
    }
}
