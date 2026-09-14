#ifndef FAR_PLANNER_LOCAL_OBSERVATION_WINDOW_H
#define FAR_PLANNER_LOCAL_OBSERVATION_WINDOW_H

#include <algorithm>
#include <cmath>

#include "far_planner/point_struct.h"

/**
 * Robot-relative 2-D footprint of one complete local voxel snapshot.
 *
 * The upstream map may use an asymmetric box which rotates with the robot.
 * Treating it as FAR's old world-aligned sensor square makes cropped wall ends
 * look physical and permits history outside the snapshot to be revalidated.
 */
struct LocalObservationWindow2D {
    bool enabled = false;
    float min_x = 0.0f;
    float max_x = 0.0f;
    float min_y = 0.0f;
    float max_y = 0.0f;
    float guard = 0.0f;
    Point3D origin = Point3D(0.0f, 0.0f, 0.0f);
    Point3D forward = Point3D(1.0f, 0.0f, 0.0f);

    bool IsValid() const {
        return enabled && min_x + guard < max_x - guard &&
               min_y + guard < max_y - guard;
    }

    Point3D ToLocal(const Point3D& point) const {
        const float norm = std::hypot(forward.x, forward.y);
        const float fx = norm > EPSILON ? forward.x / norm : 1.0f;
        const float fy = norm > EPSILON ? forward.y / norm : 0.0f;
        const float dx = point.x - origin.x;
        const float dy = point.y - origin.y;
        return Point3D(dx * fx + dy * fy, -dx * fy + dy * fx,
                       point.z - origin.z);
    }

    bool Contains(const Point3D& point) const {
        if (!IsValid()) return false;
        const Point3D local = ToLocal(point);
        return local.x >= min_x + guard && local.x <= max_x - guard &&
               local.y >= min_y + guard && local.y <= max_y - guard;
    }

    /** Complete upstream voxel footprint, including the guard band. */
    bool ContainsFull(const Point3D& point,
                      const float tolerance = 0.0f) const {
        if (!IsValid()) return false;
        const Point3D local = ToLocal(point);
        const float margin = std::max(0.0f, tolerance);
        return local.x >= min_x - margin && local.x <= max_x + margin &&
               local.y >= min_y - margin && local.y <= max_y + margin;
    }

    bool SegmentFullyContained(const Point3D& start,
                               const Point3D& end) const {
        // A rectangle is convex, so endpoint containment proves containment
        // of the complete segment.
        return Contains(start) && Contains(end);
    }

    bool SegmentFullyContainedFull(const Point3D& start,
                                   const Point3D& end,
                                   const float tolerance = 0.0f) const {
        return ContainsFull(start, tolerance) &&
               ContainsFull(end, tolerance);
    }

    bool SegmentIntersects(const Point3D& start,
                           const Point3D& end) const {
        if (!IsValid()) return false;
        const Point3D local_start = ToLocal(start);
        const Point3D local_end = ToLocal(end);
        const float lower_x = min_x + guard;
        const float upper_x = max_x - guard;
        const float lower_y = min_y + guard;
        const float upper_y = max_y - guard;
        float lower = 0.0f;
        float upper = 1.0f;
        const auto clip_axis = [&lower, &upper](
            const float coordinate, const float direction,
            const float minimum, const float maximum) {
            if (std::abs(direction) <= EPSILON) {
                return coordinate >= minimum && coordinate <= maximum;
            }
            float first = (minimum - coordinate) / direction;
            float second = (maximum - coordinate) / direction;
            if (first > second) std::swap(first, second);
            lower = std::max(lower, first);
            upper = std::min(upper, second);
            return lower <= upper;
        };
        return clip_axis(local_start.x, local_end.x - local_start.x,
                         lower_x, upper_x) &&
               clip_axis(local_start.y, local_end.y - local_start.y,
                         lower_y, upper_y);
    }
};

#endif  // FAR_PLANNER_LOCAL_OBSERVATION_WINDOW_H
