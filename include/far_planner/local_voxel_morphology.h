#ifndef FAR_PLANNER_LOCAL_VOXEL_MORPHOLOGY_H
#define FAR_PLANNER_LOCAL_VOXEL_MORPHOLOGY_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

struct LocalVoxelMorphologyResult {
    // One entry per input 3D point. A point is retained only when its original
    // XY pixel is still occupied after opening; pixels created only by the
    // dilation phase never invent new 3D points.
    std::vector<std::uint8_t> keep;
    std::size_t input_pixels = 0;
    std::size_t retained_input_pixels = 0;
    std::size_t removed_input_pixels = 0;
    std::size_t removed_points = 0;
    int radius_cells = 0;
    bool applied = false;
};

/**
 * Project one semantic layer to a fine source-frame XY raster and apply
 * metric morphological opening (erosion followed by dilation). Each input
 * point represents one source voxel footprint rather than one fine-raster
 * pixel; this prevents a 0.10 m voxel cloud from becoming a set of isolated
 * pixels merely because the requested morphology raster is 0.01 m.
 *
 * Layers are processed independently by the caller so a dense terrain pixel
 * cannot preserve an isolated obstacle/dynamic pixel at the same XY. Every
 * surviving XY pixel maps back to all original height samples in that cell.
 * Invalid parameters and pathologically large extents fail open, retaining
 * the snapshot instead of silently deleting collision geometry.
 */
inline LocalVoxelMorphologyResult ComputeLocalVoxelMorphologyOpening(
    const std::vector<cv::Point2f>& source_xy, const float source_resolution,
    const float raster_resolution, const float opening_radius,
    const bool enabled = true,
    const std::size_t maximum_raster_cells = 4000000u) {
    LocalVoxelMorphologyResult result;
    result.keep.assign(source_xy.size(), 1u);
    if (!enabled || source_xy.empty() ||
        !std::isfinite(source_resolution) ||
        !std::isfinite(raster_resolution) ||
        !std::isfinite(opening_radius) ||
        source_resolution <= 0.0f || raster_resolution <= 0.0f ||
        opening_radius <= 0.0f) {
        return result;
    }

    result.radius_cells = std::max(
        1, static_cast<int>(std::ceil(
               opening_radius / raster_resolution - 1e-6f)));
    std::vector<std::int64_t> center_cell_x(source_xy.size());
    std::vector<std::int64_t> center_cell_y(source_xy.size());
    std::vector<std::int64_t> footprint_min_x(source_xy.size());
    std::vector<std::int64_t> footprint_max_x(source_xy.size());
    std::vector<std::int64_t> footprint_min_y(source_xy.size());
    std::vector<std::int64_t> footprint_max_y(source_xy.size());
    std::int64_t minimum_x = std::numeric_limits<std::int64_t>::max();
    std::int64_t maximum_x = std::numeric_limits<std::int64_t>::lowest();
    std::int64_t minimum_y = std::numeric_limits<std::int64_t>::max();
    std::int64_t maximum_y = std::numeric_limits<std::int64_t>::lowest();
    const double half_source = 0.5 * source_resolution;
    const auto footprint_min = [raster_resolution](const double value) {
        return static_cast<std::int64_t>(std::ceil(
            value / raster_resolution - 1e-6));
    };
    const auto footprint_max = [raster_resolution](const double value) {
        return static_cast<std::int64_t>(std::ceil(
            value / raster_resolution - 1e-6)) - 1;
    };
    for (std::size_t index = 0; index < source_xy.size(); ++index) {
        if (!std::isfinite(source_xy[index].x) ||
            !std::isfinite(source_xy[index].y)) {
            return result;
        }
        center_cell_x[index] = static_cast<std::int64_t>(
            std::llround(source_xy[index].x / raster_resolution));
        center_cell_y[index] = static_cast<std::int64_t>(
            std::llround(source_xy[index].y / raster_resolution));
        footprint_min_x[index] = footprint_min(
            source_xy[index].x - half_source);
        footprint_max_x[index] = footprint_max(
            source_xy[index].x + half_source);
        footprint_min_y[index] = footprint_min(
            source_xy[index].y - half_source);
        footprint_max_y[index] = footprint_max(
            source_xy[index].y + half_source);
        // A source voxel smaller than one raster pixel still owns the pixel
        // containing its centre.
        footprint_min_x[index] = std::min(
            footprint_min_x[index], center_cell_x[index]);
        footprint_max_x[index] = std::max(
            footprint_max_x[index], center_cell_x[index]);
        footprint_min_y[index] = std::min(
            footprint_min_y[index], center_cell_y[index]);
        footprint_max_y[index] = std::max(
            footprint_max_y[index], center_cell_y[index]);
        minimum_x = std::min(minimum_x, footprint_min_x[index]);
        maximum_x = std::max(maximum_x, footprint_max_x[index]);
        minimum_y = std::min(minimum_y, footprint_min_y[index]);
        maximum_y = std::max(maximum_y, footprint_max_y[index]);
    }

    const std::int64_t padding = result.radius_cells + 1;
    const std::int64_t row_count =
        maximum_x - minimum_x + 1 + 2 * padding;
    const std::int64_t column_count =
        maximum_y - minimum_y + 1 + 2 * padding;
    if (row_count <= 0 || column_count <= 0 ||
        row_count > std::numeric_limits<int>::max() ||
        column_count > std::numeric_limits<int>::max() ||
        static_cast<std::size_t>(row_count) >
            maximum_raster_cells /
                std::max<std::size_t>(1u,
                    static_cast<std::size_t>(column_count))) {
        return result;
    }

    cv::Mat occupied = cv::Mat::zeros(
        static_cast<int>(row_count), static_cast<int>(column_count), CV_8UC1);
    for (std::size_t index = 0; index < source_xy.size(); ++index) {
        const int first_row = static_cast<int>(
            footprint_min_x[index] - minimum_x + padding);
        const int last_row = static_cast<int>(
            footprint_max_x[index] - minimum_x + padding);
        const int first_column = static_cast<int>(
            footprint_min_y[index] - minimum_y + padding);
        const int last_column = static_cast<int>(
            footprint_max_y[index] - minimum_y + padding);
        occupied(cv::Range(first_row, last_row + 1),
                 cv::Range(first_column, last_column + 1)).setTo(255u);
    }
    result.input_pixels = static_cast<std::size_t>(cv::countNonZero(occupied));

    const int kernel_size = result.radius_cells * 2 + 1;
    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
    cv::Mat opened;
    cv::morphologyEx(occupied, opened, cv::MORPH_OPEN, kernel);

    cv::Mat retained_pixels;
    cv::bitwise_and(occupied, opened, retained_pixels);
    result.retained_input_pixels =
        static_cast<std::size_t>(cv::countNonZero(retained_pixels));
    result.removed_input_pixels =
        result.input_pixels - result.retained_input_pixels;
    result.keep.assign(source_xy.size(), 0u);
    for (std::size_t index = 0; index < source_xy.size(); ++index) {
        const int row = static_cast<int>(
            center_cell_x[index] - minimum_x + padding);
        const int column =
            static_cast<int>(center_cell_y[index] - minimum_y + padding);
        if (opened.at<std::uint8_t>(row, column) != 0u) {
            result.keep[index] = 1u;
        } else {
            ++result.removed_points;
        }
    }
    result.applied = true;
    return result;
}

#endif  // FAR_PLANNER_LOCAL_VOXEL_MORPHOLOGY_H
