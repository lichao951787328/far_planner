/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */

#include "far_planner/contour_detector.h"

// const static int BLUR_SIZE = 10;

/***************************************************************************************/

void ContourDetector::Init(const ContourDetectParams& params) {
    cd_params_ = params;
    /* Allocate Pointcloud pointer memory */
    new_corners_cloud_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    // Init projection cv Mat
    MAT_SIZE = std::ceil(cd_params_.sensor_range * 2.0f /
                         cd_params_.contour_grid_resolution);
    if (MAT_SIZE % 2 == 0) MAT_SIZE ++;
    MAT_RESIZE = MAT_SIZE * (int)cd_params_.kRatio;
    CMAT = MAT_SIZE / 2, CMAT_RESIZE = MAT_RESIZE / 2;
    img_mat_ = cv::Mat::zeros(MAT_SIZE, MAT_SIZE, CV_32FC1);
    debug_base_img_.release();
    debug_processed_img_.release();
    topology_img_.release();
    configuration_space_img_.release();
    img_counter_ = 0;
    odom_node_ptr_ = NULL;
    refined_contours_.clear(), refined_hierarchy_.clear();
    dense_contours_.clear(), dense_world_contours_.clear();
    simplified_dense_indices_.clear();
    DIST_LIMIT = cd_params_.kRatio * 1.5f;
    // Match upstream FAR's RemoveWallConnection tolerance. kAcceptAlign has
    // already been converted from degrees to radians by LoadROSParams().
    ALIGN_ANGLE_COS = std::cos(FARUtil::kAcceptAlign / 2.0f);
    VOXEL_DIM_INV = 1.0f / cd_params_.contour_grid_resolution;
}

// odom_node_ptr 决定局部裁剪窗口，但栅格原点会对齐到 map_start 下
// 固定的 contour_grid_resolution 网格。这样机器人连续运动时，已有障碍
// 不会因为投影原点每帧变化而产生半个栅格以内的锯齿跳动。
// free_odom_resized_ 仍使用真实 odom/free-odom 位置，保证内外轮廓判断正确。
// 在同一张局部障碍基础图上分别生成 FAR 风格拓扑图和欧氏配置空间图；只有
// 前者进入 findContours / approxPolyDP，后者留给实际 edge/waypoint 碰撞检查。
void ContourDetector::BuildTerrainImgAndExtractContour(const NavNodePtr& odom_node_ptr,
                                                       const PointCloudPtr& surround_cloud,
                                                       std::vector<PointStack>& realworl_contour,
                                                       const bool& is_verified_occupied,
                                                       const float simplify_ratio) {
    CVPointStack cv_corners;
    PointStack corner_vec;
    this->UpdateOdom(odom_node_ptr);
    this->ResetImgMat(img_mat_);
    this->UpdateImgMatWithCloud(surround_cloud, img_mat_,
                                is_verified_occupied);
    debug_base_img_ = img_mat_.clone();
    this->ExtractContourFromImg(img_mat_, refined_contours_, realworl_contour,
                                std::max(1.0f, simplify_ratio));
}

void ContourDetector::UpdateImgMatWithCloud(
    const PointCloudPtr& pc,
    cv::Mat& img_mat,
    const bool& is_verified_occupied) {
    int row_idx, col_idx;
    for (const auto& pcl_p : pc->points) {
        this->PointToImgSub(
            pcl_p, raster_center_, row_idx, col_idx, false, false);
        if (!this->IsIdxesInImg(row_idx, col_idx)) continue;
        // Rasterize the measured occupied voxel only.  Robot clearance is
        // applied once, in metric units, by BuildConfigurationSpaceImg().
        // The old 3x3 write followed by interpolation/blur produced an
        // implicit and resolution-dependent extra dilation.
        img_mat.at<float>(row_idx, col_idx) += 1.0f;
    }
    if (is_verified_occupied) {
        // Semantic-octomap points have already passed occupancy and class
        // validation.  Requiring several points to hit the same projected
        // pixel is an old raw-scan denoising rule; it erases sparse or
        // single-height occupied voxels, especially small dynamic objects.
        cv::threshold(img_mat, img_mat, 0.0, 1.0,
                      cv::ThresholdTypes::THRESH_BINARY);
    } else if (!FARUtil::IsStaticEnv) {
        cv::threshold(img_mat, img_mat, cd_params_.kThredValue, 1.0, cv::ThresholdTypes::THRESH_BINARY);
    }
    if (cd_params_.is_save_img) this->SaveCurrentImg(img_mat);
}

void ContourDetector::BuildConfigurationSpaceImg(const cv::Mat& img,
                                                 cv::Mat& Rimg) {
    cv::Mat base_occupied;
    cv::threshold(img, base_occupied, 0.0, 255.0, cv::THRESH_BINARY);
    base_occupied.convertTo(base_occupied, CV_8UC1);

    // Refinement changes only coordinate precision.  Preserve every occupied
    // base-cell centre as exactly one refined seed; nearest-neighbour block
    // replication would first turn one measurement into a kRatio-by-kRatio
    // square and silently add another half base cell to the requested metric
    // clearance.
    const int ratio = std::max(1, static_cast<int>(cd_params_.kRatio));
    cv::Mat refined_seeds = cv::Mat::zeros(
        base_occupied.rows * ratio, base_occupied.cols * ratio, CV_8UC1);
    const int base_center_row = base_occupied.rows / 2;
    const int base_center_col = base_occupied.cols / 2;
    const int refined_center_row = refined_seeds.rows / 2;
    const int refined_center_col = refined_seeds.cols / 2;
    for (int row = 0; row < base_occupied.rows; ++row) {
        const std::uint8_t* source = base_occupied.ptr<std::uint8_t>(row);
        for (int col = 0; col < base_occupied.cols; ++col) {
            if (source[col] == 0) continue;
            const int refined_row = refined_center_row +
                (row - base_center_row) * ratio;
            const int refined_col = refined_center_col +
                (col - base_center_col) * ratio;
            if (refined_row >= 0 && refined_row < refined_seeds.rows &&
                refined_col >= 0 && refined_col < refined_seeds.cols) {
                refined_seeds.at<std::uint8_t>(refined_row, refined_col) = 255;
            }
        }
    }
    if (cv::countNonZero(refined_seeds) == 0) {
        Rimg = cv::Mat::zeros(refined_seeds.size(), CV_8UC1);
        return;
    }

    const float refined_resolution =
        cd_params_.contour_grid_resolution / cd_params_.kRatio;
    const float clearance_pixels =
        std::max(0.0f, cd_params_.configuration_space_clearance) /
        refined_resolution;
    if (clearance_pixels <= FARUtil::kEpsilon) {
        Rimg = refined_seeds;
        return;
    }

    // distanceTransform measures every free pixel to the nearest occupied
    // pixel (occupied is encoded as zero in distance_input).  Thresholding in
    // metres creates a Euclidean configuration-space obstacle rather than a
    // square morphology kernel.
    cv::Mat distance_input;
    cv::bitwise_not(refined_seeds, distance_input);
    cv::Mat distance_pixels;
    cv::distanceTransform(distance_input, distance_pixels,
                          cv::DIST_L2, cv::DIST_MASK_PRECISE);
    cv::compare(distance_pixels, clearance_pixels, Rimg, cv::CMP_LE);
}

void ContourDetector::BuildFarTopologyImg(const cv::Mat& img,
                                          cv::Mat& Rimg) {
    cv::Mat occupied;
    cv::threshold(img, occupied, 0.0, 255.0, cv::THRESH_BINARY);
    occupied.convertTo(occupied, CV_8UC1);

    // Reproduce FAR's 3x3 point write without contaminating the independent
    // robot-centre configuration-space image.  For the verified local-voxel
    // snapshots used by this fork, dilating the binary occupied cells is
    // equivalent to writing each occupied voxel into its 3x3 neighbourhood.
    cv::dilate(occupied, occupied,
               cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    cv::resize(occupied, Rimg, cv::Size(), cd_params_.kRatio,
               cd_params_.kRatio, cv::INTER_LINEAR);

    const int blur_size = std::max(1, cd_params_.topology_blur_size);
    if (blur_size > 1) {
        // FAR uses normalize=false.  On CV_8UC1 the neighbourhood sum
        // saturates, preserving every non-zero interpolated topology pixel.
        cv::boxFilter(Rimg, Rimg, -1, cv::Size(blur_size, blur_size),
                      cv::Point(-1, -1), false);
    }
}

void ContourDetector::ExtractContourFromImg(const cv::Mat& img,
                                            std::vector<CVPointStack>& img_contours, 
                                            std::vector<PointStack>& realworld_contour,
                                            const float simplify_ratio)
{
    // Keep topology and execution safety as two independent representations.
    // Topology extraction follows FAR's sparse TC89/RDP chain.  The separate
    // configuration-space image remains available only to later executable
    // edge and waypoint collision checks.
    this->BuildFarTopologyImg(img, topology_img_);
    this->BuildConfigurationSpaceImg(img, configuration_space_img_);
    this->ExtractRefinedContours(topology_img_, img_contours,
                                 DIST_LIMIT * simplify_ratio);
    debug_processed_img_ = topology_img_.clone();
    if (!img_contours.empty()) {
        std::vector<std::vector<cv::Point>> rounded;
        rounded.reserve(img_contours.size());
        for (const auto& contour : img_contours) {
            std::vector<cv::Point> points;
            points.reserve(contour.size());
            for (const auto& point : contour) points.emplace_back(point);
            rounded.push_back(std::move(points));
        }
        cv::Mat overlay;
        cv::cvtColor(debug_processed_img_, overlay, cv::COLOR_GRAY2BGR);
        cv::drawContours(overlay, rounded, -1, cv::Scalar(0, 255, 255), 1,
                         cv::LINE_8);
        debug_processed_img_ = overlay;
    }
    this->ConvertContoursToRealWorld(img_contours, realworld_contour);
}

void ContourDetector::ConvertContoursToRealWorld(const std::vector<CVPointStack>& ori_contours,
                                                 std::vector<PointStack>& realWorld_contours)
{
    const std::size_t C_N = ori_contours.size();
    realWorld_contours.clear(), realWorld_contours.resize(C_N);
    dense_world_contours_.clear(), dense_world_contours_.resize(C_N);
    simplified_dense_indices_.clear();
    simplified_dense_indices_.resize(C_N);
    for (std::size_t i=0; i<C_N; i++) {
        const CVPointStack cv_contour = ori_contours[i];
        this->ConvertCVToPoint3DVector(cv_contour, realWorld_contours[i], true);
        if (i < dense_contours_.size()) {
            this->ConvertCVToPoint3DVector(
                dense_contours_[i], dense_world_contours_[i], true);
        }

        // The RDP vertices originate on the ordered TC89 source chain. Keep a
        // compatibility correspondence for the current Graph implementation;
        // the next Graph step will stop interpreting it as a robot trajectory.
        auto& correspondence = simplified_dense_indices_[i];
        correspondence.reserve(realWorld_contours[i].size());
        for (const Point3D& vertex : realWorld_contours[i]) {
            std::size_t best_index = 0;
            float best_distance = FARUtil::kINF;
            for (std::size_t dense_index = 0;
                 dense_index < dense_world_contours_[i].size();
                 ++dense_index) {
                const float distance =
                    (vertex - dense_world_contours_[i][dense_index])
                        .norm_flat();
                if (distance < best_distance) {
                    best_distance = distance;
                    best_index = dense_index;
                }
            }
            correspondence.push_back(best_index);
        }
    }
}


void ContourDetector::ShowCornerImage(const cv::Mat& img_mat,
                                     const PointCloudPtr& pc) {
    cv::Mat dst = cv::Mat::zeros(MAT_RESIZE, MAT_RESIZE, CV_8UC3);
    const int circle_size = (int)(cd_params_.kRatio*1.5);
    for (std::size_t i=0; i<pc->size(); i++) {
        cv::Point2f cv_p = this->ConvertPoint3DToCVPoint(
            pc->points[i], raster_center_, true);
        cv::circle(dst, cv_p, circle_size, cv::Scalar(128,128,128), -1);

    }
    // show free odom point
    cv::circle(dst, free_odom_resized_, circle_size, cv::Scalar(0,0,255), -1);
    std::vector<std::vector<cv::Point2i>> round_contours;
    this->RoundContours(refined_contours_, round_contours);
    for(std::size_t idx=0; idx<round_contours.size(); idx++) {
        cv::Scalar color(rand()&255, rand()&255, rand()&255 );
        cv::drawContours(dst, round_contours, idx, color, cv::LineTypes::LINE_4);
    }
    cv::imshow("Obstacle Cloud Image", dst);
    cv::waitKey(30);
}

void ContourDetector::ExtractRefinedContours(const cv::Mat& imgIn,
                                            std::vector<CVPointStack>& refined_contours,
                                            const float distance_limit)
{ 

    std::vector<std::vector<cv::Point2i>> raw_contours;
    refined_contours.clear(), refined_hierarchy_.clear();
    dense_contours_.clear();
    // Match upstream FAR: TC89_L1 first removes raster stair steps, then RDP,
    // topology filtering and adjacent-distance filtering produce the contour
    // vertices used by the visibility graph.
    cv::Mat contour_input = imgIn.clone();
    cv::findContours(contour_input, raw_contours, refined_hierarchy_,
                     cv::RetrievalModes::RETR_TREE, 
                     cv::ContourApproximationModes::CHAIN_APPROX_TC89_L1);

    dense_contours_.resize(raw_contours.size());
    for (std::size_t i = 0; i < raw_contours.size(); ++i) {
        dense_contours_[i].reserve(raw_contours[i].size());
        for (const cv::Point2i& point : raw_contours[i]) {
            dense_contours_[i].emplace_back(point);
        }
    }
                     
    refined_contours.resize(raw_contours.size());
    for (std::size_t i=0; i<raw_contours.size(); i++) {
        // using Ramer–Douglas–Peucker algorithm url: https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
        cv::approxPolyDP(raw_contours[i], refined_contours[i],
                         distance_limit, true);
    }
    this->TopoFilterContours(refined_contours, dense_contours_);
    this->AdjecentDistanceFilter(
        refined_contours, dense_contours_, distance_limit);
}

void ContourDetector::AdjecentDistanceFilter(
    std::vector<CVPointStack>& contoursInOut,
    std::vector<CVPointStack>& denseContoursInOut,
    const float distance_limit) {
    /* filter out vertices that are overlapped with neighbor */
    std::unordered_set<int> remove_idxs;
    for (std::size_t i=0; i<contoursInOut.size(); i++) { 
        const auto c = contoursInOut[i];
        const std::size_t c_size = c.size();
        std::size_t refined_idx = 0;
        for (std::size_t j=0; j<c_size; j++) {
            cv::Point2f p = c[j]; 
            if (refined_idx < 1 ||
                FARUtil::PixelDistance(contoursInOut[i][refined_idx-1], p) >
                    distance_limit) {
                /** Reduce wall nodes */
                RemoveWallConnection(contoursInOut[i], p, refined_idx);
                contoursInOut[i][refined_idx] = p;
                refined_idx ++;
            }
        }
        /** Reduce wall nodes */
        RemoveWallConnection(contoursInOut[i], contoursInOut[i][0], refined_idx);
        contoursInOut[i].resize(refined_idx);
        if (refined_idx > 1 &&
            FARUtil::PixelDistance(contoursInOut[i].front(),
                                   contoursInOut[i].back()) < distance_limit) {
            contoursInOut[i].pop_back();
        }
        if (contoursInOut[i].size() < 3) remove_idxs.insert(i);
    }
    if (!remove_idxs.empty()) { // clear contour with vertices size less that 3
        std::vector<CVPointStack> temp_contours = contoursInOut;
        std::vector<CVPointStack> temp_dense = denseContoursInOut;
        contoursInOut.clear();
        denseContoursInOut.clear();
        for (int i=0; i<temp_contours.size(); i++) {
            if (remove_idxs.find(i) != remove_idxs.end()) continue;
            contoursInOut.push_back(temp_contours[i]);
            if (i < temp_dense.size()) {
                denseContoursInOut.push_back(temp_dense[i]);
            }
        }
    }
}

void ContourDetector::TopoFilterContours(
    std::vector<CVPointStack>& contoursInOut,
    std::vector<CVPointStack>& denseContoursInOut) {
    std::unordered_set<int> remove_idxs;
    for (int i=0; i<contoursInOut.size(); i++) {
        if (remove_idxs.find(i) != remove_idxs.end()) continue;
        const auto poly = contoursInOut[i];
        if (poly.size() < 3) {
            remove_idxs.insert(i);
        } else if (!FARUtil::PointInsideAPoly(poly, free_odom_resized_)) {
            InternalContoursIdxs(refined_hierarchy_, i, remove_idxs);
        }
    }
    if (!remove_idxs.empty()) {
        std::vector<CVPointStack> temp_contours = contoursInOut;
        std::vector<CVPointStack> temp_dense = denseContoursInOut;
        contoursInOut.clear();
        denseContoursInOut.clear();
        for (int i=0; i<temp_contours.size(); i++) {
            if (remove_idxs.find(i) != remove_idxs.end()) continue;
            contoursInOut.push_back(temp_contours[i]);
            if (i < temp_dense.size()) {
                denseContoursInOut.push_back(temp_dense[i]);
            }
        }
    }
}
