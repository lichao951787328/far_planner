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
    img_counter_ = 0;
    odom_node_ptr_ = NULL;
    refined_contours_.clear(), refined_hierarchy_.clear();
    DIST_LIMIT = cd_params_.kRatio * 1.5f;
    ALIGN_ANGLE_COS = std::cos(
        std::max(0.0f, cd_params_.collinear_angle_deg) *
        static_cast<float>(M_PI) / 180.0f);
    VOXEL_DIM_INV = 1.0f / cd_params_.contour_grid_resolution;
}

// odom_node_ptr 决定局部裁剪窗口，但栅格原点会对齐到 map_start 下
// 固定的 contour_grid_resolution 网格。这样机器人连续运动时，已有障碍
// 不会因为投影原点每帧变化而产生半个栅格以内的锯齿跳动。
// free_odom_resized_ 仍使用真实 odom/free-odom 位置，保证内外轮廓判断正确。
// 在这张局部障碍图像上提取轮廓
// 然后做 threshold / blur / findContours / approxPolyDP，最后转回 realworld_contour。
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
    this->ExtractContourFromImg(img_mat_, refined_contours_, realworl_contour,
                                std::max(1.0f, simplify_ratio));
}

void ContourDetector::UpdateImgMatWithCloud(
    const PointCloudPtr& pc,
    cv::Mat& img_mat,
    const bool& is_verified_occupied) {
    int row_idx, col_idx, inf_row, inf_col;
    const std::vector<int> inflate_vec{-1, 0, 1};
    for (const auto& pcl_p : pc->points) {
        this->PointToImgSub(
            pcl_p, raster_center_, row_idx, col_idx, false, false);
        if (!this->IsIdxesInImg(row_idx, col_idx)) continue;
        for (const auto& dr : inflate_vec) {
            for (const auto& dc : inflate_vec) {
                inf_row = row_idx+dr, inf_col = col_idx+dc;
                if (this->IsIdxesInImg(inf_row, inf_col)) {
                    img_mat.at<float>(inf_row, inf_col) += 1.0;
                }
            }
        }
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

void ContourDetector::ResizeAndBlurImg(const cv::Mat& img, cv::Mat& Rimg) {
    img.convertTo(Rimg, CV_8UC1, 255);
    cv::resize(Rimg, Rimg, cv::Size(), cd_params_.kRatio, cd_params_.kRatio, 
               cv::InterpolationFlags::INTER_LINEAR);
    //cv::morphologyEx(Rimg, Rimg, cv::MORPH_OPEN, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    cv::boxFilter(Rimg, Rimg, -1, cv::Size(cd_params_.kBlurSize, cd_params_.kBlurSize), cv::Point2i(-1, -1), false);
    //cv::morphologyEx(Rimg, Rimg, cv::MORPH_CLOSE, getStructuringElement(cv::MORPH_RECT, cv::Size(cd_params_.kBlurSize+2, cd_params_.kBlurSize+2)));
}

void ContourDetector::ExtractContourFromImg(const cv::Mat& img,
                                            std::vector<CVPointStack>& img_contours, 
                                            std::vector<PointStack>& realworld_contour,
                                            const float simplify_ratio)
{
    cv::Mat Rimg;
    this->ResizeAndBlurImg(img, Rimg);
    this->ExtractRefinedContours(Rimg, img_contours,
                                 DIST_LIMIT * simplify_ratio);
    this->ConvertContoursToRealWorld(img_contours, realworld_contour);
}

void ContourDetector::ConvertContoursToRealWorld(const std::vector<CVPointStack>& ori_contours,
                                                 std::vector<PointStack>& realWorld_contours)
{
    const std::size_t C_N = ori_contours.size();
    realWorld_contours.clear(), realWorld_contours.resize(C_N);
    for (std::size_t i=0; i<C_N; i++) {
        const CVPointStack cv_contour = ori_contours[i];
        this->ConvertCVToPoint3DVector(cv_contour, realWorld_contours[i], true);
        SimplifyClosedContourCollinearVertices(
            realWorld_contours[i], cd_params_.collinear_tolerance,
            cd_params_.collinear_angle_deg);
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
    cv::findContours(imgIn, raw_contours, refined_hierarchy_, 
                     cv::RetrievalModes::RETR_TREE, 
                     cv::ContourApproximationModes::CHAIN_APPROX_TC89_L1);
                     
    refined_contours.resize(raw_contours.size());
    for (std::size_t i=0; i<raw_contours.size(); i++) {
        // using Ramer–Douglas–Peucker algorithm url: https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
        cv::approxPolyDP(raw_contours[i], refined_contours[i],
                         distance_limit, true);
    }
    this->TopoFilterContours(refined_contours); 
    this->AdjecentDistanceFilter(refined_contours, distance_limit);
}

void ContourDetector::AdjecentDistanceFilter(
    std::vector<CVPointStack>& contoursInOut,
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
        contoursInOut.clear();
        for (int i=0; i<temp_contours.size(); i++) {
            if (remove_idxs.find(i) != remove_idxs.end()) continue;
            contoursInOut.push_back(temp_contours[i]);
        }
    }
}

void ContourDetector::TopoFilterContours(std::vector<CVPointStack>& contoursInOut) {
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
        contoursInOut.clear();
        for (int i=0; i<temp_contours.size(); i++) {
            if (remove_idxs.find(i) != remove_idxs.end()) continue;
            contoursInOut.push_back(temp_contours[i]);
        }
    }
}
