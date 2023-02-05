#pragma once

#ifndef MYVIOSLAM_FEATURE_H
#define MYVIOSLAM_FEATURE_H

#include "common_include.h"
#include "frame.h"
#include "mappoint.h"


namespace myvioslam {

struct Frame;
struct MapPoint;

struct Feature {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame_;            // the frame that takes this feature
    cv::KeyPoint position_;                 // 2d pixel location of the feature point
    std::weak_ptr<MapPoint> map_point_;     // assigned map point

    bool is_outlier_ = false;               // check if outlier point
    bool is_on_left_img_ = true;            // was feature detected on left img?

    Feature() {}

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
        : frame_(frame), position_(kp) {}
};

}

#endif