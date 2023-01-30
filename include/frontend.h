#pragma once
#ifndef MYVIOSLAM_FRONTEND_H
#define MYVIOSLAM_FRONTNED_H

#include "common_include.h"
#include <opencv2/features2d.hpp>

#include "frame.h"
#include "map.h"

namespace myvioslam {

class Backend;
class Viewer;

enum class FrontendStatus {INITING, TRACKING_GOOD, TRACKING_BAD, LOST};

class Frontend {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frontend> Ptr;

    Frontend();

    bool AddFrame(Frame::Ptr frame);

    void SetMap(Map::Ptr map) {map_ = map;}

    void SetBackend(std::shared_ptr<Backend> backend) {backend_ = backend;}

    void SetViewer(std::shared_ptr<Viewer> viewer) {viewer_ = viewer;}

    FrontendStatus GetStatus() const {return status_;}

    void SetCameras(Camera::Ptr left, Camera::Ptr right) {
        camera_left_ = left;
        camera_right_ = right;
    }

private:
    /**
     * @brief Track in normal mode
     * @return true if successful
     */
    bool Track();

    /**
     * @brief Reset frontend when lost
     * @return true if success
     */
    bool Reset();

    /**
     * @brief Track based on last frame
     * @return Number of tracked points
     */
    int TrackLastFrame();

    /**
     * @brief Estimate the current frame's pose
     * @return number of valid inliers
     */
    int EstimateCurrentPose();

    /**
     * @brief Set current frame as a keyframe and insert to backend
     * @return true if success
     */
    bool InsertKeyFrame();

    /**
     * @brief Initialize frontend with stereo images saved in current_frame_
     * @return true if success
     */
    bool InitStereo();

    int DetectFeatures();

    /**
     * @brief Find the corresponding features in right img of current_frame_
     * 
     * @return int number of corespondences
     */
    int FindFeaturesRight();

    /**
     * @brief build initial map from single image
     * 
     * @return true if successful
     */
    bool BuildInitMap();

    /**
     * @brief Triangulate 3d position of 2d feature points
     * 
     * @return int num points
     */
    int TriangulatePoints();

    /**
     * @brief Set the features in keyframe as new observations for particular map point
     * (remember that many features might map to a single landmark/map point)
     * 
     */
    void SetObservationsForKeyFrame();

private:
    // data fields
    FrontendStatus status_ = FrontendStatus::INITING;

    Frame::Ptr current_frame_ = nullptr;
    Frame::Ptr last_frame_ = nullptr;
    Camera::Ptr camera_left_ = nullptr;
    Camera::Ptr camera_right_ = nullptr;

    Map::Ptr map_ = nullptr;
    std::shared_ptr<Backend> backend_ = nullptr;
    std::shared_ptr<Viewer> viewer_ = nullptr;

    SE3 relative_motion_;       // relative transformation from last frame to the current frame

    int tracking_inliers_ = 0;

    // tracking thresholds TODO convert into config file params
    int num_features_ = 200;
    int num_features_init_ = 100;
    int num_features_tracking_ = 50;
    int num_features_tracking_bad_ = 20;
    int num_features_needed_for_keyframe_ = 80;

    // util
    cv::Ptr<cv::GFTTDetector> gftt_;        // feature detector from opencv

};

}

#endif