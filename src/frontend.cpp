#include <opencv2/opencv.hpp>

#include "frontend.h"
#include "feature.h"
#include "map.h"
#include "frame.h"

namespace myvioslam {

Frontend::Frontend(Viewer viewer) :
            viewer_(viewer) {

}

bool Frontend::AddFrame(myvioslam::Frame::Ptr frame) {
    current_frame_ = frame;

    switch (status_) {
        case FrontendStatus::INITING : 
            InitStereo();
            break;
        case FrontendStatus::LOST :
            Reset();
            break;
        case FrontendStatus::TRACKING_BAD :
            Track();
            break;
        case FrontendStatus::TRACKING_GOOD :
            std::cout << "tracking..." << std::endl;
    }

    last_frame_ = current_frame_;
    return true;
}

bool Frontend::Track() {
    if (last_frame_) {
        current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
    }

    int num_tracking = TrackLastFrame();
    tracking_inliers_ = EstimateCurrentPose();

    if (tracking_inliers_ > num_features_tracking_) {
        // tracking is good
        status_ = FrontendStatus::TRACKING_GOOD;
    } else if (tracking_inliers_ > num_features_tracking_bad_) {
        // tracking is bad
        status_ = FrontendStatus::TRACKING_BAD;
    } else {
        // num tracked inliers is below threshold, we are lost
        status_ = FrontendStatus::LOST;
    }

    InsertKeyFrame();
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

    if (viewer_) {
        viewer_->AddKeyFrame(current_frame_);
    }
    return true;
}

int Frontend::TrackLastFrame() {
    // Make use of Lucas Kanade optical flow to estimate points in right img
    std::vector<cv::Point2f> keypoints_last, keypoints_current;
    for (auto &kp : last_frame_->features_left_) {
        if (kp->map_point_.lock()) {
            // use projection point
            auto mp = kp->map_point_.lock();
            auto px = camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
            keypoints_last.push_back(kp->position_.pt);
            keypoints_current.push_back(cv::Point2f(px[0],px[1]));
        } else {
            keypoints_last.push_back(kp->position_.pt);
            keypoints_current.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;      // results of optical flow
    cv::Mat error;
    cv::calcOpticalFlowPyrLK(
        last_frame_->left_img_, current_frame_->left_img_, keypoints_last,
        keypoints_current, status, error, cv::Size(21, 21), 3, 
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;

    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(keypoints_current[i], 7);
            Feature::Ptr feature(new Feature(current_frame_, kp));
            feature->map_point_ = last_frame_->features_left_[i]->map_point_;   // align prev seen mps with currently seen mpps
            current_frame_->features_left_.push_back(feature);
            num_good_pts++;
        }
    }

    LOG(INFO) << "Found: " << num_good_pts << " in the last image.";
    return num_good_pts;
}

}