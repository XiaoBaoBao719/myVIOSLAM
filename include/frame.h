#pragma once

#ifndef MYVIOSLAM_FRAME_H
#define MYVIOSLAM_FRAME_H

#include "common_include.h"

namespace myvioslam {

struct MapPoint;
struct Feature;

struct Frame {
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frame> Ptr;

        unsigned long id_ = 0;          // frame id
        unsigned long keyframe_id = 0;  // keyframe id
        bool is_keyframe = false;       // if frame is a keyframe
        double timestamp;               // frame time stamp
        SE3 pose_;                      // pose as defined as a Tcw (world to cam)
        std::mutex pose_mutex_;         // Pose data frame mutex
        cv::Mat left_img_, right_img_;  // stereo images that define that frame

        // extracted image features from the left frame
        std::vector<std::shared_ptr<Feature>> features_left_;
        // features in the right image that correspond! nullptr if no corresponding
        std::vector<std::shared_ptr<Feature>> features_right_;



    public:     // data members
        Frame() {}

        Frame(long id, double time_stamp, const SE3 &pose, const cv::Mat &leftimg,
                const cv::Mat &rightimg);

        // set and get the pose, make it thread safe
        void SetPose(SE3 pose) {
            std::unique_lock<std::mutex> lock(pose_mutex_);
            pose_ = pose;
        }

        SE3 Pose() {
            std::unique_lock<std::mutex> lock(pose_mutex_);
            return pose_;
        }

        // set keyframe and keyframe id
        void SetKeyFrame();

        // create a new frame and assign an id
        static std::shared_ptr<Frame> CreateFrame();
};

}   // namespace myvioslam

#endif