#pragma once
#ifndef MYVIOSLAM_MAP_H
#define MYVIOSLAM_MAP_H

namespace myvioslam {

struct MapPoint;
struct Frame;

class Map {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;     // id association of map points to landmarks
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyFramesType;        // id association of frames to keyframes

    Map() {}

    void InsertKeyFrame(Frame::Ptr frame);

    void InsertMapPoint(MapPoint::Ptr mappoint);

    LandmarksType GetAllMapPoints() {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return landmarks_;
    }

    KeyFramesType GetAllKeyFrames() {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return keyframes_;
    }

    LandmarksType GetActiveMapPoints() {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return active_landmarks_;
    }

    KeyFramesType GetActiceKeyFrames() {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return active_keyframes_;
    }

    void ClearMap();

private:
    // member functions
    void RemoveOldKeyFrames();

    // member data fields
    std::mutex data_mutux_;
    LandmarksType landmarks_;
    LandmarksType active_landmarks_;
    KeyFramesType keyframes_;
    KeyFramesType active_keyframes_;

    Frame::Ptr current_frame_ = nullptr;

    // settings
    int num_active_keyframes_ = 7;          // number of keyframes in the sliding window
};

}

#endif