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
};

}

#endif