#pragma once

#ifndef MYVIOSLAM_MAPPOINT_H
#define MYVIOSLAM_MAPPOINT_H

namespace myvioslam{

struct Frame;
struct Feature;

struct MapPoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<MapPoint> Ptr;
    unsigned long id_ = 0;                      // map point id
    Eigen::Vec3 pos_ = Eigen::Vec3::Zero();     // position in the world
    std::mutex data_mutex_;
    int num_observed_times_ = 0;                // num of features that have seen this point
    std::list<std::weak_ptr<Feature>> observations_;

    MapPoint() {}

    MapPoint(long id, Eigen::Vec3 position);

    Eigen::Vec3 Position() {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return pos_;
    }

    void SetPosition(const Eigen::Vec3 &pos) {
        std::unique_lock<std::mutex> lock(data_mutex_);
        pos_ = pos;
    }

    void AddObservation(std::shared_ptr<Feature> feature) {
        std::unique_lock<std::mutex> lock(data_mutex_);
        observations_.push_back(feature);
        num_observed_times_++;
    }

    void RemoveObservation(std::shared_ptr<Feature> feature);

    std::list<std::weak_ptr<Feature>> GetObservations() {
        std::unique_locK<std::mutex> lock(data_mutex_);
        return observations_;
    }

    // factory function
    static MapPoint::Ptr CreateNewMapPoint();
};

}

#endif