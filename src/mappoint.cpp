#include "mappoint.h"
#include "feature.h"

namespace myvioslam {

    MapPoint::MapPoint(long id, Vec3 position) :
                        id_(id), pos_(position) 
    {

    }

    MapPoint::Ptr MapPoint::CreateNewMapPoint() {
        static long factory_id = 0;
        MapPoint::Ptr new_mappoint(new MapPoint);
        new_mappoint->id_ = factory_id++;
        return new_mappoint;
    } 

    void MapPoint::RemoveObservation(std::shared_ptr<Feature> feature) {
        std::unique_lock<std::mutex> lock(data_mutex_);

        for (auto iter = observations_.begin(); iter != observations_.end();
                iter++)
        {
            if (iter->lock() == feature) {
                observations_.erase(iter);
                feature->map_point_.reset();
                num_observed_times_--;
                break;
            }
        }
    }
}   // namespace myvioslam