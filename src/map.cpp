#include "map.h"
#include "mappoint.h"
#include "frame.h"
#include "feature.h"

namespace myvioslam {

    void Map::InsertKeyFrame(Frame::Ptr frame) {
        current_frame_ = frame;
        
    }
}