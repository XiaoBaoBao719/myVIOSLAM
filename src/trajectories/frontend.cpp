#include <opencv2/opencv.hpp>

#include "frontend.h"
#include "feature.h"
#include "map.h"
#include "frame.h"

namespace myvioslam {

Frontend::Frontend() {

}

bool AddFrame(myvioslam::Frame::Ptr frame) {
    current_frame_ = frame;
}

}