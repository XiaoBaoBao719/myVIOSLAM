#include "common_include.h"
#include "frame.h"

namespace myvioslam {

    Frame::Frame(long id, double time_stamp, const SE3 &pose, const cv::Mat &leftimg,
                    const cv::Mat &rightimg) :
                id_(id), timestamp(time_stamp), pose_(pose), left_img_(leftimg),
                    right_img_(rightimg)
    {

    }

    Frame::Ptr Frame::CreateFrame() {
        static long factory_id = 0;
        Frame::Ptr new_frame(new Frame);
        new_frame->id_ = factory_id++;
        return new_frame;
    }

    void Frame::SetKeyFrame() {
        static long keyframe_factory_id = 0;
        is_keyframe = true;
        keyframe_id = keyframe_factory_id++;
    }
}