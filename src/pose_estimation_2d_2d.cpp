#include <iostream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;

void find_feature_matches(
    const cv::Mat &img_1, const cv::Mat &img_2,
    std::vector<cv::KeyPoint> &keypoints_1,
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches 
);

void pose_estimation_2d2d(
    std::vector<cv::KeyPoint> &keypoints_1,
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches,
    cv::Mat &R, cv::Mat &t
);

cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K);

int main(int argc, char **argv) {
    if (argc != 3) {
        cout << "usage: pose estimation_2d2d img1 img2" << endl;
        return 1;
    }

    // load images
    cv::Mat img_1 = cv::imread(argv[1], cv::IMREAD_COLOR);
    cv::Mat img_2 = cv::imread(argv[2], cv::IMREAD_COLOR);
    assert(img_1.data && img_2.data && "Cannot load images!");

    // match and estimate pose
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    std::vector<cv::DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);

    cv::Mat R, t;
    pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);

    // Check E=t^R*scale skew symmetric matrix validity
    cv::Mat t_x =
            (cv::Mat_<double>(3,3) << 0, -t.at<double>(2,0), t.at<double>(1,0),
            t.at<double>(2,0), 0, -t.at<double>(0,0),
            -t.at<double>(1,0), t.at<double>(0,0), 0);

    cout << "t^R= " << endl << t_x * R << endl;

    // Check the epipolar constraints
    cv::Mat K = (cv::Mat_<double>(3,3) << 520.9,    0,      325.1, 
                                            0,      521.0,  249.7, 
                                            0,      0,      1); // Camera intrinsics matrix

    for (cv::DMatch m : matches) {
        cv::Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        cv::Mat y1 = (cv::Mat_<double>(3,1) << pt1.x, pt1.y, 1);

        cv::Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
        cv::Mat y2 = (cv::Mat_<double>(3,1) << pt2.x, pt2.y, 1);

        cv::Mat d = y2.t() * t_x * R * y1;
        cout << "epipolar constraint: " << d << endl;
    }

    return 0;
}


void find_feature_matches(
    const cv::Mat &img_1, const cv::Mat &img_2,
    std::vector<cv::KeyPoint> &keypoints_1,
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches) {

    cv::Mat descriptors_1, descriptors_2;

    // Create detector and feature descriptor
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    // match points
    std::vector<cv::DMatch> match;
    // brute force approach to matching
    matcher->match(descriptors_1, descriptors_2, match);

    double min_dist = 10000, max_dist = 0;

    // compute the min_max distances to reject outlier matches
    for (int i = 0; i < descriptors_1.rows; i++) {
        double dist = match[i].distance;

        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }

    std::printf("-- Max dist : %f \n", max_dist);
    std::printf("-- Min dist : %f \n", min_dist);

    // reject outlier distances
    for (int i = 0; i < descriptors_1.rows; i++) {
        if (match[i].distance <= max(2 * min_dist, 30.0)) {
            matches.push_back(match[i]);
        }
    }
}

cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K) {
    return cv::Point2d( (p.x - K.at<double>(0,2)) / K.at<double>(0,0),
                        (p.y - K.at<double>(1,2)) / K.at<double>(1,1));
}

void pose_estimation_2d2d(
    std::vector<cv::KeyPoint> &keypoints_1,
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches,
    cv::Mat &R, cv::Mat &t ) {

    cv::Mat K = (cv::Mat_<double>(3,3) << 520.9,    0,      325.1, 
                                            0,      521.0,  249.7, 
                                            0,      0,      1); // Camera intrinsics matrix
    // Convert matching points into a form of vector<Point2f>
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;

    for (int i = 0; i < (int) matches.size(); i++) {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    // Get camera principal point as calculated in the TUM dataset
    cv::Point2d principal_point(325.1, 249.7);
    double focal_length = 521;  // camera's focal length as calibrated from the TUM dataset

    // Calculate the Fundemental matrix
    cv::Mat fundemental_matrix;
    fundemental_matrix = cv::findFundamentalMat(points1, points2, cv::FM_8POINT);
    cout << "Calculated fundemental matrix is: " << endl << fundemental_matrix << endl;

    // Calculate the Essential matrix
    cv::Mat essential_matrix;
    essential_matrix = cv::findEssentialMat(points1, points2, focal_length, principal_point);
    cout << "Essential matrix is: " << endl << essential_matrix << endl;

    // If the scene planar, may compute the homography transformation
    cv::Mat homography_matrix;
    homography_matrix = cv::findHomography(points1, points2, cv::RANSAC, 3);
    cout << "Homography matrix is: " << endl << homography_matrix << endl;

    // Finally, recover the rotation and translation matricies from the essential matrix
    cv::recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);

    cout << "R is " << endl << R << endl;
    cout << "t is " << endl << t << endl;
}