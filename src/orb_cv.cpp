#include <iostream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

int main(int argc, char **argv) {
    if (argc != 3) {
        cout << "usage: feature_extraction img1 img2" << endl;
        return 1;
    }

    // read in sample images
    cv::Mat img_1 = cv::imread(argv[1], cv::IMREAD_COLOR);
    cv::Mat img_2 = cv::imread(argv[2], cv::IMREAD_COLOR);
    assert(img_1.data != nullptr && img_2.data != nullptr);

    // setup keypoint detection and feature containers
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat desc_1, desc_2;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    // detect keypoints via Oriented FAST
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    // calculate BRIEF descriptor
    descriptor->compute(img_1, keypoints_1, desc_1);
    descriptor->compute(img_2, keypoints_2, desc_2);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2- t1);
    cout << "ORB extraction time cost: " << time_used.count() << " secs. " << endl;

    // draw img with keypoints for debugging
    cv::Mat out_img_1;
    cv::drawKeypoints(img_1, keypoints_1, out_img_1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("ORB features", out_img_1);
    cv::waitKey(3);

    // calculate distance between matches
    std::vector<cv::DMatch> matches;
    t1 = chrono::steady_clock::now();
    matcher->match(desc_1, desc_2, matches);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "ORB matching time cost: " << time_used.count() << " secs. " << endl;

    // sort and remove outliers | calculate min and max distances
    auto min_max = minmax_element(matches.begin(), matches.end(), 
        [](const cv::DMatch &m1, const cv::DMatch &m2)
        {
            return m1.distance < m2.distance;
        }
    );

    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    printf(" -- Max distance : %f \n", max_dist);
    printf(" -- Min distance : %f \n", min_dist);

    // remove bad matches based on purely distance
    std::vector<cv::DMatch> best_matches;
    for (int i = 0; i < desc_1.rows; i++) {
        if (matches[i].distance <= max(2 * min_dist, 30.0)) {
            best_matches.push_back(matches[i]);
        }
    }

    // Draw the best matches and results
    cv::Mat img_match;      // image matches before outlier rejection
    cv::Mat img_goodmatch;  // image matches post outlier rejection

    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, best_matches, img_goodmatch);
    cv::imshow("All matches", img_match);
    cv::imshow("Best matches", img_goodmatch);
    cv::waitKey(0);

    return 0;
}