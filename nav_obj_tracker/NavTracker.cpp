#include "NavTracker.h"
#include <vector>

NavTracker::NavTracker(Mat& first_image, int min_distance_between_features = 20.) 
    : min_distance_between_features(min_distance_between_features) 
{
    // Convert the image to grayscale and equalize the histogram
    Mat gray_image;
    cvtColor(first_image, gray_image, COLOR_BGR2GRAY);
    equalizeHist(gray_image, gray_image);

    // Find good features to track
    goodFeaturesToTrack(gray_image, feature_points, 10, 0.01, min_distance_between_features, noArray(), 27, true, 0.04);
    current_img = gray_image;
}

void NavTracker::NextFrame(Mat& next_image) {
    // Convert the next image to grayscale
    Mat gray_next_image;
    cvtColor(next_image, gray_next_image, COLOR_BGR2GRAY);
    equalizeHist(gray_next_image, gray_next_image);

    // Calculate optical flow
    std::vector<Point2f> next_points;
    std::vector<uchar> status;
    std::vector<float> err;

    calcOpticalFlowPyrLK(current_img, gray_next_image, feature_points, next_points, status, err, Size(21, 21), 2);

    int removed_ctr = 0;

    // Filter out points with bad status
    feature_points.clear();
    for (int i = 0; i < status.size(); ++i) {
        if (status[i])
            feature_points.push_back(next_points[i]);
        else
            ++removed_ctr;
    }

    current_img = gray_next_image.clone();
    
    for (size_t i = 0; i < removed_ctr; i++)
    {
        //std::cout << "iter " << i << std::endl;
        auto start = std::chrono::high_resolution_clock::now();
        CreateNewPoint();
        auto stop = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout<<"CreateNewPoint() took "<<duration.count()<<" milliseconds"<<std::endl;
    }
}

const std::vector<cv::Point2f>& NavTracker::GetFeaturePoints() const {
    return feature_points;
}

void NavTracker::ShowImgWithFeatures() const
{
    Mat cpy = this->current_img.clone();
    cvtColor(cpy, cpy, COLOR_GRAY2BGR);
    for (auto p : this->feature_points)
        cv::circle(cpy, p, 8, Scalar(0, 0, 255), 2);
    cv::imshow("PrintMarkedImg()", cpy);
    cv::waitKey();
}

cv::Mat NavTracker::GetImgWithFeatures() const
{
    Mat cpy = this->current_img.clone();
    cvtColor(cpy, cpy, COLOR_GRAY2BGR);
    for (auto p : this->feature_points)
        cv::circle(cpy, p, 8, Scalar(0, 0, 255), 2);
    return cpy;
}

void NavTracker::CreateNewPoint()
{
    // support for 4 tiles only
    int mid_x = current_img.cols / 2;
    int mid_y = current_img.rows / 2;
    // pick ref pts only in 2/3 of each dim
    int roi_width = (mid_x * 2) / 3;
    int roi_height = (mid_y * 2) / 3;
    // count #of points in each half. that's how the area to add a new point is selected.
    int ctr_pts_dn = 0,
        ctr_pts_rt = 0;
    for (auto p : feature_points) {
        if (p.y > mid_y)
            ++ctr_pts_dn;
        else
            --ctr_pts_dn;
        if (p.x > mid_x)
            ++ctr_pts_rt;
        else
            --ctr_pts_rt;
    }
    // pick roi for feature pt insertions
    int roi_x = ctr_pts_rt > 0 ? mid_x - roi_width : mid_x;
    int roi_y = ctr_pts_dn > 0 ? mid_y - roi_height : mid_y;
    Rect roi_region(roi_x, roi_y, roi_width, roi_height);
    Mat roi = this->current_img(roi_region);  
    std::vector<Point2f> feature_result;
    int max_min = 0;
    Point2f max_min_point;
    goodFeaturesToTrack(roi, feature_result, 10, 0.01, 25. /*maybe more?*/, noArray(), 27, true, 0.04);
    // pick best point in roi
    for (auto relative_p : feature_result) {
        Point2f p(relative_p.x + roi_x, relative_p.y + roi_y);
        bool changed = false;
        int min = INT_MAX;
        // find min dist to existing points
        for (auto p2 : this -> feature_points) {
            int dist_sqrd = pow(p.x - p2.x, 2) + pow(p.y - p2.y, 2);
            if (dist_sqrd < min) {
                changed = true;
                min = pow(p.x - p2.x, 2) + pow(p.y - p2.y, 2);
            }
        }
        if (!changed)
            continue;
        if (min < max_min)
            continue;
        // if min is bigger than max_min, update max_min and max_min_point
        max_min = min;
        max_min_point = p;
    }
    this->feature_points.push_back(max_min_point);
}