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
    std::cout << "removed " << removed_ctr << " points." << std::endl;
    for (size_t i = 0; i < removed_ctr; i++)
    {
        std::cout << "iter " << i << std::endl;
        CreateNewPoint();
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
        circle(cpy, p, 8, Scalar(0, 0, 255), 2);
    imshow("PrintMarkedImg()", cpy);
    std::cout << "PrintMarkedImg() called" << std::endl;
    waitKey();
}

void NavTracker::CreateNewPoint()
{
    // search in lower half and higher half x vals
    int mid_x = current_img.cols;
    int mid_y = current_img.rows;

    int roi_width = (mid_x * 3) / 4;
    int roi_height = (mid_y * 3) / 4;

    int ctr_pts_up = 0,
        ctr_pts_rt = 0;

    for (auto p : feature_points) {
        if (p.x > mid_y)
            ++ctr_pts_up;
        else
            --ctr_pts_up;
        if (p.x > mid_x)
            ++ctr_pts_rt;
        else
            --ctr_pts_rt;
    }

    int roi_x = ctr_pts_rt < 0 ? mid_x - roi_width : mid_x;
    int roi_y = ctr_pts_rt < 0 ? mid_y - roi_height : mid_y;
    Rect roi_region(roi_x, roi_y, roi_width, roi_height);
    Mat roi = this->current_img(roi_region);
    equalizeHist(roi, roi);

    std::vector<Point2f> feature_result;

    int max_min = 0;
    Point2f max_min_point;

    goodFeaturesToTrack(roi, feature_result, 10, 0.01, 5. /*maybe more?*/, noArray(), 27, true, 0.04);
    for (auto p : feature_result) {
        int min = pow(mid_x,2);
        for (auto p2 : this->feature_points) {
            if (pow(p.x - p2.x, 2) + pow(p.y - p2.y, 2) < min)
                min = pow(p.x - p2.x, 2) + pow(p.y - p2.y, 2);
        }
        if (min < max_min)
            return;
        max_min = min;
        max_min_point = p;
    }
    if (ctr_pts_rt > 0)
        max_min_point.x += mid_x;
    if (ctr_pts_up > 0)
        max_min_point.x += mid_y;
    this->feature_points.push_back(max_min_point);
}
