#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <opencv2/core/cvdef.h>

using namespace cv;

class NavTracker {
public:
	NavTracker(Mat& first_image, int min_distance);
	void NextFrame(Mat& next_image);
	const std::vector<cv::Point2f>& GetFeaturePoints() const;
	void ShowImgWithFeatures() const;

private:
	Mat current_img;
	std::vector<cv::Point2f> feature_points;
	double min_distance_between_features;
	void CreateNewPoint();
};

