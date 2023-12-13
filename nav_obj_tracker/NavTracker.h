#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <opencv2/core/cvdef.h>

using namespace cv;

class NavTracker {
public:
	/// <summary>
	/// Constructor
	/// </summary>
	/// <param name="first_image">First image of Mat stream</param>
	/// <param name="min_distance">Min init dist between points in first frame</param>
	NavTracker(Mat& first_image, int min_distance);
	/// <summary>
	/// Updates and follows points in the next frame.
	/// Creates new points if necessary.
	/// </summary>
	/// <param name="next_image">Next Mat in stream</param>
	void NextFrame(Mat& next_image);
	/// <summary>
	/// Gets current reference points on the ground
	/// </summary>
	/// <returns></returns>
	const std::vector<cv::Point2f>& GetFeaturePoints() const;
	/// <summary>
	/// show the image with the features
	/// </summary>
	void ShowImgWithFeatures() const;
	/// <summary>
	/// get the image with the features
	/// </summary>
	/// <returns></returns>
	cv::Mat GetImgWithFeatures() const;

private:
	/// current frame
	Mat current_img;
	/// current reference points on the ground
	std::vector<cv::Point2f> feature_points;
	/// min distance between points on the mat
	double min_distance_between_features;
	void (*FeatureRemovedCallback)(const cv::Point2f& removed_point);

	/// <summary>
	/// creates a new point on the mat, that is not too close to other points
	/// </summary>
	void CreateNewPoint();
};