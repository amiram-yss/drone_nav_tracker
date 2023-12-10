// C++ program for the above approach 
#include <iostream> 
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


inline vector<Point2f> catch_features(Mat img) {
	vector<Point2f> output;
	Mat cpy = img.clone();
	//cpy.convertTo(cpy, -1, 1.5, 0);
	goodFeaturesToTrack(cpy, output, 10, 0.01, 20., noArray(), 27, true, 0.04);
	for (auto p : output) {
		circle(cpy, p, 8, Scalar(0, 0, 255), 2);
	}
	imshow("", cpy);
	waitKey();
	return output;
}

vector<Point2f> track_features(Mat prev_img, Mat next_img, vector<Point2f> prev_points) {
	vector<Point2f> next_points;
	vector<uchar> status;
	vector<float> err;

	calcOpticalFlowPyrLK(prev_img, next_img, prev_points, next_points, status, err, Size(21, 21), 2);
	// Filter out points with bad status
	for (int i = 0; i < status.size(); i++) {
		if (status[i]) {
			next_points[i] = next_points[i];
		}
		else {
			next_points[i] = Point2f(-1, -1);
		}
	}

	// Draw remaining points on the next frame
	for (auto p : next_points) {
		if (p.x > 0 && p.y > 0) {
			circle(next_img, p, 8, Scalar(0, 255, 0), 2);
		}
	}

	imshow("", next_img);
	waitKey();

	return next_points;
}

int main() {
	string img_add_prefix = "C:\\Users\\amira\\OneDrive\\Desktop\\harris corner detection algorithm\\pics\\frames 1b\\frameNo";
	int index = 1000;//538;
	Mat img = imread(img_add_prefix + to_string(index) + ".jpg", CV_8U);
	imshow("reg", img);
	equalizeHist(img, img);
	//imshow("histonorm", img);
	waitKey();
	Mat prev;
	auto pts = catch_features(img);

	do {
		++index;
		prev = img.clone();
		img = imread(img_add_prefix + to_string(index) + ".jpg", CV_8U);
		equalizeHist(img, img);
		pts = track_features(prev, img, pts);
		if (pts.size() < 9)
			pts = catch_features(img);
	} while (index <= 2400);
	


	waitKey(0);
}