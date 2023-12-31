// nav_obj_tracker.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include "NavTracker.h"
#include <conio.h> 


int main()
{
    std::cout << "Hello World!\n";
    std::string img_add_prefix = "C:\\Users\\amira\\OneDrive\\Desktop\\harris corner detection algorithm\\pics\\frames 1b\\frameNo";
    int initIndex = 538;
    cv::Mat initImg = imread(img_add_prefix + std::to_string(initIndex) + ".jpg");
    circle(initImg, Point(0, 0), 8, Scalar(255, 0, 0), 3);
    //imshow("", initImg);
    //waitKey();
    std::cout << "tracker init" << std::endl;
    NavTracker nav(initImg, 20.);
    nav.ShowImgWithFeatures();
    Mat img;
    for (size_t i = initIndex; i < 2640; i++)
    {
        system("cls");
        img = imread(img_add_prefix + std::to_string(i) + ".jpg");
        nav.NextFrame(img);
        nav.ShowImgWithFeatures();
    }
}

