#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

namespace cpf
{
   cv::Point2f ByBiggest(cv::Mat *img){
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(*img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
        double maxArea = 0;
        int maxAreaIdx = 0;

        // Find the largest contour by area
        for (int i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > maxArea) {
                maxArea = area;
                maxAreaIdx = i;
            }
        }

        // Calculate the center of mass of the largest contour
        cv::Moments mu = cv::moments(contours[maxAreaIdx]);
        cv::Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00);
        return center;
    }
   } 
} 

