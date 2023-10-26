#pragma once
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

namespace cpf
{

    cv::Mat Tracking_mask(cv::Mat undistorted_img, cv::Scalar min_mask_rgb, cv::Scalar max_mask_rgb){
        cv::Mat mask;

        // Get rough mask
        cv::inRange(undistorted_img, min_mask_rgb, max_mask_rgb, mask);

        // Clean the mask
        cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2)) );
        cv::dilate( mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2)) ); 

        cv::dilate( mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2)) ); 
        cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2)) );
        return mask;
    }

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
    return cv::Point2f();
   } 
} 

