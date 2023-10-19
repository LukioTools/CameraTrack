#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

#include "pointFinder.hpp"

#define log(smth) std::cout << smth << std::endl;


/* Define used checkboard size. You can create simple checkboard using online generators and print it. 
*  To calibrate camera put the checkboard front of your camera and take pic, then move little bit and take another.
*  Ten pictures is enough to calibrate camera and solve camera matrix and distCoeffs.
*/
int CHECKERBOARD[2]{10,7}; 

// Define how many cameras are there
int Cameras = 2;

// Define rezied pic size
cv::Size reziedImgSize = cv::Size(1000, 750);

// Define tracking mask lower rgb and higher rgb values
// this one is for blue objects
cv::Scalar min_mask_rgb(0,30,0);
cv::Scalar max_mask_rgb(255,150,20);


void LogCameraMatrix(cv::Mat cameraMatrix, cv::Mat distCoeffs){
    std::cout << "cameraMatrix : \n" << cameraMatrix << std::endl;
    std::cout << "distCoeffs : " << distCoeffs << std::endl; 
}

cv::Mat undistordImage(cv::Mat img, cv::Mat cameraMatrix, cv::Mat distCoeffs){
    cv::Mat newimg;
    cv::undistort(img, newimg, cameraMatrix, distCoeffs);

    return newimg;
}

cv::Vec3d rotationVectorToEulerAngles(const cv::Mat& rotationVector) {
    cv::Mat rotationMatrix;
    cv::Rodrigues(rotationVector, rotationMatrix);

    cv::Vec3d eulerAngles;
    cv::Mat rotationMatrixR = rotationMatrix(cv::Rect(0, 0, 3, 3));
    cv::Mat_<double> eulerMatrix = cv::Mat_<double>(rotationMatrixR);

    eulerAngles[0] = atan2(eulerMatrix(1, 2), eulerMatrix(2, 2)); // Roll
    eulerAngles[1] = asin(-eulerMatrix(0, 2)); // Pitch
    eulerAngles[2] = atan2(eulerMatrix(0, 1), eulerMatrix(0, 0)); // Yaw

    eulerAngles *= (180.0 / CV_PI);

    return eulerAngles;
}

// Is used to estimate or solve camera posiiton in 3d space using checkboard. 
bool estimateCameraPose(cv::Mat& image, int checkerboardRows, int checkerboardCols, 
                        cv::Mat& rotationVector, cv::Mat& translationVector, cv::Mat cameraMatrix, cv::Mat distCoeffs) {
    // Define checkerboard parameters
    float squareSize = 0.02f; // Size of each square on the checkerboard in meters

    // Create vectors to store 3D world points (checkerboard corners)
    std::vector<cv::Point3f> objectPoints;
    for (int i = 0; i < checkerboardRows; i++) {
        for (int j = 0; j < checkerboardCols; j++) {
            objectPoints.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
        }
    }

    // Detect the checkerboard corners in the captured image
    std::vector<cv::Point2f> imagePoints;
    bool found = cv::findChessboardCorners(image, cv::Size(checkerboardCols, checkerboardRows), imagePoints);

    if (found) {
        cv::Mat rvec, tvec;
        
        // Estimate the pose using solvePnP
        cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
        
        // Return the rotation vector and translation vector
        rotationVector = rvec;
        translationVector = tvec;
        
        return true;
    }

    return false;
}

bool CameraParams(int cam, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, 
                    std::vector<std::vector<cv::Point3f>> *objpoints, 
                    std::vector<std::vector<cv::Point2f> > *imgpoints){

// checkboard corner position on image 2d plane
    

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for(int i{0}; i<CHECKERBOARD[1]; i++)
    {
        for(int j{0}; j<CHECKERBOARD[0]; j++)
        objp.push_back(cv::Point3f(j,i,0));
    }

    // Calibration images
    std::vector<cv::String> images;

    //path to images
    std::string path = "./Cameras/" + std::to_string(cam) + "/Cal/*.jpg";

    cv::glob(path, images);

    cv::Mat frame, gray;

    // vector to store the pixel coordinates of detected checker board corners 
    std::vector<cv::Point2f> corner_pts;

    bool success;

    // Looping over all the images in the directory
    for(int i{0}; i<images.size(); i++)
    {
        frame = cv::imread(images[i]);
        cv::resize(frame, frame, reziedImgSize, cv::INTER_LINEAR);
        cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true  
        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        /* 
        * If desired number of corner are detected,
        * refine the pixel coordinates and display 
        * them on the images of checker board
        */
        if(success)
        {
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
            
            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(gray,corner_pts,cv::Size(11,11), cv::Size(-1,-1),criteria);
            
            // Displaying the detected corner points on the checker board
            cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
            
            (*objpoints).push_back(objp);
            (*imgpoints).push_back(corner_pts);
        }

        cv::imshow("end result",frame);
        cv::waitKey(1);
    }

    cv::Mat R,T;
    cv::calibrateCamera(*objpoints, *imgpoints, cv::Size(gray.rows,gray.cols), cameraMatrix, distCoeffs, R, T);
    return true;
}

cv::Mat Track_mask(cv::Mat undistorted_img){
    cv::Mat mask;

    // Get rough mask
    cv::inRange(undistorted_img, min_mask_rgb, max_mask_rgb, mask);

    // Clean the mask
    cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2)) );
    cv::dilate( mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2)) ); 

    cv::dilate( mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2)) ); 
    cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2)) );

    cv::imshow("mask",mask);
    cv::waitKey(0);

    return mask;
}

//cv::Point2f centerPoint()

int main(){

    for (int i = 0; i < Cameras; i++)
    {
        cv::Mat cameraMatrix,distCoeffs;
        std::vector<std::vector<cv::Point3f> > objpoints; 
        std::vector<std::vector<cv::Point2f> > imgpoints;

        CameraParams(i + 1, cameraMatrix, distCoeffs, &objpoints, &imgpoints);

        LogCameraMatrix(cameraMatrix, distCoeffs);
        

        cv::Mat pos_img = cv::imread("./Cameras/" + std::to_string(i +1 ) + "/pos.jpg");
        cv::Mat tracking_img = cv::imread("./Cameras/" + std::to_string(i +1 ) + "/tracking.jpg");

        cv::resize(pos_img, pos_img, reziedImgSize, cv::INTER_LINEAR);
        cv::resize(tracking_img, tracking_img, reziedImgSize, cv::INTER_LINEAR);

        // Cameras 3d position and rotation
        cv::Mat rotationVector, translationVector;

        bool posEstimated = estimateCameraPose(pos_img, CHECKERBOARD[0], CHECKERBOARD[1], rotationVector, translationVector, cameraMatrix, distCoeffs);

        if (posEstimated) {

            cv::Vec3d eulerAngles = rotationVectorToEulerAngles(rotationVector);

            log("Position and Rotation Information:");
            log("   Translation Vector: " << translationVector* 100);
            log("   Rotation Vector:    " << rotationVector);
            log("   euler Angles (deg): " << eulerAngles);
        }

        cv::Mat unDTtacking = undistordImage(tracking_img, cameraMatrix, distCoeffs); 
        cv::Mat mask = Track_mask(unDTtacking);

        log(" ------------------------- ")
    }
    
}