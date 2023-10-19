#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>


#define log(smth) std::cout << smth << std::endl;

// Defining the dimensions of checkerboard
int CHECKERBOARD[2]{10,7}; 


void LogCameraMatrix(cv::Mat cameraMatrix, cv::Mat distCoeffs){
    std::cout << "cameraMatrix : \n" << cameraMatrix << std::endl;
    std::cout << "distCoeffs : " << distCoeffs << std::endl; 
}

cv::Mat undistordImage(cv::Mat img, cv::Mat cameraMatrix, cv::Mat distCoeffs){
    cv::Mat resized;
    cv::resize(img, resized, cv::Size(1000, 750), cv::INTER_LINEAR);
    cv::Mat newimg;
    cv::undistort(resized, newimg, cameraMatrix, distCoeffs);

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

int main()
{
    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f> > objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f> > imgpoints;

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for(int i{0}; i<CHECKERBOARD[1]; i++)
    {
        for(int j{0}; j<CHECKERBOARD[0]; j++)
        objp.push_back(cv::Point3f(j,i,0));
    }


    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> images;
    // Path of the folder containing checkerboard images
    std::string path = "./CalibrationImages/*.jpg";

    cv::glob(path, images);

    cv::Mat frame, gray;
    // vector to store the pixel coordinates of detected checker board corners 
    std::vector<cv::Point2f> corner_pts;
    bool success;
    log(success)
    log("hmm")

    // Looping over all the images in the directory
    for(int i{0}; i<images.size(); i++)
    {
        log("smthin")
        frame = cv::imread(images[i]);
        cv::resize(frame, frame, cv::Size(1000, 750), cv::INTER_LINEAR);
        log("smthin 2")
        cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);
        log("smthin 3")

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true  
        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        log(success)
        /* 
        * If desired number of corner are detected,
        * we refine the pixel coordinates and display 
        * them on the images of checker board
        */
        if(success)
        {
        cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
        
        // refining pixel coordinates for given 2d points.
        cv::cornerSubPix(gray,corner_pts,cv::Size(11,11), cv::Size(-1,-1),criteria);
        
        // Displaying the detected corner points on the checker board
        cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
        
        objpoints.push_back(objp);
        imgpoints.push_back(corner_pts);
        }

            cv::imshow("end result",frame);
            cv::waitKey(1);
    }


    cv::Mat cameraMatrix,distCoeffs,R,T;

    /*
    * Performing camera calibration by 
    * passing the value of known 3D points (objpoints)
    * and corresponding pixel coordinates of the 
    * detected corners (imgpoints)
    */
    cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), cameraMatrix, distCoeffs, R, T);

    LogCameraMatrix(cameraMatrix, distCoeffs);

    cv::Mat rotationVector, translationVector;
    cv::Mat posImg = cv::imread("./Cameras/1/pos.jpg");
    bool posEstimated = estimateCameraPose(posImg, CHECKERBOARD[0], CHECKERBOARD[1], rotationVector, translationVector, cameraMatrix, distCoeffs);

    if (posEstimated) {

    cv::Vec3d eulerAngles = rotationVectorToEulerAngles(rotationVector);

    log("Position and Rotation Information:");
    log("   Translation Vector: " << translationVector* 100);
    log("   Rotation Vector:    " << rotationVector);
    log("   euler Angles (deg): " << eulerAngles);
    log("Done");
    }

    cv::Mat newimg = undistordImage(cv::imread("./Cameras/1/tracking.jpg"), cameraMatrix, distCoeffs);
    cv::Mat mask;

    cv::inRange(newimg, cv::Scalar(0, 30, 0), cv::Scalar(255, 150, 20), mask);

    cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2)) );
    cv::dilate( mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2)) ); 

    cv::dilate( mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2)) ); 
    cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2)) );

    cv::imshow("end result",mask);
    cv::waitKey(0);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

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

        // Print the center coordinates
        std::cout << "Center X: " << center.x << ", Center Y: " << center.y << std::endl;

        log("x (mm): " << 5.6*(center.x/1000))
        log("x (mm): " << 4.2*(center.y/750))
        log(5.6*(center.x/1000))

        // Optionally, you can mark the center on the image
        cv::circle(newimg, center, 5, cv::Scalar(0, 0, 255), -1); // Red circle

    }
    cv::imshow("end result",newimg);
    cv::waitKey(0);
    
    return 0;
}