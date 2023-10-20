#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

//int CHECKERBOARD[2]{10,7}; 

#define log(smth) std::cout << smth << std::endl;

namespace cam {
    bool CameraParams(int cam, int CHECKERBOARD[2], cv::Size rezizedImg, double scale, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<std::vector<cv::Point3f>> *objpoints, std::vector<std::vector<cv::Point2f> > *imgpoints){

        // checkboard corner position on image 2d plane
        

        // Defining the world coordinates for 3D points
        std::vector<cv::Point3f> objp;
        for(int i{0}; i<CHECKERBOARD[1]; i++)
        {
            for(int j{0}; j<CHECKERBOARD[0]; j++)
            objp.push_back(cv::Point3f(j*scale,i*scale,0));
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
            cv::resize(frame, frame, rezizedImg, cv::INTER_LINEAR);
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
    
    void SaveImg(const cv::Mat& mat, const std::string& filename) {
    // Check if the provided matrix is empty
    if (mat.empty()) {
        std::cerr << "Input matrix is empty. Cannot save as an image." << std::endl;
        return;
    }

    // Use OpenCV's imwrite function to save the matrix as an image
    if (cv::imwrite(filename, mat)) {
        std::cout << "Image saved as " << filename << std::endl;
    } else {
        std::cerr << "Failed to save the image." << std::endl;
    }
    }

    bool CamPosARUCO(cv::Mat image, cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Mat *rotationVector, cv::Mat *translationVector){
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

        cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();

        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::aruco::ArucoDetector detector(dictionary, detectorParams);

        detector.detectMarkers(image, markerCorners, markerIds, rejectedCandidates);

        if(markerCorners.size() == 4){
            std::vector<cv::Point3f> objectPoints = {cv::Point3f(0,1,0), cv::Point3f(1,1,0), cv::Point3f(1,0,0), cv::Point3f(0,0,0)};
            cv::Mat r, t;
            cv::solvePnP(objectPoints, markerCorners[0], cameraMatrix, distCoeffs, r, t);
            *rotationVector = -r;
            *translationVector = -t;
            return true;
        }
        else {
            return false;
        }
    }

    bool CamPosCheckboard(cv::Mat& image, int checkerboardRows, int checkerboardCols, 
                        cv::Mat& rotationVector, cv::Mat& translationVector, cv::Mat cameraMatrix, cv::Mat distCoeffs) {
    // Define checkerboard parameters

    // Create vectors to store 3D world points (checkerboard corners)
    std::vector<cv::Point3f> objectPoints;
    for (int i = 0; i < checkerboardRows; i++) {
        for (int j = 0; j < checkerboardCols; j++) {
            objectPoints.push_back(cv::Point3f(j, i, 0));
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

    cv::Mat undistordImage(cv::Mat image, cv::Mat cameraMatrix, cv::Mat cameraCoeffs){
        cv::Mat result;
        cv::undistort(image, result, cameraMatrix, cameraCoeffs);
        return result;
    }

}