#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

//int CHECKERBOARD[2]{10,7}; 

#define log(smth) std::cout << smth << std::endl;

namespace cam {
    bool CameraParams(int cam, int CHECKERBOARD[2], cv::Size rezizedImg, double scale, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<std::vector<cv::Point3f>> *objpoints, std::vector<std::vector<cv::Point2f> > *imgpoints, double *error = nullptr){

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

    bool CameraParams(std::vector<cv::Mat> images, int CHECKERBOARD[2], double scale, cv::Size imgSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<std::vector<cv::Point3f>> *objpointsPointer = nullptr, std::vector<std::vector<cv::Point2f>> *imgpointsPointer = nullptr, double *error = nullptr){

        // checkboard corner position on image 2d plane
        

        // Defining the world coordinates for 3D points
        std::vector<cv::Point3f> objp;
        for(int i{0}; i<CHECKERBOARD[1]; i++)
        {
            for(int j{0}; j<CHECKERBOARD[0]; j++)
            objp.push_back(cv::Point3f(j*scale,i*scale,0));
        }

        std::vector<std::vector<cv::Point3f> > objpoints; 
        std::vector<std::vector<cv::Point2f> > imgpoints;
        

        // vector to store the pixel coordinates of detected checker board corners 
        std::vector<cv::Point2f> corner_pts;

        bool success;
        // Looping over all the images in the directory
        for(int i=0; i<images.size(); i++)
        {
            cv::Mat gray;
            cv::cvtColor(images[i],gray,cv::COLOR_BGR2GRAY);

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
                //cv::drawChessboardCorners(images[i], cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
                
                objpoints.push_back(objp);
                
                imgpoints.push_back(corner_pts);
            }

            
        }

        cv::Mat R,T;

        double errorr = cv::calibrateCamera(objpoints, imgpoints, cv::Size(imgSize.width, imgSize.height), cameraMatrix, distCoeffs, R, T);
        
        if(objpointsPointer != nullptr){
            *objpointsPointer = objpoints;
        }

        if(imgpointsPointer != nullptr){
            *imgpointsPointer = imgpoints;
        }

        if(error != nullptr){
            *error = errorr;
        }

        return true;
    }

    bool findCheckboardPattern(cv::Mat& img, int CHECKERBOARD[2]){
        cv::Mat gray;
        cv::cvtColor(img, gray,cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corner_pts;

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true  
        return cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
  
    }

    double blurness(cv::Mat& img){
        cv::Mat gray;
        cv::cvtColor( img, gray, cv::COLOR_RGB2GRAY );

        //Cool, let's compute the laplacian of the gray image:
        cv::Mat laplacianImage;
        cv::Laplacian( gray, laplacianImage, CV_64F );

        //Prepare to compute the mean and standard deviation of the laplacian:
        cv::Scalar mean, stddev; 
        cv::meanStdDev( laplacianImage, mean, stddev, cv::Mat() );

        //Let’s compute the variance:
        return stddev.val[0] * stddev.val[0];

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

    bool CamPosARUCO(cv::Mat& image, cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Mat *rotationVector, cv::Mat *translationVector){
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

        cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();

        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::aruco::ArucoDetector detector(dictionary, detectorParams);

        detector.detectMarkers(image, markerCorners, markerIds, rejectedCandidates);

        if(markerCorners.size() != 0){
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

    // Function to save camera matrix and distortion coefficients to a file
    bool saveCameraCalibration(const std::string& directory, const std::string& filename, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) {
        // Create the directory if it doesn't exist
        if (!fs::exists(directory)) {
            if (!fs::create_directories(directory)) {
                std::cerr << "Failed to create the directory." << std::endl;
                return false;
            }
        }

        // Create the full file path
        std::string filePath = directory + "/" + filename;

        cv::FileStorage fs(filePath, cv::FileStorage::WRITE);
        if (!fs.isOpened()) {
            return false;
        }
        fs << "CameraMatrix" << cameraMatrix;
        fs << "DistortionCoeffs" << distCoeffs;
        fs.release();
        return true;
    }

    // Function to load camera matrix and distortion coefficients from a file
    bool loadCameraCalibration(const std::string& filename, cv::Mat& cameraMatrix, cv::Mat& distCoeffs) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            return false;
        }
        fs["CameraMatrix"] >> cameraMatrix;
        fs["DistortionCoeffs"] >> distCoeffs;
        fs.release();
        return true;
    }

}