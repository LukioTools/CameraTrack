#pragma once
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <iostream>
#include <filesystem>

#include "./Triangulation.hpp"

namespace fs = std::filesystem;

namespace cam{

    class Camera{
        public: 
            int camID;
            cv::Mat cameraMatrix, DistCoeffs;
            cv::VideoCapture cap;
            cv::Mat rMat;
            gty::vector3 tMat;

            Camera(int id){
                camID = id;
            };

            Camera(int id, cv::Mat cameraMatrix, cv::Mat DistCoeffs, cv::Mat rMat, gty::vector3 tMat, cv::VideoCapture cap){
            this->camID = id;
            this->cameraMatrix = cameraMatrix;
            this->DistCoeffs = DistCoeffs;
            this->rMat = rMat;
            this->cap = cap;
        }
        

    };

    bool findCheckboardPattern(cv::Mat& img, int CHECKERBOARD[2]){
        cv::Mat gray;
        cv::cvtColor(img, gray,cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corner_pts;
        // Finding checker board corners
        // If desired number of corners are found in the image then success = true  
        return cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
  
    }

    bool camARUCOPos(cv::Mat& image, cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Mat& transformationMatrix, cv::Point2f *point = nullptr){
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

        cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();

        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::aruco::ArucoDetector detector(dictionary, detectorParams);

        
        detector.detectMarkers(image, markerCorners, markerIds, rejectedCandidates);

        if(markerCorners.size() != 0){
            if(point != nullptr){
                *point = markerCorners[0][0];
            }
            std::vector<cv::Point3f> objectPoints = {cv::Point3f(0,1,0), cv::Point3f(1,1,0), cv::Point3f(1,0,0), cv::Point3f(0,0,0)};
            cv::Mat r, t;
            cv::solvePnP(objectPoints, markerCorners[0], cameraMatrix, distCoeffs, r, t);
            transformationMatrix = tri::createTransformationMatrix(tri::rvecToMat(-r), -t);
            return true;
        }
        return false;
        
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

    cv::Mat undistordImage(cv::Mat image, cv::Mat cameraMatrix, cv::Mat cameraCoeffs){
        cv::Mat result;
        cv::undistort(image, result, cameraMatrix, cameraCoeffs);
        return result;
    }

        // Function to save camera matrix and distortion coefficients to a file
    bool saveCameraCalibration(std::string cam, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) {
        // Create the directory if it doesn't exist
        cam = "Cameras/" + cam;

        if (!fs::exists(cam)) {
            if (!fs::create_directories(cam)) {
                std::cerr << "Failed to create the directory." << std::endl;
                return false;
            }
        }

        // Create the full file path
        std::string filePath = cam + "/" + "calibration.xml";

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
    bool loadCameraCalibration(std::string cam, cv::Mat& cameraMatrix, cv::Mat& distCoeffs) {
        cam = "Cameras/" + cam + "/";
        cv::FileStorage fs(cam + "calibration.xml", cv::FileStorage::READ);
        if (!fs.isOpened()) {
            return false;
        }
        fs["CameraMatrix"] >> cameraMatrix;
        fs["DistortionCoeffs"] >> distCoeffs;
        fs.release();
        return true;
    }

    bool saveCameraPosition(std::string cam, const cv::Mat rotMat, const cv::Vec3d pos) {
        // Create the directory if it doesn't exist
        cam = "Cameras/" + cam;

        if (!fs::exists(cam)) {
            if (!fs::create_directories(cam)) {
                std::cerr << "Failed to create the directory." << std::endl;
                return false;
            }
        }

        // Create the full file path
        std::string filePath = cam + "/" + "transform.xml";

        cv::FileStorage fs(filePath, cv::FileStorage::WRITE);
        if (!fs.isOpened()) {
            return false;
        }
        fs << "CameraRotation" << rotMat;
        fs << "CameraPsition" << pos;
        fs.release();
        return true;
    }

    bool loadCameraPosition(std::string cam, cv::Mat& rotMat, gty::vector3& cameraPosition) {
        
        cv::Vec3d ct;
        cv::Mat rm;
        
        cam = "Cameras/" + cam + "/";
        cv::FileStorage fs(cam + "transform.xml", cv::FileStorage::READ);
        if (!fs.isOpened()) {
            return false;
        }
        fs["CameraRotation"] >> rm;
        fs["CameraPsition"] >> ct;
        fs.release();

        cameraPosition = ct;
        rotMat = rm;

        return true;
    }

    //TODO: saving&loading camera transformation matrix

}