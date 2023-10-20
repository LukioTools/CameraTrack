#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

#include "./libs/pointFinder.hpp"
#include "./libs/Triangulation.hpp"
#include "./libs/Camera.hpp"

#define log(smth) std::cout << smth << std::endl;


/* Define used checkboard size. You can create simple checkboard using online generators and print it. 
*  To calibrate camera put the checkboard front of your camera and take pic, then move little bit and take another.
*  Ten pictures is enough to calibrate camera and solve camera matrix and distCoeffs.
*/
int CHECKERBOARD[2]{10,7}; 

// Define how many cameras are there
int Cameras = 2;

double unit = 25; // currently in mm

// Define rezied pic size
cv::Size reziedImgSize = cv::Size(1000, 750);

// Define tracking mask lower rgb and higher rgb values
// this one is for blue objects
cv::Scalar min_mask_rgb(0,30,0);
cv::Scalar max_mask_rgb(255,150,20);

void GenerateARUCOMarker(int id){
    cv::Mat markerImage;
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::generateImageMarker(dictionary, id, 200, markerImage, 1);
    cv::imwrite("marker.png", markerImage);
}

void LogCameraMatrix(cv::Mat cameraMatrix, cv::Mat distCoeffs){
    std::cout << "cameraMatrix : \n" << cameraMatrix << std::endl;
    std::cout << "distCoeffs : " << distCoeffs << std::endl; 
}

int main(){

    GenerateARUCOMarker(4);

    std::vector<gty::Line3D> TrackerLines;

    cv::Mat ftv;

    for (int i = 0; i < Cameras; i++)
    {
        cv::Mat cameraMatrix,distCoeffs;
        std::vector<std::vector<cv::Point3f> > objpoints; 
        std::vector<std::vector<cv::Point2f> > imgpoints;

        cam::CameraParams(i + 1, CHECKERBOARD, reziedImgSize,1, cameraMatrix, distCoeffs, &objpoints, &imgpoints);

        cv::Mat arucoImg = cv::imread("./Cameras/1/aruco_pos.jpg");
        cv::resize(arucoImg, arucoImg, reziedImgSize, cv::INTER_LINEAR);
        cv::Mat arucoMarkers = arucoDetect(arucoImg, cameraMatrix, distCoeffs);

        cv::imshow("end result",arucoMarkers);
        cv::waitKey(0);

        LogCameraMatrix(cameraMatrix, distCoeffs);
        

        cv::Mat pos_img = cv::imread("./Cameras/" + std::to_string(i +1 ) + "/pos.jpg");
        cv::Mat tracking_img = cv::imread("./Cameras/" + std::to_string(i +1 ) + "/tracking.jpg");

        cv::resize(pos_img, pos_img, reziedImgSize, cv::INTER_LINEAR);
        cv::resize(tracking_img, tracking_img, reziedImgSize, cv::INTER_LINEAR);

        // Cameras 3d position and rotation
        cv::Mat rv, tv;

        bool posEstimated = estimateCameraPose(pos_img, CHECKERBOARD[0], CHECKERBOARD[1], rv, tv, cameraMatrix, distCoeffs);

        if (posEstimated) {

            cv::Vec3d eulerAngles = rotationVectorToEulerAngles(rv);

            log("Position and Rotation Information:");
            log("   Translation Vector: " << tv* 100);
            log("   Rotation Vector:    " << rv);
            log("   euler Angles (deg): " << eulerAngles);
        }

        tv = -tv;
        rv = -rv;

        cv::Mat unDTtacking = undistordImage(tracking_img, cameraMatrix, distCoeffs); 
        cv::Mat mask = Tracking_mask(unDTtacking);

        cv::Point2f tracker = cpf::ByBiggest(&mask);

        //calculate tracker rotation in local space (mm)
        gty::vector3 localRotVec;
        localRotVec.x = 5.6*(tracker.x/reziedImgSize.width);
        localRotVec.y = 4.2*(tracker.y/reziedImgSize.height);
        localRotVec.z = 1.74;

        gty::vector3 CamRotVec = gty::vector3(rv);

        cv::Mat localRotMat = localRotVec.RotMat();
        cv::Mat CamRotMat = CamRotVec.RotMat();

        cv::Mat lineRotMat = localRotMat*CamRotMat;
        
        TrackerLines.push_back(gty::Line3D(gty::vector3(lineRotMat),tv));
        log("rot" << gty::vector3(lineRotMat));
        ftv = tv;

        log(" ------------------------- ")
    }


    cv::Point3d it;
    double dist;
    tri::EstimateIntersection(TrackerLines, &it, &dist);

    double distToCam = tri::Dist(it, gty::vector3().fromCVMat(ftv).toCVPoint());

    log("point: " << it << " dist from lines: " << dist << " dist to last cam" << distToCam);
    
}