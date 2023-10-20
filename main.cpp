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
#include <string>

#include "./libs/pointFinder.hpp"
#include "./libs/Triangulation.hpp"
#include "./libs/Camera.hpp"
#include "./libs/performance.hpp"


int CHECKERBOARD[2]{10,7}; 
cv::Size reziedImgSize = cv::Size(640, 480);

double scale = 60;

#define log(smth) std::cout << smth << std::endl;

int main(){

    pfm::FPSCounter fpsCounter;

    cv::Mat frame;
    cv::VideoCapture cap;
    cap.open(0);

    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    bool captureRequested = false;
    bool saved = false;

    int c = 0;

    cv::Mat cameraMatrix,distCoeffs;
    std::vector<std::vector<cv::Point3f> > objpoints; 
    std::vector<std::vector<cv::Point2f> > imgpoints;

    cam::CameraParams(3, CHECKERBOARD, reziedImgSize, 5, cameraMatrix, distCoeffs, &objpoints, &imgpoints);
    
    for (;;)
    {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        // show live and wait for a key with timeout long enough to show images
        cv::Mat camT, camR;
        bool pos = cam::CamPosARUCO(frame, cameraMatrix, distCoeffs, &camR, &camT);

        imshow("Live", frame);


        int key = cv::waitKey(1);
        //sfpsCounter.update();
        //log(fpsCounter.getFPS());

        if (key == 13) {
            cam::SaveImg(frame, "./Cameras/3/" + std::to_string(c) + ".jpg");
            c++;
        }




    }
}