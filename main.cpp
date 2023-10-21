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
#include <cstdlib>
#include <fstream>

#include "./libs/pointFinder.hpp"
#include "./libs/Camera.hpp"
#include "./libs/performance.hpp"


int CHECKERBOARD[2]{10,7}; 
cv::Size reziedImgSize = cv::Size(640, 480);

double scale = 60;

#define log(smth) std::cout << smth << std::endl;

void CamFinder(){
    cv::VideoCapture capture;
    int cameraIndex = 0; // Start with the first camera (index 0)

    // Redirect stderr to a null stream to suppress warning and error messages
    std::ofstream nullStream;
    std::streambuf* originalErrorStream = std::cerr.rdbuf(nullStream.rdbuf());

    while (cameraIndex < 500) {
        capture.open(cameraIndex);

        if (capture.isOpened()) {
            std::cout << "Camera index " << cameraIndex << " is available." << std::endl;
            capture.release(); // Release the camera
        }

        cameraIndex++; // Try the next camera index
    }

    // Restore the original stderr stream
    std::cerr.rdbuf(originalErrorStream);

}

int main(){
    
    CamFinder();
    pfm::FPSCounter fpsCounter;

    cv::Mat frame;
    cv::VideoCapture cap;
    cap.open(202);
    cap.set(cv::CAP_PROP_FPS, 75);

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

    cam::loadCameraCalibration("Cameras/1/calibration.xml", cameraMatrix, distCoeffs);

    for (;;)
    {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        //log(frame.size().width << " : " << frame.size().height)
        // check if we succeeded
        if (frame.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        // show live and wait for a key with timeout long enough to show images
        cv::Mat camT, camR;
        bool pos = cam::CamPosARUCO(frame, cameraMatrix, distCoeffs, camR, camT);
        if(pos){
            cv::Mat aruco = frame.clone();

            cv::drawFrameAxes(aruco, cameraMatrix, distCoeffs, -camR, -camT, 1);
            imshow("aruco", aruco);
        }

        cv::Mat undistord = cam::undistordImage(frame, cameraMatrix, distCoeffs);
        imshow("u", frame);
        imshow("Live", frame);
        


        int key = cv::waitKey(1);
        fpsCounter.update();
        //log(fpsCounter.getFPS());

        if (key == 13) {
            cam::SaveImg(frame, "./Cameras/3/" + std::to_string(c) + ".jpg");
            c++;
        }




    }
}