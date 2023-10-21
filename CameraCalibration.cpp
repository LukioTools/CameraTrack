#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <stdio.h>
#include <iostream>
#include <string>
#include <cstdlib>
#include <fstream>
#include <vector>

#include "./libs/Camera.hpp"

#define log(smth) std::cout << smth << std::endl;

int CHECKERBOARD[2]{10,7}; 

cv::Size imgSize = cv::Size(640, 480);


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

    int cvID;
    int camID;

    log("give camera cv ID")
    std::cin >>  cvID;
    log("give camera ID")
    std::cin >> camID;

    // Open camera
    cv::VideoCapture cap;
    cap.open(cvID);

    // Check for camera errors
    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    log("Camera has been opened");

    cv::Mat frame;

    bool cal = false;

    std::vector<cv::Mat> CalibrationImages;

    cv::Mat cameraMatrix, cameraDistCoeffs;

    for(;;){
        cap.read(frame);

        // check if we succeeded
        if (frame.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";
            break;
        }

        cv::imshow("frame", frame);

        int key = cv::waitKey(1);
        if (key == 13) {
            cal = !cal;
            log("keypress");
        }

        if (key == 8) {
            break;
        }

        if (cal) {
            double error = 0;
            bool hasChessboardPattern = cam::findCheckboardPattern(frame, CHECKERBOARD);
            if (!hasChessboardPattern) continue;

            double blurrness = cam::blurness(frame);
            //if(blurrness < 200) continue;;

            if (CalibrationImages.size() >= 10) {
                
                bool outout = cam::CameraParams(CalibrationImages, CHECKERBOARD, 1, imgSize, cameraMatrix, cameraDistCoeffs, nullptr, nullptr, &error);     
                log(error);
            }

            if(CalibrationImages.size() >= 20){
                CalibrationImages.erase(CalibrationImages.begin());
            }

            CalibrationImages.push_back(frame.clone());

            cv::putText(frame, //target image
            "images: " + std::to_string(CalibrationImages.size()) + "error: " + std::to_string(error), //text
            cv::Point(10, frame.rows / 10), //top-left position
            cv::FONT_HERSHEY_DUPLEX,
            0.5,
            CV_RGB(255, 255, 0), //font color
            2);

            cv::imshow("pattern", frame);
            cv::waitKey(100);
        }

    }

    if( CalibrationImages.size() > 10){
        
        log(CalibrationImages.size());
        bool outout = cam::CameraParams(CalibrationImages, CHECKERBOARD, 1, imgSize, cameraMatrix,cameraDistCoeffs);
        log("done");
        log("Saving camera params to file")
        bool saved = cam::saveCameraCalibration("./Cameras/" + std::to_string(camID), "calibration.xml", cameraMatrix, cameraDistCoeffs);
        log("saved");
    }
    
}