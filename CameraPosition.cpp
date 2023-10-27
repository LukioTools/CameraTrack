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

#include "./libs2/Camera.hpp"

#define log(smth) std::cout << smth << std::endl;

class CameraPos{
    public: cv::Mat transformationMatrix;

    CameraPos(cv::Mat transformationMatrix){
        this->transformationMatrix = transformationMatrix;
    }

    CameraPos(){
        this->transformationMatrix = cv::Mat();
    }
};

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

    int camID;
    int maxIterations;

    double scale;

    CamFinder();

    log("give camera ID")
    std::cin >> camID;
    log("give size of the aruco marker (mm)")
    std::cin >> scale;
    log("how many iterations")
    std::cin >>maxIterations;

    cv::VideoCapture cap;
    cap.open(camID);

    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    cv::Mat cameraMatrix,distCoeffs;
    cam::loadCameraCalibration(std::to_string(camID), cameraMatrix, distCoeffs);

    cv::Mat frame;

    std::vector<CameraPos> camerapositions;
    bool est = false;

    int iterations = 0;

    for(;;){
        cap.read(frame);

        if (frame.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";
            break;
        }

        cv::imshow("frame", frame);

        int key = cv::waitKey(1);

        if (key == 13) {
            est = !est;
            
            if(est){
                log("starting position estimation")
            }else {
                log("ended position estimation")
            }
        }

        if (key == 8) {
            log("exiting position estimation")
            break;
        }

        if(iterations >= maxIterations){log("done finding positions"); break;}

        if (est) {
            cv::Mat transformationMatrix;
            bool found = cam::camARUCOPos(frame, cameraMatrix, distCoeffs, transformationMatrix);
            if(!found) continue;
            camerapositions.push_back(CameraPos(transformationMatrix));
            log(iterations << "/" << maxIterations);
            iterations++;
        }
    }

    CameraPos average;

    log("size: " << camerapositions.size());

    std::vector<cv::Mat> Transformations;

    for (CameraPos average: camerapositions)
    {
        Transformations.push_back(average.transformationMatrix);
    }

    cv::Mat a = tri::averageTransformationMatrix(Transformations);
    

    cam::saveCameraPosition(std::to_string(camID), tri::extractRotationMatrix(a), tri::extractTranslationVector(a));
    log("position and rotation of camera saved!");
}   