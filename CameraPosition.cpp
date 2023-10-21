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

class CameraPos{
    public: gty::vector3 RotationVector, translationVector;

    CameraPos(cv::Mat cameraRotationVector, cv::Mat translationVector){
        this->RotationVector = cameraRotationVector;
        this->translationVector = translationVector;
    }

    CameraPos(){
        this->RotationVector = cv::Mat();
        this->translationVector = cv::Mat();
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

    int cvID;
    int camID;
    int maxIterations;

    CamFinder();

    log("give camera cv ID")
    std::cin >>  cvID;
    log("give camera ID")
    std::cin >> camID;
    log("how many iterations")
    std::cin >>maxIterations;

    cv::VideoCapture cap;
    cap.open(cvID);

    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    cv::Mat cameraMatrix,distCoeffs;
    cam::loadCameraCalibration(std::to_string(camID), cameraMatrix, distCoeffs);

    cv::Mat frame;

    std::vector<CameraPos> camerapositions;
    bool est = false;

    int iterations;

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

        if(iterations == maxIterations) break;

        if (est) {
            cv::Mat rvec, tvec;
            bool found = cam::CamPosARUCO(frame, cameraMatrix, distCoeffs, rvec, tvec);
            
            if(!found) continue;

            camerapositions.push_back(CameraPos(rvec, tvec));

            iterations++;
        }
    }

    CameraPos average;

    for (int i; i < camerapositions.size(); i++) {
        average.RotationVector += camerapositions[i].RotationVector;
        average.translationVector += camerapositions[i].translationVector;
    }

    average.RotationVector /= camerapositions.size();
    average.translationVector /= camerapositions.size();

    for (int i; i < camerapositions.size(); i++) {
        double similarityRot = tri::similarity(average.RotationVector, camerapositions[i].RotationVector);
        double similarityTra = tri::similarity(average.translationVector, camerapositions[i].translationVector);
        log(similarityRot << " : " << similarityTra);

        if (similarityRot < 0.8 || similarityTra < 0.8) {
            camerapositions.erase(camerapositions.begin() + i);
        }
    }

    for (int i; i < camerapositions.size(); i++) {
        average.RotationVector += camerapositions[i].RotationVector;
        average.translationVector += camerapositions[i].translationVector;
    }

    average.RotationVector /= camerapositions.size();
    average.translationVector /= camerapositions.size();

    cam::saveCameraPosition(std::to_string(camID), average.RotationVector.toCVVector(), average.RotationVector.toCVVector());
    log("position and rotation of camera saved!");
    
}   