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

#include "./libs2/Camera.hpp"
#include "./libs2/tcp.hpp"

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

    tcp::tcp_server tcpServer;
    tcpServer.Thread(8080);

    std::vector<cam::Camera> cams;

    CamFinder();

    log("input camera id's. use one at time and leave empty when all cameras are set");

    do {
        std::string input;
        std::cout << "Enter a number (or leave it empty to exit): ";
        std::getline(std::cin, input);

        if (input.empty()) {
            break;  // Exit the loop if the input is empty
        }

        try {

            int camId = std::stoi(input);

            cv::VideoCapture testCap;
            testCap.open(camId);

            if (!testCap.isOpened())
            {
               log("Camera not found");
               continue;
            }

            testCap.release();


            log("Camera found. Loading camera preferences...");

            cv::Mat cameraMatrix, cameraDistCoeffs, CameraRot;
            gty::vector3 cameraPos;

            if(!cam::loadCameraCalibration(std::to_string(camId), cameraMatrix, cameraDistCoeffs)) {
                log("Camera prefrences not found");
                continue;
            }

            if(!cam::loadCameraPosition(std::to_string(camId), CameraRot, cameraPos)){
                log("Camera position not found");
                continue;
            }

            cams.push_back(cam::Camera(camId, cameraMatrix, cameraDistCoeffs, CameraRot.clone(), cameraPos, cv::VideoCapture()));
            cams[cams.size()-1].cap.open(camId);

            log("Camera settings loaded up successfully");
            

        } catch (std::invalid_argument const &ex) {
            std::cerr << "Invalid input. Please enter a valid number." << std::endl;
        }
    } while (true);

    while (true)
    {

        std::vector<gty::Line3D> lines;

        for (int i = 0; i < cams.size(); i++)
        {
            
            cv::Mat frame = cv::Mat();
            cams[i].cap.read(frame);
            
            cv::Mat transformationMatrix;

            cv::Point2f cord;

            bool found = cam::camARUCOPos(frame, cams[i].cameraMatrix, cams[i].DistCoeffs, transformationMatrix, &cord);
            
            if (found)
            {
                //log("Camera matrix: " << cams[i].cameraMatrix << "DistCoeffs: " << cams[i].DistCoeffs << " position: " << ap << " rotation: " << ar);
                //cv::drawFrameAxes(frame, cams[i].cameraMatrix, cams[i].DistCoeffs, -ar, -ap, 1);
                
                gty::vector3 localRotVec;
                localRotVec.x = 5.6*(cord.x/640);
                localRotVec.y = 4.2*(cord.y/480);
                localRotVec.z = 3.649;
                
                //log(cams[i].rMat << " : " << localRotVec.toString());
                cv::Mat rotMat = tri::MatVecProduct(cams[i].rMat, localRotVec).rotMat();
                
                //log("rotation matrix: " << rotationMatrix);
                //log("rotation vector: " << gty::vector3().fromRotationMatrix(rotationMatrix).to_string());
                lines.push_back(gty::Line3D(rotMat, cams[i].tMat));
                //log(lines[i].dir.to_string());

                cv::drawFrameAxes(frame, cams[i].cameraMatrix, cams[i].DistCoeffs, -gty::vector3().fromRotationMatrix(tri::extractRotationMatrix(transformationMatrix)).toCVVector(), -tri::extractTranslationVector(transformationMatrix), 1);
                cv::imshow(std::to_string(i), frame);
            }
        cv::Point3d Intresection;
        double distance;
        
        if(lines.size() == cams.size()){
        tri::EstimateIntersection(lines, Intresection, distance);
        
        //log("position: " << Intresection << " distance: " << distance);
        tcpServer.scene.setScene(cams, gty::vector3(Intresection),lines);
        
        }
        cv::waitKey(1);
        
        }
    }
}