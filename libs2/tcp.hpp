#pragma once

#include <opencv2/core/mat.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <thread>
#include <csignal>
#include <iostream>
#include <vector>

#include "./Triangulation.hpp"
#include "./Camera.hpp"


//#include "./Camera.hpp"
#define log(smth) std::cout << smth << std::endl;

namespace tcp{

    class Scene{
        public: 
        std::vector<cv::Mat> camTransformation;
        gty::vector3 intersection;
        std::vector<gty::Line3D> lines;

        Scene(){
            camTransformation = std::vector<cv::Mat>();
            intersection = gty::vector3();
            lines = std::vector<gty::Line3D>();
        }

        void setScene(std::vector<cam::Camera> cameras, gty::vector3 intersection, std::vector<gty::Line3D> lines){

            camTransformation = std::vector<cv::Mat>();
            
            
            for (int i = 0; i < cameras.size(); i++)
            {
                camTransformation.push_back(cv::Mat::eye(4, 4, CV_64F));
                
                cv::Mat rotationMatrix = cameras[i].rMat;
                // Copy the rotation matrix to the upper-left 3x3 block
                rotationMatrix.copyTo(camTransformation[i](cv::Rect(0, 0, 3, 3)));
                // Copy the translation vector to the right-most column
                cameras[i].tMat.toCVVector().copyTo(camTransformation[i](cv::Rect(3, 0, 1, 3)));

                // Set the bottom row to [0 0 0 1]
                camTransformation[i].at<double>(3, 3) = 1.0;
            }
            

            this->intersection = intersection;
            this->lines = lines;

            //log(this->toJson());
        }

        std::string toJson() {
            std::stringstream json;
            json << "{";

            // Convert camTransform array to JSON
            json << "\"camTransform\": [";
            for (size_t i = 0; i < camTransformation.size(); i++) {
                if (i != 0) {
                    json << ',';
                }
                json << "{";
                for (int row = 0; row < camTransformation[i].rows; row++) {
                    for (int col = 0; col < camTransformation[i].cols; col++) {
                        if (row != 0 || col != 0) {
                            json << ',';
                        }
                        json << "\"data" << row << "-" << col << "\": " << camTransformation[i].at<double>(row, col);
                    }
                }
                json << "}";
            }
            json << "],";

            // Add other properties to the JSON
            json << "\"intersection\":" << intersection.toJson();
            json << ",\"lines\": [";
            for (size_t i = 0; i < lines.size(); i++) {
                if (i != 0) {
                    json << ',';
                }
                json << lines[i].toJson();
            }
            json << "]";

            json << "}";

            return json.str();
        }

    };

    class tcp_server
    {
        
        public: Scene scene;
        private:
        std::thread thread_obj; // Store the thread as a member variable.
        int sockfd;


        std::string convertToString(char* a, int size)
        {
            int i;
            std::string s = "";
            for (i = 0; i < size; i++) {
                s = s + a[i];
            }
            return s;
        }

        void error(const char *msg)
        {
            perror(msg);
            exit(1);
        }

        private: static void signalHandler(int signum, int sockfd) {
            if (signum == SIGINT) {
                // Handle Ctrl+C: Close the server and clean up
                close(sockfd);
                exit(signum);
            }
        }

        public: void Server(int portno)
        {
            log("starting server");
            int newsockfd;
            socklen_t clilen;
            char buffer[256];
            struct sockaddr_in serv_addr, cli_addr;
            int n;
            sockfd = socket(AF_INET, SOCK_STREAM, 0);
            if (sockfd < 0) 
                error("ERROR opening socket");

            bzero((char *) &serv_addr, sizeof(serv_addr));
            serv_addr.sin_family = AF_INET;
            serv_addr.sin_addr.s_addr = INADDR_ANY;
            serv_addr.sin_port = htons(portno);
            if (bind(sockfd, (struct sockaddr *) &serv_addr,
                    sizeof(serv_addr)) < 0) 
                    error("ERROR on binding");
            
            int yes = 1;
            setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));
            listen(sockfd,5);


            while (true)
            {
                clilen = sizeof(cli_addr);
                newsockfd = accept(sockfd, 
                            (struct sockaddr *) &cli_addr, 
                            &clilen);
                if (newsockfd < 0) 
                    error("ERROR on accept");
                bzero(buffer,256);
                n = read(newsockfd,buffer,255);
                if (n < 0) error("ERROR reading from socket");
        
                std::string unclean = std::string(buffer);
                unclean.erase(std::remove(unclean.begin(), unclean.end(), '\n'), unclean.cend());
                
                log(unclean.size() << " " << std::string("pos").size())

                if (unclean == "pos")
                {
                    log("sending scene");
                    std::string m = scene.toJson();
                    log(m);
                    n = write(newsockfd,m.c_str(),m.size());
                }    

                if (n < 0) error("ERROR writing to socket");  
            }
            
            close(sockfd);

        }

        void Thread(int port){
            thread_obj = std::thread(&tcp::tcp_server::Server, this, port);
        }
            
    };
}
