#pragma once
#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <regex>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <cmath>
#include <string>
#include "tinyxml2.h"

#define logi(smth) std::cout << smth << std::endl;

namespace gty{
    class Line2D{
        public: double a, c;

        public: Line2D(double slope, double intercept)
        {
            a = slope;
            c = intercept;
        }

        public: Line2D(){
            a = 0;
            c = 0;
        }

        public: double valueY(double x){
            return a*x+c;
        }

        public: double valueX(double y){
            return (-y+c)/-a;
        }
    };

    class vector2{
        public: double x,y;

        vector2(cv::Point2d p){
            x=p.x;
            y=p.y;
        }
    };

        class vector3{
        public: double x,y,z;

        public: vector3(cv::Point3d point) {
            this->x = point.x;
            this->y = point.y;
            this->z = point.z;
        }

        public: vector3(cv::Vec3d point) {
            this->x = point[0];
            this->y = point[1];
            this->z = point[2];
        }

        public: vector3(double x, double y, double z) {
            this->x = x;
            this->y = y;
            this->z = z;
        }

        public: vector3() {
            this->x = 0;
            this->y = 0;
            this->z = 0;
        }

        public: vector3(const cv::Mat& mat) {
            if (mat.rows == 3 && mat.cols == 1) {
                this->x = mat.at<double>(0, 0);
                this->y = mat.at<double>(1, 0);
                this->z = mat.at<double>(2, 0);
            } else {
                // Handle invalid input (e.g., set x, y, z to 0)
                this->x = 0;
                this->y = 0;
                this->z = 0;
            }
        }

        public: cv::Point3d toCVPoint(){
            return cv::Point3d(x,y,z);
        }

        public: cv::Mat toCVVector(){
            return (cv::Mat_<double>(3, 1) << x, y, z);
        }

                
        public: std::string toJson(){
            //this is supported by Unity
            std::stringstream s;
            s << "{\"x\": " << x << ",\"y\":" << y << ",\"z\":" << z << "}"; 
            return s.str();
        }

        public: cv::Mat rotMat(){
            cv::Mat ret;
            cv::Rodrigues(toCVVector(), ret);
            return ret;
        }

        public: double magnitude(){
            double magnitude = 0.0;
            magnitude = this->x*this->x + this->y*this->y + this->z*this->z;
            return std::sqrt(magnitude);
        }

        public: vector3 fromRotationMatrix(const cv::Mat rotationMatrix) {
            // Ensure the rotationMatrix is a 3x3 matrix
            if (rotationMatrix.rows == 3 && rotationMatrix.cols == 3) {
                // Convert the rotation matrix to a Rodrigues vector
                cv::Vec3d rodrigues;
                cv::Rodrigues(rotationMatrix, rodrigues);

                // Assign the Rodrigues vector components to x, y, and z
                this->x = rodrigues[0];
                this->y = rodrigues[1];
                this->z = rodrigues[2];
            } 
            return *this;
        }

        public: friend std::ostream& operator<<(std::ostream& os, const vector3 vec) {
            os << "[" << vec.x << ", " << vec.y << ", " << vec.z << "]";
            return os;
        }

        vector3 operator/(double& divisor){
            if(divisor != 0){
                return vector3(x / divisor, y / divisor, z / divisor);
            }else{
                return *this;
            }
        }

        vector3 operator/(int& divisor){
            if(divisor != 0){
                return vector3(x / divisor, y / divisor, z / divisor);
            }else{
                return *this;
            }
        }

        vector3 operator/(unsigned long& divisor){
            if(divisor != 0){
                return vector3(x / divisor, y / divisor, z / divisor);
            }else{
                return *this;
            }
        }

        public: vector3 operator+(vector3& vec){
            vector3 val;
            val.x = this->x+vec.x;
            val.y = this->y+vec.y;
            val.z = this->z+vec.z;

            return val;
        }

        public: vector3 operator+=(vector3 vec){
            *this= *this+vec;
            return *this;
        }

        public: vector3 operator/=(double vec){
            *this= *this/vec;
            return *this;
        }

        public: vector3 operator/=(int vec){
            *this= *this/vec;
            return *this;
        }

        public: vector3 operator/=(unsigned long val){
            *this= *this/val;
            return *this;
        }

        // Overload the << operator to print the vector
        public: std::string toString() {
            std::string s =  "[" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + "]";
            return s;
        }

        // Overload the >> operator to load from an XML file
        friend std::istream& operator>>(std::istream& is, vector3& v) {
            tinyxml2::XMLDocument doc;
            if (doc.LoadFile("vector3.xml") != tinyxml2::XML_SUCCESS) {
                std::cerr << "Failed to load XML file." << std::endl;
                return is;
            }

            tinyxml2::XMLElement* root = doc.FirstChildElement("vector3");
            if (!root) {
                std::cerr << "Invalid XML format." << std::endl;
                return is;
            }

            root->FirstChildElement("x")->QueryDoubleText(&v.x);
            root->FirstChildElement("y")->QueryDoubleText(&v.y);
            root->FirstChildElement("z")->QueryDoubleText(&v.z);

            return is;
        }
    };

    class Line3D{
        public: vector3 dir, pos;

        public: Line3D(vector3 dir, vector3 pos){
            this->dir = dir;
            this->pos = pos;
        }

        std::string toJson(){
            std::stringstream s;
            s << "{\"rvec\":" << dir.toJson() << ",\"tvec\": " << pos.toJson() << "}";
            return s.str();
        }
    };

}

namespace tri{
    cv::Mat rvecToMat(cv::Mat rvec){
        cv::Mat mat;
        cv::Rodrigues(rvec, mat);
        return mat;
    }

    cv::Mat createTransformationMatrix(const cv::Mat& rotationMatrix, const cv::Mat& translationVector) {
        cv::Mat transformationMatrix = cv::Mat::eye(4, 4, CV_64F);

        // Copy the rotation matrix into the top-left 3x3 block of the transformation matrix
        cv::Mat rotationBlock = transformationMatrix(cv::Rect(0, 0, 3, 3));
        rotationMatrix.copyTo(rotationBlock);

        // Copy the translation vector into the rightmost column of the transformation matrix
        cv::Mat translationBlock = transformationMatrix(cv::Rect(3, 0, 1, 3));
        translationVector.copyTo(translationBlock);

        return transformationMatrix;
    }

    gty::vector3 MatVecProduct(cv::Mat mat, gty::vector3 vec){
        gty::vector3 output;
        output.x = mat.at<double>(0,0) * vec.x + mat.at<double>(1,0)*vec.x + mat.at<double>(2,0)*vec.x;
        output.y = mat.at<double>(0,1) * vec.y + mat.at<double>(1,1)*vec.y + mat.at<double>(2,1)*vec.y;
        output.z = mat.at<double>(0,2) * vec.z + mat.at<double>(1,2)*vec.z + mat.at<double>(2,2)*vec.z;

        return output;
    }
    
}