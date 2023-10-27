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

        Line3D(cv::Mat dirMat, vector3 pos){
            vector3 d(1,0,0);
            this->dir = MatVecProduct(dirMat, d);
        }

        gty::vector3 MatVecProduct(cv::Mat mat, gty::vector3 vec){
            gty::vector3 output;
            output.x = mat.at<double>(0,0) * vec.x + mat.at<double>(1,0)*vec.x + mat.at<double>(2,0)*vec.x;
            output.y = mat.at<double>(0,1) * vec.y + mat.at<double>(1,1)*vec.y + mat.at<double>(2,1)*vec.y;
            output.z = mat.at<double>(0,2) * vec.z + mat.at<double>(1,2)*vec.z + mat.at<double>(2,2)*vec.z;

            return output;
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

    cv::Mat extractRotationMatrix(const cv::Mat& transformationMatrix) {
        // Extract the top-left 3x3 submatrix as the rotation matrix
        cv::Mat rotationMatrix = transformationMatrix(cv::Rect(0, 0, 3, 3));

        return rotationMatrix;
    }

    cv::Mat extractTranslationVector(const cv::Mat& transformationMatrix) {
        // Extract the rightmost column (elements at index 3) as the translation vector
        cv::Mat translationVector = transformationMatrix(cv::Rect(3, 0, 1, 3));

        return translationVector;
    }

    cv::Mat averageTransformationMatrix(std::vector<cv::Mat>& matrices) {
        // Initialize the result matrix with zeros
        cv::Mat result = cv::Mat::zeros(4, 4, CV_64F);

        // Sum all the transformation matrices
        for (const cv::Mat& matrix : matrices) {
            result += matrix;
        }

        // Calculate the average by dividing by the number of matrices
        result /= static_cast<double>(matrices.size());

        return result;
    }

    double scale(double valueIn, double baseMin, double baseMax, double limitMin, double limitMax) {
        return ((limitMax - limitMin) * (valueIn - baseMin) / (baseMax - baseMin)) + limitMin;
    }

    double Dist(cv::Point3d p1, cv::Point3d p2){
        return std::sqrt((p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y)+(p2.z-p1.z)*(p2.z-p1.z));
    }

    double solveLambda(gty::vector3 pos, gty::vector3 dir){
        return (dir.x*pos.x-dir.y*pos.y-dir.z*pos.z)/(dir.x*dir.x+dir.y*dir.y + dir.z*dir.z);
    }

    double stepSize(double current, double max){
        double scaled = scale(current, 0, max, 0, 1);
        //return -logWithBase(50, scaled) * 10;
        return -1*scaled + 1;
    }

    cv::Point3d ClosestPoint(gty::Line3D line, cv::Point3d point){
        double lambda = solveLambda(line.pos, line.dir);
        cv::Point3d solvedPoint;
        solvedPoint.x = line.pos.x+lambda*line.dir.x;
        solvedPoint.y = line.pos.y+lambda*line.dir.y;
        solvedPoint.z = line.pos.z+lambda*line.dir.z;
        return solvedPoint;
    }

    std::vector<gty::vector3> dirs = {  
                                        gty::vector3(1,0,0),
                                        gty::vector3(-1,0,0), 
                                        gty::vector3(0,1,0),
                                        gty::vector3(0,-1,0),
                                        gty::vector3(0,0,1),
                                        gty::vector3(0,0,-1),
                                    };

    gty::vector3 MatVecProduct(cv::Mat mat, gty::vector3 vec){
        gty::vector3 output;
        output.x = mat.at<double>(0,0) * vec.x + mat.at<double>(1,0)*vec.x + mat.at<double>(2,0)*vec.x;
        output.y = mat.at<double>(0,1) * vec.y + mat.at<double>(1,1)*vec.y + mat.at<double>(2,1)*vec.y;
        output.z = mat.at<double>(0,2) * vec.z + mat.at<double>(1,2)*vec.z + mat.at<double>(2,2)*vec.z;

        return output;
    }

        void EstimateIntersection(std::vector<gty::Line3D> lines, cv::Point3d& point, double& dist){
        cv::Point3d movingPoint = cv::Point3d(1000,1000,1000);

        int iterations = 500;

        double step = 0.02;

        double bestLenght = std::numeric_limits<double>::max();

        for (int i = 0; i < iterations; i++)
        {
            gty::vector3 bestDir;
            bestLenght = std::numeric_limits<double>::max();

            for (int j = 0; j < dirs.size(); j++)
            {
                cv::Point3d np = movingPoint + step*dirs[j].toCVPoint();

                double distSum = 0;

                for (int k = 0; k < lines.size(); k++)
                {
                    cv::Point3d lp = ClosestPoint(lines[k], np);
                    distSum += Dist(np, lp);
                }

                if(bestLenght > distSum){
                    bestDir = dirs[j];
                    bestLenght = distSum;
                }   
            }   

            //logi(bestDir.to_string());
            
            step = stepSize(i + 10, iterations + 10);
            movingPoint += step*bestDir.toCVPoint();
            //logi(i << " : " << movingPoint << " : " << bestLenght);
        }
        point = movingPoint;
        dist = bestLenght;
    }
    
}