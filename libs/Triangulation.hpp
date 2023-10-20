
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

#define logi(smth) std::cout << smth << std::endl;

namespace gty
{
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

    class vector3{
        public: double x,y,z;

        vector3(cv::Point3d point) {
            this->x = point.x;
            this->y = point.y;
            this->z = point.z;
        }

        vector3(cv::Vec3d point) {
            this->x = point[0];
            this->y = point[1];
            this->z = point[2];
        }

        vector3(double x, double y, double z) {
            this->x = x;
            this->y = y;
            this->z = z;
        }

        vector3() {
            this->x = 0;
            this->y = 0;
            this->z = 0;
        }

        vector3 fromCVMat(const cv::Mat& mat) {
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
            return *this;
        }

        cv::Point3d toCVPoint(){
            return cv::Point3d(x,y,z);
        }

        cv::Mat toCVVector(){
            return (cv::Mat_<double>(3, 1) << x, y, z);
        }

        cv::Mat RotMat(){
            cv::Mat ret;
            cv::Rodrigues(toCVVector(), ret);
            return ret;
        }

        vector3(const cv::Mat& rotationMatrix) {
            // Ensure the rotationMatrix is a 3x3 matrix
            if (rotationMatrix.rows == 3 && rotationMatrix.cols == 3) {
                // Convert the rotation matrix to a Rodrigues vector
                cv::Vec3d rodrigues;
                cv::Rodrigues(rotationMatrix, rodrigues);

                // Assign the Rodrigues vector components to x, y, and z
                this->x = rodrigues[0];
                this->y = rodrigues[1];
                this->z = rodrigues[2];
            } else {
                // Handle the case where the input matrix is not a valid rotation matrix
                // You could throw an exception or handle it in your preferred way.
                // For this example, I'm setting x, y, and z to 0 in case of an invalid matrix.
                this->x = 0;
                this->y = 0;
                this->z = 0;
            }
        }

        friend std::ostream& operator<<(std::ostream& os, const vector3 vec) {
            os << "[" << vec.x << ", " << vec.y << ", " << vec.z << "]";
            return os;
        }

        vector3 operator/(double divisor){
            if(divisor != 0){
                return vector3(x / divisor, y / divisor, z / divisor);
            }else{
                return *this;
            }
        }
    };

    class Line3D{
        public: vector3 dir, pos;

        public: Line3D(vector3 dir, vector3 pos){
            this->dir = dir;
            this->pos = pos;
        }
    };

    class location{
        public: vector3 pos, rot;
    };

    class ray{
        public: vector3 camPos, camDir, RayDir;
    };
}


namespace tri
{

    double scale(double valueIn, double baseMin, double baseMax, double limitMin, double limitMax) {
        return ((limitMax - limitMin) * (valueIn - baseMin) / (baseMax - baseMin)) + limitMin;
    }

    double logWithBase(double base, double x) {
        return log(x) / log(base);  
    }

    double stepSize(double current, double max){
        double scaled = scale(current, 0, max, 0, 1);
        //return -logWithBase(50, scaled) * 10;
        return -1*scaled + 1;
    }

    double Dist(cv::Point3d p1, cv::Point3d p2){
        return std::sqrt((p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y)+(p2.z-p1.z)*(p2.z-p1.z));
    }

    double Dist(cv::Point2d p1, cv::Point2d p2){
        return std::sqrt((p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y));
    }

    gty::vector3 VecFrom2Point(cv::Point3d start, cv::Point3d end){
        return gty::vector3(end - start);
    }  

    double solveLambda(gty::vector3 pos, gty::vector3 dir){
        return (dir.x*pos.x-dir.y*pos.y-dir.z*pos.z)/(dir.x*dir.x+dir.y*dir.y, dir.z*dir.z);
    }

    cv::Vec3d rotationVectorToEulerAngles(const cv::Mat& rotationVector) {
        cv::Mat rotationMatrix;
        cv::Rodrigues(rotationVector, rotationMatrix);

        cv::Vec3d eulerAngles;
        cv::Mat rotationMatrixR = rotationMatrix(cv::Rect(0, 0, 3, 3));
        cv::Mat_<double> eulerMatrix = cv::Mat_<double>(rotationMatrixR);

        eulerAngles[0] = atan2(eulerMatrix(1, 2), eulerMatrix(2, 2)); // Roll
        eulerAngles[1] = asin(-eulerMatrix(0, 2)); // Pitch
        eulerAngles[2] = atan2(eulerMatrix(0, 1), eulerMatrix(0, 0)); // Yaw

        eulerAngles *= (180.0 / CV_PI);

        return eulerAngles;
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

    void EstimateIntersection(std::vector<gty::Line3D> lines, cv::Point3d *point, double *dist){
        cv::Point3d movingPoint = cv::Point3d(-45,32,63);

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
            
            step = stepSize(i + 10, iterations + 10);
            movingPoint += step*bestDir.toCVPoint();
            //logi(i << " : " << movingPoint << " : " << bestLenght);
        }
        *point = movingPoint;
        *dist = bestLenght;
    }
}
