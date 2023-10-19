
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace Geometry
{
    class Line2D{
        public: float a, c;

        public: Line2D(float slope, float intercept)
        {
            this->a = slope;
            this->c = intercept;
        }

        public: Line2D(){
            this->a = 0;
            this->c = 0;
        }

        public: float valueY(float x){
            return a*x+c;
        }

        public: float valueX(float y){
            return (-y+c)/-a;
        }
    };
}


namespace tri
{

    cv::Point3f VecFrom2Point(cv::Point3f start, cv::Point3f end){
        return end - start;
    }  

    float PerpendicularSlope(float k){
        return -1/k;
    }

    cv::Point2f ClosestPoint2D(Geometry::Line2D line, cv::Point2f point){
        float k = PerpendicularSlope(line.a);

        float x = line.valueX(k*(x-point.x) + point.y);
        float y = line.valueY(x);

        cv::Point2f p = cv::Point2f(x,y);
        return p;
    }

    cv::Point3f ClosestPoint3D(Geometry::Line2D lineTopDown, Geometry::Line2D lineSide, cv::Point3f point){
        cv::Point2f TD, S;

        TD.x = point.x;
        TD.y = point.z;

        S.x = point.x;
        S.y = point.y;

        cv::Point2f TDP, SP;

        TDP = ClosestPoint2D(lineTopDown, TD);
        SP = ClosestPoint2D(lineSide, S);

        cv::Point3f Point3D = cv::Point3f(SP.x, SP.y, TDP.y);

        return Point3D;
    }

    void GradientDescent(){
        
    }
}
