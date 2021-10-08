#ifndef READER_H
#define READER_H
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include<opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<math.h>

#include <cmath>
#include <cmath>
#include <algorithm>
#include <functional>
#include <array>
#include <iostream>
#include<numeric>
#include <vector>

using namespace std;
using namespace cv;



const float min_angle = 50;
const float max_angle = 320;
const float min_value = 0;
const float max_value = 2.5;
const string units = "MPa";

class Reader
{
public:
    cv::Mat readimg(string src_image_path);
    cv::Mat readimg(cv::Mat &src_img,cv::Vec4i x);
    cv::Mat readimg(cv::Mat &src_img);
    cv::Vec3d avg_circles(vector<cv::Vec3f> circles, int b);
    float dist_2_pts(int x1, int y1, int x2, int y2);
    cv::Mat detectCircles();
    float detectLines();
    float getDist_P2L(cv::Point2f pointP, cv::Point2f pointA, cv::Point2f pointB);
    cv::Mat region_of_interest(cv::Mat &img,vector<vector<cv::Point>> &vertices);
     Reader();   

public:
    int x;
    int y;
    float r;
    cv::Mat srcimg;
    cv::Vec4i line;
public:
    float min_value;
    float max_value ;
    float min_angle;
    float max_angle ;

};

#endif
