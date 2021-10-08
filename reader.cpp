#include "reader.hpp"

Reader::Reader(){

    x=0;
    y=0;
    r=0.f;
    min_value=0.f;
    max_value=0.f ;
    min_angle=0.f;
    max_angle=0.f ;
};

Mat Reader::readimg(string src_image_path){

    this->srcimg = imread(src_image_path, 1);
    return this->srcimg.clone();
};

Mat Reader::readimg(Mat &src_img,Vec4i x){
    Mat zhizhen = src_img(cv::Rect(int(x[0])-15,int(x[1])-15,int(x[2])-int(x[0])+30,int(x[3])-int(x[1])+30));
    this->srcimg = zhizhen;
    return this->srcimg.clone();
};

Mat Reader::readimg(Mat &src_img){
    this->srcimg = src_img;
    return this->srcimg.clone();
};


Vec3d Reader::avg_circles(vector<cv::Vec3f> circles, int b){
    int avg_x=0;
    int avg_y=0;
    int avg_r=0;
    for (int i=0;  i< b; i++ )
    {
        //平均圆心 半径
        avg_x = avg_x + circles[i][0];
        avg_y = avg_y + circles[i][1];
        avg_r = avg_r + circles[i][2];
    }
    //半径为啥int
    avg_x = int(avg_x/(b));
    avg_y = int(avg_y/(b));
    avg_r = int(avg_r/(b));

    Vec3d xyr = Vec3d(avg_x, avg_y, avg_r);
    return xyr;

}

float Reader::getDist_P2L(cv::Point2f pointP, cv::Point2f pointA, cv::Point2f pointB)
{
    float A = 0, B = 0, C = 0;
    A = pointA.y - pointB.y;
    B = pointB.x - pointA.x;
    C = pointA.x*pointB.y - pointA.y*pointB.x;

    float distance = 0.0;
    distance = ((float)abs(A*pointP.x + B*pointP.y + C)) / ((float)sqrtf(A*A + B*B));
    return distance;
}


float Reader::dist_2_pts(int x1, int y1, int x2, int y2){
    int pp = pow(x2-x1,2)+pow(y2-y1,2);
    float tmp = sqrt(pp);
    return tmp;
}
Mat Reader::region_of_interest(Mat &img,vector<vector<cv::Point>> &vertices){
    Mat mask = Mat::zeros(img.size(), img.type());
    int match_mask_color= 255;

    fillPoly(mask, vertices, Scalar(match_mask_color));

    Mat masked_image;
    bitwise_and(img, mask,masked_image);

    return masked_image.clone();;


}


Mat Reader::detectCircles() {
    if (this->srcimg.empty()) {
        cv::Mat a;
        return a;
    }
    Mat midd_img = this->srcimg.clone();
    int wight = midd_img.rows;
    int height = midd_img.cols;

    Mat gray_img;
    cvtColor(midd_img, gray_img, COLOR_RGB2GRAY);
    medianBlur(gray_img, gray_img, 5);

    std::vector<Vec3f> circles;

    HoughCircles(gray_img, circles, HOUGH_GRADIENT, 1, 120, 100, 50, int(height * 0.35),int(height * 0.48));

    int b = circles.size();
    if (b == 0) {
        cv::Mat a;
        return a;
    } else {
        Vec3d xyr = this->avg_circles(circles, b);

        this->x = xyr[0];
        this->y = xyr[1];
        this->r = xyr[2];


        float separation = 10.0;
        int interval = int(360 / separation);

        std::vector<Point> pts;

        for (int i = 0; i < interval; i++) {
            Point pp;
            for (int j = 0; j < 2; j++) {
                if (j % 2 == 0)
                    pp.x = this->x + 1.0 * r * cos(separation * i * CV_PI / 180);
                else
                    pp.y = this->y + 1.0 * r * sin(separation * i * CV_PI / 180);
            }
            pts.push_back(pp);
        }

        Mat canny;
        Canny(gray_img, canny, 200, 20);


        std::vector<std::vector<Point>> region_of_interest_vertices;
        region_of_interest_vertices.push_back(pts);

        Mat cropped_image = region_of_interest(canny, region_of_interest_vertices);


        std::vector<std::vector<Point>> contours;
        std::vector<Vec4i> hierarchy;

        //findContours(cropped_image,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point());
        findContours(cropped_image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

        vector<vector<cv::Point> > int_cnt;

        for (int i = 0; i < contours.size(); i++) {
            float area = contourArea(contours[i]);
            Rect prect = boundingRect(contours[i]);

            float cpd = dist_2_pts(prect.x + prect.width / 2, prect.y + prect.height / 2, this->x,this->y);

            if ((area < 500) && (cpd < this->r * 4 / 4) && (cpd > this->r * 2 / 4)) {
                //drawContours(contours3, vector<vector<Point> >(1, contours[i]), -1,Scalar(255, 0, 0), 3);
                int_cnt.push_back(contours[i]);
            }
        }


        float reference_zero_angle = 20;
        float reference_end_angle = 340;
        float min_angle = 90;
        float max_angle = 270;

        std::vector<int> frth_quad_index;
        std::vector<int> thrd_quad_index;
        std::vector<float> frth_quad_angle;
        std::vector<float> thrd_quad_angle;

        for (int i = 0; i < int_cnt.size(); i++) {
            
            vector<cv::Point> conPoints;
            float x1, y1;
            float sx1 = 0, sy1 = 0;
            for (int j = 0; j < contours[i].size(); j++) {
                //绘制出contours向量内所有的像素点
                sx1 += contours[i][j].x;
                sy1 += contours[i][j].y;
            }
            x1 = sx1 / contours[i].size();
            y1 = sy1 / contours[i].size();

            float xlen = x1 - this->x;
            float ylen = this->y - y1;


            if ((xlen < 0) && (ylen < 0)) {
                double res = atan2(abs(float(ylen)), abs(float(xlen)));
                res = res * 180.0 / M_PI;
                float final_start_angle = 90 - res;

                frth_quad_index.push_back(i);
                frth_quad_angle.push_back(final_start_angle);

                if (final_start_angle > reference_zero_angle)
                    if (final_start_angle < min_angle)
                        min_angle = final_start_angle;

            }

            if ((xlen > 0) && (ylen < 0)) {
                double res = atan2(abs(float(ylen)), abs(float(xlen)));
                res = res * 180.0 / M_PI;
                float final_end_angle = 270 + res;

                thrd_quad_index.push_back(i);
                thrd_quad_angle.push_back(final_end_angle);

                if (final_end_angle < reference_end_angle)
                    if (final_end_angle > max_angle)
                        max_angle = final_end_angle;
            }

        }


        std::vector<float> frth_angle_(frth_quad_angle);
        std::vector<float> thrd_angle_(thrd_quad_angle);

        //升序 ，，降序
        std::sort(frth_angle_.begin(), frth_angle_.end(), std::less<float>());
        std::sort(thrd_angle_.begin(), thrd_angle_.end(), std::greater<float>());

        std::vector<float> frth_sub;
        std::vector<float> thrd_sub;
        for (int i = 0; i < frth_angle_.size() - 1; i++)
            frth_sub.push_back(frth_angle_[i + 1] - frth_angle_[i]);
        for (int i = 0; i < thrd_angle_.size() - 1; i++)
            thrd_sub.push_back(thrd_angle_[i + 1] - thrd_angle_[i]);


        std::vector<float>::iterator maxPosition = max_element(frth_sub.begin(), frth_sub.end());
        min_angle = frth_angle_[maxPosition - frth_sub.begin() + 1];
        //min_angle = *(max_element(frth_sub.begin(), frth_sub.end())+1);

        std::vector<float>::iterator minPosition = min_element(thrd_sub.begin(), thrd_sub.end());
        max_angle = thrd_angle_[minPosition - thrd_sub.begin() + 1];

        this->min_angle = min_angle;
        this->max_angle = max_angle;

        return midd_img;
    }
}

float Reader::detectLines() {
        Mat gray_img;
        if((this->min_angle==0) || (this->max_angle==0)) return float(10086.111f);
        if(this->srcimg.empty()) return float(10086.111f);
        Mat midd_img = this->srcimg.clone();

        cvtColor(midd_img, gray_img, COLOR_RGB2GRAY);

        int thresh = 166;
        int maxValue = 255;
        Mat midd_img2;


        vector<Vec4i> mylines;
        int g_nthreshold = 39;


        threshold(gray_img, midd_img2, thresh, maxValue, CV_THRESH_BINARY_INV);
        //imwrite("midd_img2.jpg", midd_img2);
        HoughLinesP(midd_img2, mylines, 3, CV_PI / 180, 100, 10, 0);
        //HoughLinesP(gray_img, mylines, 3, CV_PI / 180, 100, 10, 0);
        if (mylines.empty()) return float(10086.111f);
        Point circle_center = Point2f(this->x, this->y);
        float circle_radius = this->r;


        float diff1LowerBound = 0.05;
        float diff1UpperBound = 0.25;
        float diff2LowerBound = 0.05;
        float diff2UpperBound = 1.0;


        vector<Vec4i> final_line_list;
        vector<float> distance_list;
        vector<float> line_length_list;
        Mat midd_img6 = midd_img.clone();

        for (size_t i = 0; i < mylines.size(); i++) {
            Vec4i l = mylines[i];
            float diff1 = this->dist_2_pts(circle_center.x, circle_center.y, l[0], l[1]);
            float diff2 = this->dist_2_pts(circle_center.x, circle_center.y, l[2], l[3]);
            if (diff1 > diff2) {
                float temp = diff1;
                diff1 = diff2;
                diff2 = temp;
            }

            if (((diff1 < diff1UpperBound * circle_radius) && (diff1 > diff1LowerBound * circle_radius)) &&
                ((diff2 < diff2UpperBound * circle_radius) && (diff2 > diff2LowerBound * circle_radius))) {

                float line_length = this->dist_2_pts(l[0], l[1], l[2], l[3]);
                float distance = this->getDist_P2L(Point2f(circle_center.x, circle_center.y), Point2f(l[0], l[1]),
                                             Point2f(l[2], l[3]));

                if ((line_length>0.1*circle_radius)  && (distance>-20) && (distance <10)){
                final_line_list.push_back(Vec4i(l[0], l[1], l[2], l[3]));
                distance_list.push_back(distance);
                line_length_list.push_back(line_length);
                //cv::line(midd_img6, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(23, 180, 55), 2,CV_AA);
                }

            }

        };
        //imwrite("midd_img6.jpg", midd_img6);
        if (final_line_list.empty()) return float(10086.111f);
        vector<float>::iterator maxPosition = max_element(line_length_list.begin(),line_length_list.end());

        vector<float>::iterator minPosition = min_element(distance_list.begin(),distance_list.end());

        Vec4i final_line;

        final_line = final_line_list[maxPosition - line_length_list.begin() + 1];


        float x1 = final_line[0];
        float y1 = final_line[1];
        float x2 = final_line[2];
        float y2 = final_line[3];


        //find the farthest point from the center to be what is used to determine the angle
        float dist_pt_0 = this->dist_2_pts(circle_center.x, circle_center.y, x1, y1);
        float dist_pt_1 = this->dist_2_pts(circle_center.x, circle_center.y, x2, y2);

        float x_angle = 0.0;
        float y_angle = 0.0;
        if (dist_pt_0 > dist_pt_1) {
            x_angle = x1 - circle_center.x;
            y_angle = circle_center.y - y1;
        } else {
            x_angle = x2 - circle_center.x;
            y_angle = circle_center.y - y2;
        }

        x_angle = (x1 + x2) / 2 - circle_center.x;
        y_angle = circle_center.y - (y1 + y2) / 2;


        double res = atan2(float(y_angle), float(x_angle));

        //these were determined by trial and error
        res = res * 180.0 / M_PI;

        float final_angle = 0.0;

        if ((x_angle > 0) && (y_angle > 0))//in quadrant I
            final_angle = 270 - res;
        if (x_angle < 0 && y_angle > 0) //in quadrant II
            final_angle = 90 - res;
        if (x_angle < 0 && y_angle < 0)  //in quadrant III
            final_angle = 90 - res;
        if (x_angle > 0 && y_angle < 0)  //in quadrant IV
            final_angle = 270 - res;


            float old_min = float(this->min_angle) ;
            float old_max = float(this->max_angle) ;

            float new_min = float( this->min_value);
            float new_max = float( this->max_value);

            float old_value = final_angle;

            float old_range = (old_max - old_min);
            float new_range = (new_max - new_min);
            float final_value = (((old_value - old_min) * new_range) / old_range) + new_min;

        if ((final_value< float(this->min_value) ) || (final_value> float(this->max_value)))
            return float(10086.111f);

        return final_value;
    }
