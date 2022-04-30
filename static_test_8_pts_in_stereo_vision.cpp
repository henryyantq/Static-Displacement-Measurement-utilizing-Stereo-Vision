
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <cmath>
#define N 8
#define FOC_IN_PIX 928.334  // iPhone 13 Pro广角摄像头像素级焦距

using namespace std;
using namespace cv;

Rect* l_ROI = new Rect[N];
Rect* r_ROI = new Rect[N];

void proc(Mat &binary, Rect *rect, int size) {
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binary, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
    if (contours.size() > 0) {
        double *area = new double[contours.size()];
        for (size_t t = 0; t < contours.size(); t++)
            area[t] = contourArea(contours[static_cast<int>(t)]);
        sort(area, area + contours.size());
        for (size_t t = 0; t < contours.size(); t++)
            for (int i = 0; i < size; i++)
                if (contourArea(contours[static_cast<int>(t)]) == area[contours.size() - (i + 1)])
                    rect[i] = boundingRect(contours[static_cast<int>(t)]);
    }
    else
        for (int i = 0; i < size; i++)
            rect[i].x = rect[i].y = rect[i].width = rect[i].height = 0;
}

void getColorMask(Mat &src, Mat &dst, Mat &mask) {
    Mat mask1, mask2;
    cvtColor(src, dst, COLOR_BGR2HSV);
    inRange(dst, Scalar(0, 127, 0), Scalar(10, 255, 255), mask1);
    inRange(dst, Scalar(156, 43, 46), Scalar(180, 255, 255), mask2);
    mask = mask1 + mask2;
}

void getCenter(Point2f &center, Rect *rect, int size) {
    center.x = 0;
    center.y = 0;
    for (int i = 0; i < size; i++) {
        center.x += rect[i].x + rect[i].width / 2;
        center.y += rect[i].y + rect[i].height / 2;
    }
    center.x /= size;
    center.y /= size;
}

void getImgPlaneIndex(Mat &src, Rect *rect, vector<Point2f> &point, int size, String windowName) {
    for (int i = 0; i < size; i++) {
        Point pt;
        point[i].x = pt.x = rect[i].x + rect[i].width / 2;
        point[i].y = pt.y = rect[i].y + rect[i].height / 2;
        rectangle(src, rect[i], Scalar(0, 255, 0), 3, 8, 0);
        circle(src, pt, 1, Scalar(0, 255, 0), -1);
        cv::putText(src, to_string(i + 1), Point(pt.x, pt.y), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255, 0), 3);
    }
    namedWindow(windowName, WINDOW_AUTOSIZE);
    imshow(windowName, src);
    waitKey(0);
}

void getRealPlaneIndex(vector<Point2f> &point, int size) {
    for (int i = 0; i < size; i++) {
        cout << "Point" << i + 1 << ".x = ";
        cin >> point[i].x;
        cout << "Point" << i + 1 << ".y = ";
        cin >> point[i].y;
        cout << endl;
    }
}

int main() {
    Point2f l_centerImg_fir, l_centerImg_sec;
    Point2f r_centerImg_fir, r_centerImg_sec;
    double wldDist = 0;     // 公制位移
    Mat l_centerWld_fir = (Mat_<double>(3, 1) << 0, 0, 1);
    Mat l_centerWld_sec = (Mat_<double>(3, 1) << 0, 0, 1);
    Mat r_centerWld_fir = (Mat_<double>(3, 1) << 0, 0, 1);
    Mat r_centerWld_sec = (Mat_<double>(3, 1) << 0, 0, 1);
    vector<Point2f> l_imgPlane(N);
    vector<Point2f> l_realPlane(N);
    vector<Point2f> r_imgPlane(N);
    vector<Point2f> r_realPlane(N);
    Mat l_src_fir, l_dst_fir;
    Mat l_src_sec, l_dst_sec;
    Mat r_src_fir, r_dst_fir;
    Mat r_src_sec, r_dst_sec;
    Mat l_mask, r_mask;
    Mat l_homoMat, l_invertMat, r_homoMat, r_invertMat;
    double T = 50, f = FOC_IN_PIX, deltaZ;
    
    l_src_fir = imread("/Users/chenxuan/Desktop/毕业设计/test samples/l1.jpeg");
    l_src_sec = imread("/Users/chenxuan/Desktop/毕业设计/test samples/l2.jpeg");
    r_src_fir = imread("/Users/chenxuan/Desktop/毕业设计/test samples/r1.jpeg");
    r_src_sec = imread("/Users/chenxuan/Desktop/毕业设计/test samples/r2.jpeg");
    
    getColorMask(l_src_fir, l_dst_fir, l_mask);
    getColorMask(r_src_fir, r_dst_fir, r_mask);
    proc(l_mask, l_ROI, N);
    proc(r_mask, r_ROI, N);
    getCenter(l_centerImg_fir, l_ROI, N);
    getCenter(r_centerImg_fir, r_ROI, N);
    circle(l_src_fir, l_centerImg_fir, 1, Scalar(255, 0, 0), -1);
    circle(r_src_fir, r_centerImg_fir, 1, Scalar(255, 0, 0), -1);
    getImgPlaneIndex(l_src_fir, l_ROI, l_imgPlane, N, "Left IL");
    getImgPlaneIndex(r_src_fir, r_ROI, r_imgPlane, N, "Right IL");
    getRealPlaneIndex(l_realPlane, N);
    getRealPlaneIndex(r_realPlane, N);
    l_homoMat = findHomography(l_realPlane, l_imgPlane);
    r_homoMat = findHomography(r_realPlane, r_imgPlane);
    l_invertMat = l_homoMat.inv();
    r_invertMat = r_homoMat.inv();
    cout << l_homoMat << endl << endl;
    cout << r_homoMat << endl << endl;
    Mat l_centerImg_fir_in_3d = (Mat_<double>(3, 1) << l_centerImg_fir.x, l_centerImg_fir.y, 1);
    Mat r_centerImg_fir_in_3d = (Mat_<double>(3, 1) << r_centerImg_fir.x, r_centerImg_fir.y, 1);
    l_centerWld_fir = l_invertMat * l_centerImg_fir_in_3d;
    r_centerWld_fir = r_invertMat * r_centerImg_fir_in_3d;
    
    getColorMask(l_src_sec, l_dst_sec, l_mask);
    getColorMask(r_src_sec, r_dst_sec, r_mask);
    proc(l_mask, l_ROI, N);
    proc(r_mask, r_ROI, N);
    getCenter(l_centerImg_sec, l_ROI, N);
    getCenter(r_centerImg_sec, r_ROI, N);
    Mat l_centerImg_sec_in_3d = (Mat_<double>(3, 1) << l_centerImg_sec.x, l_centerImg_sec.y, 1);
    Mat r_centerImg_sec_in_3d = (Mat_<double>(3, 1) << r_centerImg_sec.x, r_centerImg_sec.y, 1);
    l_centerWld_sec = l_invertMat * l_centerImg_sec_in_3d;
    r_centerWld_sec = r_invertMat * r_centerImg_sec_in_3d;
    
    double z_fir, z_sec;
    double d_fir, d_sec;
    double d_fir_x = pow(l_centerImg_fir.x - r_centerImg_fir.x, 2);
    double d_fir_y = pow(l_centerImg_fir.y - r_centerImg_fir.y, 2);
    d_fir = sqrt(d_fir_x + d_fir_y);
    z_fir = f * T / d_fir;
    double d_sec_x = pow(l_centerImg_sec.x - r_centerImg_sec.x, 2);
    double d_sec_y = pow(l_centerImg_sec.y - r_centerImg_sec.y, 2);
    d_sec = sqrt(d_sec_x + d_sec_y);
    z_sec = f * T / d_sec;
    deltaZ = z_sec - z_fir;
    cout << "深度距离：" << deltaZ << endl << endl;
    double x1, y1, x2, y2;
    x1 = l_centerWld_fir.at<double>(0, 0);
    y1 = l_centerWld_fir.at<double>(1, 0);
    x2 = l_centerWld_sec.at<double>(0, 0);
    y2 = l_centerWld_sec.at<double>(1, 0);
    if (deltaZ >= 0)
        wldDist = pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(deltaZ, 2);
    if (deltaZ < 0)
        wldDist = pow(x1 - x2, 2) + pow(y1 - y2, 2) - pow(deltaZ, 2);
    wldDist = sqrt(fabs(wldDist));
    cout << "静态位移：" << deltaZ << endl << endl;
}
