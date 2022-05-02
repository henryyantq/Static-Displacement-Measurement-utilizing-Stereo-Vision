
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <cmath>
#define N 1
#define FOC_IN_PIX 928.334  // iPhone 13 Pro广角摄像头像素级焦距

using namespace std;
using namespace cv;

Rect* l_ROI = new Rect[N];
Rect* r_ROI = new Rect[N];

void proc(Mat& binary, Rect* rect, int size) {
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binary, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
    if (contours.size() > 0) {
        double* area = new double[contours.size()];
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

void getColorMask(Mat& src, Mat& dst, Mat& mask) {
    Mat mask1, mask2;
    cvtColor(src, dst, COLOR_BGR2HSV);
    inRange(dst, Scalar(0, 127, 0), Scalar(10, 255, 255), mask1);
    inRange(dst, Scalar(156, 43, 46), Scalar(180, 255, 255), mask2);
    mask = mask1 + mask2;
}

void getCenter(Point2f& center, Rect* rect, int size) {
    center.x = 0;
    center.y = 0;
    for (int i = 0; i < size; i++) {
        center.x += rect[i].x + rect[i].width / 2;
        center.y += rect[i].y + rect[i].height / 2;
    }
    center.x /= size;
    center.y /= size;
}

int main() {
    Point2f l_centerImg_fir, l_centerImg_sec;
    Point2f r_centerImg_fir, r_centerImg_sec;
    double wldDist = 0;     // 公制位移
    Mat l_src_fir, l_dst_fir;
    Mat l_src_sec, l_dst_sec;
    Mat r_src_fir, r_dst_fir;
    Mat r_src_sec, r_dst_sec;
    Mat l_mask, r_mask;
    double T = 50, f = FOC_IN_PIX;

    l_src_fir = imread("");
    l_src_sec = imread("");
    r_src_fir = imread("");
    r_src_sec = imread("");

    getColorMask(l_src_fir, l_dst_fir, l_mask);
    getColorMask(r_src_fir, r_dst_fir, r_mask);
    proc(l_mask, l_ROI, N);
    proc(r_mask, r_ROI, N);
    getCenter(l_centerImg_fir, l_ROI, N);
    getCenter(r_centerImg_fir, r_ROI, N);

    getColorMask(l_src_sec, l_dst_sec, l_mask);
    getColorMask(r_src_sec, r_dst_sec, r_mask);
    proc(l_mask, l_ROI, N);
    proc(r_mask, r_ROI, N);
    getCenter(l_centerImg_sec, l_ROI, N);
    getCenter(r_centerImg_sec, r_ROI, N);

    double deltaX, deltaY, deltaZ;
    double lx1, rx1, ly1, ry1, lx2, rx2, ly2, ry2, x1, x2, y1, y2, z1, z2;
    lx1 = l_centerImg_fir.x;
    rx1 = r_centerImg_fir.x;
    ly1 = l_centerImg_fir.y;
    ry1 = r_centerImg_fir.y;
    z1 = f * T / (lx1 - rx1);
    x1 = lx1 * z1 / f;
    y1 = ly1 * z1 / f;
    Mat fir_pt = (Mat_<double>(3, 1) << x1, y1, z1);
    cout << "First Point (in mm) :\n" << fir_pt << endl << endl;
    lx2 = l_centerImg_sec.x;
    rx2 = r_centerImg_sec.x;
    ly2 = l_centerImg_sec.y;
    ry2 = r_centerImg_sec.y;
    z2 = f * T / (lx2 - rx2);
    x2 = lx2 * z2 / f;
    y2 = ly2 * z2 / f;
    Mat sec_pt = (Mat_<double>(3, 1) << x2, y2, z2);
    cout << "Second Point (in mm) :\n" << sec_pt << endl << endl;
    deltaZ = z2 - z1;
    deltaX = x2 - x1;
    deltaY = y2 - y1;
    wldDist = sqrt(pow(deltaZ, 2) + pow(deltaX, 2) + pow(deltaY, 2));
    cout << "Distance (in mm) : " << wldDist << endl;
}
