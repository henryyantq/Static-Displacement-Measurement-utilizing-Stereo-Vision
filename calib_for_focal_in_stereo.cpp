
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <cmath>
#define N 1

using namespace std;
using namespace cv;

Rect* l_ROIs = new Rect[N];
Rect* r_ROIs = new Rect[N];

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

int main () {
    Mat left, right;
    Mat l_mark, r_mark;
    Point2f l_center;
    Point2f r_center;
    double Z, d, T, f;
    left = imread("/Users/chenxuan/Desktop/毕业设计/l_calib.jpeg");
    right = imread("/Users/chenxuan/Desktop/毕业设计/r_calib.jpeg");
    cout << "Z = ";
    cin >> Z;
    cout << "T = ";
    cin >> T;
    getColorMask(left, left, l_mark);
    getColorMask(right, right, r_mark);
    proc(l_mark, l_ROIs, 1);
    proc(r_mark, r_ROIs, 1);
    getCenter(l_center, l_ROIs, 1);
    getCenter(r_center, r_ROIs, 1);
    cout << endl << l_center << endl;
    cout << r_center << endl;
    double dx2 = pow(l_center.x - r_center.x, 2);
    double dy2 = pow(l_center.y - r_center.y, 2);
    d = sqrt(dx2 + dy2);
    cout << "d = " << d << " pix" << endl;
    f = Z * d / T;
    cout << endl << "f = " << f << endl;
}
