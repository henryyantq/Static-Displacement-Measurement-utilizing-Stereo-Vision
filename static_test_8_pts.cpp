
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <cmath>
#define N 8

using namespace std;
using namespace cv;

Rect* ROI = new Rect[N];

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
    blur(src, dst, Size(3, 3));
    cvtColor(dst, dst, COLOR_BGR2HSV);
    inRange(dst, Scalar(0, 127, 0), Scalar(10, 255, 255), mask1);
    inRange(dst, Scalar(156, 43, 46), Scalar(180, 255, 255), mask2);
    mask = mask1 + mask2;
}

void getCenter(Point &center, Rect *rect, int size) {
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
    Point centerImg_fir, centerImg_sec;     // 位移始端、位移终端靶标中心的像素坐标
    double wldDist = 0;     // 公制位移
    Mat centerWld_fir = (Mat_<double>(3, 1) << 0, 0, 1);    // 位移始端靶标中心的世界坐标
    Mat centerWld_sec = (Mat_<double>(3, 1) << 0, 0, 1);    // 位移终端靶标中心的世界坐标
    vector<Point2f> imgPlane(N);    // 图像平面标记点像素坐标
    vector<Point2f> realPlane(N);   // 实际靶标平面标记点世界坐标
    Mat src_fir, dst_fir;
    Mat src_sec, dst_sec;
    Mat mask;     // 颜色滤镜
    Mat homoMat, invertMat;     // 单应矩阵及其逆矩阵
    
    src_fir = imread("/Users/chenxuan/Downloads/OpenCV_Proj/start.jpeg");
    src_sec = imread("/Users/chenxuan/Downloads/OpenCV_Proj/end.jpeg");
    
    // 对位移始端拍摄的靶标图像进行处理并获得单应矩阵及其逆矩阵
    getColorMask(src_fir, dst_fir, mask);   // 获得颜色滤镜
    proc(mask, ROI, N);     // 识别得到8个感兴趣区域
    getCenter(centerImg_fir, ROI, N);   // 获得8个感兴趣区域的正中心
    circle(src_fir, centerImg_fir, 1, Scalar(255, 0, 0), -1);   // 将正中心点出
    getImgPlaneIndex(src_fir, ROI, imgPlane, N, "Initial Location");    // 获得像素坐标
    getRealPlaneIndex(realPlane, N);    // 获得世界坐标
    homoMat = findHomography(realPlane, imgPlane, RANSAC, 5.0);     // 使用RANSAC方法求解最优单应映射
    invertMat = homoMat.inv();      // 求单应映射矩阵的逆
    cout << homoMat << endl << endl;    // 输出计算得到的单应矩阵
    Mat centerImg_fir_in_3V = (Mat_<double>(3, 1) << centerImg_fir.x, centerImg_fir.y, 1);      // 将靶标中心的像素坐标张成三维向量
    centerWld_fir = invertMat * centerImg_fir_in_3V;   // 映射得到靶标中心的实际坐标（三维）
    
    // 对位移终端拍摄的靶标图像进行处理并获得单应矩阵及其逆矩阵
    getColorMask(src_sec, dst_sec, mask);
    proc(mask, ROI, N);
    getCenter(centerImg_sec, ROI, N);
    circle(src_sec, centerImg_sec, 1, Scalar(255, 0, 0), -1);
    Mat centerImg_sec_in_3V = (Mat_<double>(3, 1) << centerImg_sec.x, centerImg_sec.y, 1);
    centerWld_sec = invertMat * centerImg_sec_in_3V;
    
    // 计算静态位移并输出结果
    double x1, y1, x2, y2;
    x1 = centerWld_fir.at<double>(0, 0);
    y1 = centerWld_fir.at<double>(1, 0);
    x2 = centerWld_sec.at<double>(0, 0);
    y2 = centerWld_sec.at<double>(1, 0);
    wldDist = pow(x1 - x2, 2) + pow(y1 - y2, 2);
    wldDist = sqrt(wldDist);
    cout << "静态位移（直线距离）：" << wldDist << "cm" << endl;
}
