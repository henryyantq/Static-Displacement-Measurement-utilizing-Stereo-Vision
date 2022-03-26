#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace std;
using namespace cv;

Rect *ROI = new Rect [4];
void proc(Mat &binary, Rect &rect1, Rect &rect2, Rect &rect3, Rect &rect4);

int main() {
    Point center_pre(0, 0), center_cur;
    double realDist = 0;
    Mat centerVectTarget_pre = (Mat_<double>(3, 1) << 0, 0, 1);
    Point *circle_pre = new Point [4];
    Point *circle_cur = new Point[4];
    for (int i = 0; i <= 3; i++)
        circle_pre[i].x = circle_pre[i].y = 0;
    Mat srcFrame, dstFrame, mask, mask1, mask2;
    Mat kernel_open = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
    Mat kernel_dilate = getStructuringElement(MORPH_RECT, Size(5, 5), Point(-1, -1));
    VideoCapture cap(0);
    cap.set(CAP_PROP_FRAME_WIDTH, 640);     // You can adjust this value to adapt your frame width.
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);    // You can adjust this value to adapt your frame height.
    // Note that, usually, the frame height and width you set will decide the frame rates of the camera when recording.
    while (true) {
        if (!cap.read(srcFrame)) {
            cout << "Capture failed!" << endl;
            break;
        }
        blur(srcFrame, dstFrame, Size(3, 3));
        cvtColor(dstFrame, dstFrame, COLOR_BGR2HSV);
        // Here below, if you redesign the marker, especially the color of the circles, you'll need to adjust the HSV range in the Scalar().
        inRange(dstFrame, Scalar(0, 127, 0), Scalar(10, 255, 255), mask1);
        inRange(dstFrame, Scalar(156, 43, 46), Scalar(180, 255, 255), mask2);
        // Here above, if you redesign the marker, especially the color of the circles, you'll need to adjust the HSV range in the Scalar().
        mask = mask1 + mask2;   //Here, if you have only one HSV filter range utilized above, this line of codes will be needless.
        imshow("Mask", mask);
        proc(mask, ROI[0], ROI[1], ROI[2], ROI[3]);
        Rect *ROI_whole = new Rect[4];
        Rect *ROI_temp = new Rect[4];
        int *ROI_x = new int[4];
        int *ROI_y = new int[4];
        int *ROI_x_temp = new int[4];
        int *ROI_y_temp = new int[4];
        for (int i = 0; i <= 3; i++) {
            ROI_x[i] = ROI[i].x;
            ROI_y[i] = ROI[i].y;
            ROI_x_temp[i] = ROI[i].x;
            ROI_whole[i] = ROI[i];
        }
        sort(ROI_x, ROI_x + 4);
        for (int i = 0; i <= 3; i++)
            for (int j = 0; j <= 3; j++)
                if (ROI_x_temp[j] == ROI_x[i]) {
                    ROI_y_temp[i] = ROI_y[j];
                    ROI_temp[i] = ROI_whole[j];
                }
        for (int i = 0; i <= 3; i++) {
            ROI_y[i] = ROI_y_temp[i];
            ROI_whole[i] = ROI_temp[i];
        }
        if (ROI_y[0] <= ROI_y[1]) {
            ROI[0] = ROI_whole[0];
            ROI[2] = ROI_whole[1];
        }
        else {
            ROI[0] = ROI_whole[1];
            ROI[2] = ROI_whole[0];
        }
        if (ROI_y[2] <= ROI_y[3]) {
            ROI[1] = ROI_whole[2];
            ROI[3] = ROI_whole[3];
        }
        else {
            ROI[1] = ROI_whole[3];
            ROI[3] = ROI_whole[2];
        }
        center_cur.x = (ROI[0].x + ROI[0].width / 2 + ROI[1].x + ROI[1].width / 2 + ROI[2].x + ROI[2].width / 2 + ROI[3].x + ROI[3].width / 2) / 4;
        center_cur.y = (ROI[0].y + ROI[0].height / 2 + ROI[1].y + ROI[1].height / 2 + ROI[2].y + ROI[2].height / 2 + ROI[3].y + ROI[3].height / 2) / 4;
        if (center_pre.x != 0 && center_pre.y != 0) {
            line(srcFrame, center_pre, center_cur, Scalar(0, 255, 0), 3, 8);
            circle(srcFrame, center_cur, 1, Scalar(255, 0, 0), -1);
        }
        vector<Point2f> imgplane(4);
        for (int i = 0; i <= 3; i++) {
            Point pt;
            imgplane[i].x = pt.x = ROI[i].x + ROI[i].width / 2;
            imgplane[i].y = pt.y = ROI[i].y + ROI[i].height / 2;
            circle_cur[i].x = pt.x;
            circle_cur[i].y = pt.y;
            if (circle_pre[i].x != 0 && circle_pre[i].y != 0)
                line(srcFrame, circle_pre[i], circle_cur[i], Scalar(0, 255, 0), 3, 8);
            circle_pre[i].x = circle_cur[i].x;
            circle_pre[i].y = circle_cur[i].y;
            rectangle(srcFrame, ROI[i], Scalar(0, 255, 0), 3, 8, 0);
            circle(srcFrame, pt, 1, Scalar(0, 255, 0), -1);
        }
        vector<Point2f> realplane{Point2f(6.6, 5.9), Point2f(18.3, 5.9), Point2f(6.6, 13.2), Point2f(18.3, 13.2)};
        Mat homoMat;    // Homo Matrix
        homoMat = findHomography(imgplane, realplane);
        cout << homoMat << endl << endl;    // Here's where the 3x3 homography matrix is printed in real time.
        Mat centerVect = (Mat_<double>(3, 1) << center_cur.x, center_cur.y, 1);
        Mat centerVectTarget_cur =  homoMat * centerVect;
        if (centerVectTarget_pre.at<double>(0, 0) != 0 && centerVectTarget_pre.at<double>(1, 0) != 0) {
            realDist = pow(centerVectTarget_cur.at<double>(0, 0) - centerVectTarget_pre.at<double>(0, 0), 2);
            realDist += pow(centerVectTarget_cur.at<double>(1, 0) - centerVectTarget_pre.at<double>(1, 0), 2);
            realDist = sqrt(realDist);
            cout << "Real-world Distance: " << realDist << " cm" << endl;   // Here's where the real-world movement distance is printed in real time. 
        }
        centerVectTarget_pre = centerVectTarget_cur;
        center_pre.x = center_cur.x;
        center_pre.y = center_cur.y;
        imshow("Output", srcFrame);
        int keycode = waitKey(30) & 0xff;
        if (keycode == 27) break;
    }
    cap.release();
    cv::destroyAllWindows();
}

void proc(Mat &binary, Rect &rect1, Rect &rect2, Rect &rect3, Rect &rect4) {
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binary, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
    if (contours.size() > 0) {
        double *area = new double[contours.size()];
        for (size_t t = 0; t < contours.size(); t++)
            area[t] = contourArea(contours[static_cast<int>(t)]);
        sort(area, area + contours.size()); 
        for (size_t t = 0; t < contours.size(); t++) {
            if (contourArea(contours[static_cast<int>(t)]) == area[contours.size() - 1])
                rect1 = boundingRect(contours[static_cast<int>(t)]);
            else if (contourArea(contours[static_cast<int>(t)]) == area[contours.size() - 2])
                rect2 = boundingRect(contours[static_cast<int>(t)]);
            else if (contourArea(contours[static_cast<int>(t)]) == area[contours.size() - 3])
                rect3 = boundingRect(contours[static_cast<int>(t)]);
            else if (contourArea(contours[static_cast<int>(t)]) == area[contours.size() - 4])
                rect4 = boundingRect(contours[static_cast<int>(t)]);
        }
    }
    else {
        rect1.x = rect1.y = rect1.width = rect1.height = 0;
        rect2.x = rect2.y = rect2.width = rect2.height = 0;
        rect3.x = rect3.y = rect3.width = rect3.height = 0;
        rect4.x = rect4.y = rect4.width = rect4.height = 0;
    }
}
