#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() 
{
    Mat img = imread("armor.jpg");
    if (img.empty()) 
    {
        cout << "图片读取失败！" << endl;
        return -1;
    }

    Mat hsv;
    cvtColor(img, hsv, COLOR_BGR2HSV);

    vector<Mat> channels;
    split(hsv, channels);
    Mat H = channels[0];
    Mat S = channels[1];
    Mat V = channels[2];

    imshow("H Channel", H);
    imshow("S Channel", S);
    imshow("V Channel", V);

    Mat mask1, mask2, red_mask;
    inRange(hsv, Scalar(0, 43, 46), Scalar(10, 255, 255), mask1);
    inRange(hsv, Scalar(156, 43, 46), Scalar(180, 255, 255), mask2);
    bitwise_or(mask1, mask2, red_mask);

    imwrite("red_mask.png", red_mask);
    imshow("Red Mask", red_mask);

    waitKey(0);
    return 0;
}
