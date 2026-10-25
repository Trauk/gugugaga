#include <opencv2/opencv.hpp>
using namespace cv;

int main() 
{
    Mat img = imread("armor2.jpg"); 
    
    if (img.empty())
    {
        return -1;
    }

    imshow("Image", img);
    waitKey(0);
    return 0;
}
