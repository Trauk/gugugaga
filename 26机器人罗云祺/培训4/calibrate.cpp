#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

namespace
{

const cv::Size kBoardSize(9, 6);
const float kSquareSize = 1.0f;
const int kMinViews = 6;
const int kRecommendedViews = 10; 

string lowerExtension(const string& path)
{
    const string::size_type dot = path.find_last_of('.');
    if (dot == string::npos)
    {
        return "";
    }
    string ext = path.substr(dot);
    transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(tolower(c)); });
    return ext;
}

string baseName(const string& path)
{
    const string::size_type pos = path.find_last_of("/\\");
    return (pos == string::npos) ? path : path.substr(pos + 1);
}

vector<string> findImages(const string& dir)
{
    vector<cv::String> found;
    cv::glob(dir + "/*", found, false);

    vector<string> files;
    for (const cv::String& item : found)
    {
        const string path = item.c_str();
        const string ext = lowerExtension(path);
        if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp")
        {
            files.push_back(path);
        }
    }
    sort(files.begin(), files.end());
    return files;
}

}

int main(int argc, char** argv)
{
    const string imageDir = (argc > 1) ? argv[1] : "../images";

    vector<string> fileNames = findImages(imageDir);
    if (fileNames.empty()) {
        cerr << "未找到图片: " << imageDir << "/*.jpg|*.png" << endl;
        cerr << "用法: " << argv[0] << " <图片目录> [测试图片]" << endl;
        return -1;
    }

    cout << "OpenCV " << CV_VERSION << endl;
    cout << "图片目录: " << imageDir << "，共 " << fileNames.size() << " 张" << endl;
    cout << "棋盘格内角点: " << kBoardSize.width << " x " << kBoardSize.height << endl;

    vector<vector<cv::Point3f>> objectPoints;
    vector<vector<cv::Point2f>> imagePoints;
    vector<string> usedFiles;
    vector<cv::Point2f> corners;

    vector<cv::Point3f> obj;
    obj.reserve(static_cast<size_t>(kBoardSize.area()));
    for (int y = 0; y < kBoardSize.height; ++y) {
        for (int x = 0; x < kBoardSize.width; ++x)
        {
            obj.push_back(cv::Point3f(x * kSquareSize, y * kSquareSize, 0.0f));
        }
    }

    cv::Size imageSize;
    int unreadable = 0;
    int sizeMismatch = 0;
    int notFound = 0;

    for (const string& file : fileNames)
    {
        cv::Mat img = cv::imread(file, cv::IMREAD_COLOR);
        if (img.empty())
        {
            cerr << "[跳过] 无法读取: " << file << endl;
            ++unreadable;
            continue;
        }

        if (imageSize.width == 0)
        {
            imageSize = img.size();
        }
        else if (img.size() != imageSize)
        {
            cerr << "[跳过] 尺寸不一致: " << baseName(file) << " ("
                      << img.cols << "x" << img.rows << " != "
                      << imageSize.width << "x" << imageSize.height << ")" << endl;
            ++sizeMismatch;
            continue;
        }

        cv::Mat gray;
        if (img.channels() == 3)
        {
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        }
        else
        {
            gray = img;
        }

        corners.clear();
        const int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
        bool found = cv::findChessboardCorners(gray, kBoardSize, corners, flags);
        if (!found)
        {
            found = cv::findChessboardCorners(
                gray, kBoardSize, corners, flags | cv::CALIB_CB_EXHAUSTIVE);
        }

        if (!found || corners.size() != static_cast<size_t>(kBoardSize.area()))
        {
            cerr << "[失败] 未找到 " << kBoardSize.width << "x" << kBoardSize.height
                      << " 内角点: " << baseName(file) << endl;
            ++notFound;
            continue;
        }

        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                                          30, 0.1));

        imagePoints.push_back(corners);
        objectPoints.push_back(obj);
        usedFiles.push_back(file);

        cv::Mat debug = img.clone();
        cv::drawChessboardCorners(debug, kBoardSize, corners, true);
        cv::imwrite("corners_" + baseName(file), debug);

        cout << "[成功] " << baseName(file) << "，角点数 " << corners.size() << endl;
    }

    cout << "\n成功 " << imagePoints.size() << " 张"
              << "，无法读取 " << unreadable << " 张"
              << "，分辨率不一致 " << sizeMismatch << " 张"
              << "，未找到角点 " << notFound << " 张。" << endl;

    if (sizeMismatch > 0)
    {
        cout << "提示：分辨率不一致的图片没有参与标定。"
                  << "请把同一分辨率的标定图单独放一个目录，例如 ../images/calib/。"
                  << endl;
    }

    if (imagePoints.size() < static_cast<size_t>(kMinViews))
    {
        cerr << "有效视角不足 " << kMinViews << " 张，无法可靠标定。" << endl;
        return -1;
    }
    if (imagePoints.size() < static_cast<size_t>(kRecommendedViews))
    {
        cerr << "警告：有效视角少于 " << kRecommendedViews
                  << " 张，内参可能不稳定；建议 15~20 张不同角度/位置的图片。" << endl;
    }

    cv::Mat cameraMatrix, distCoeffs;
    vector<cv::Mat> rvecs, tvecs;
    double rms = 0.0;

    try
    {
        rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize,
                                  cameraMatrix, distCoeffs, rvecs, tvecs);
    }
    catch (const cv::Exception& e)
    {
        cerr << "calibrateCamera 失败: " << e.what() << endl;
        return -1;
    }

    cout << fixed << setprecision(6);
    cout << "\n================ 标定结果 ================" << endl;
    cout << "图像尺寸: " << imageSize.width << " x " << imageSize.height << endl;
    cout << "有效视角: " << imagePoints.size() << endl;
    cout << "RMS 重投影误差: " << rms << " px" << endl;
    cout << "相机内参 K:\n" << cameraMatrix << endl;
    cout << "fx = " << cameraMatrix.at<double>(0, 0)
              << ", fy = " << cameraMatrix.at<double>(1, 1)
              << ", cx = " << cameraMatrix.at<double>(0, 2)
              << ", cy = " << cameraMatrix.at<double>(1, 2) << endl;

    cout << "畸变系数 [k1, k2, p1, p2, k3]:" << endl;
    for (int i = 0; i < static_cast<int>(distCoeffs.total()); ++i)
    {
        cout << "  [" << i << "] " << distCoeffs.at<double>(i) << endl;
    }
    if (distCoeffs.total() >= 4)
    {
        cout << "k1=" << distCoeffs.at<double>(0)
                  << ", k2=" << distCoeffs.at<double>(1)
                  << ", p1=" << distCoeffs.at<double>(2)
                  << ", p2=" << distCoeffs.at<double>(3);
        if (distCoeffs.total() >= 5)
        {
            cout << ", k3=" << distCoeffs.at<double>(4);
        }
        cout << endl;
    }

    cout << "\n每张图的 RMSE (px):" << endl;
    double errorSum = 0.0;
    for (size_t i = 0; i < objectPoints.size(); ++i)
    {
        vector<cv::Point2f> projected;
        cv::projectPoints(objectPoints[i], rvecs[i], tvecs[i],
                          cameraMatrix, distCoeffs, projected);
        const double err = cv::norm(imagePoints[i], projected, cv::NORM_L2) /
                           sqrt(static_cast<double>(projected.size()));
        errorSum += err;
        cout << "  " << baseName(usedFiles[i]) << ": " << err << endl;
    }
    cout << "平均每张图 RMSE: " << (errorSum / static_cast<double>(objectPoints.size()))
              << " px" << endl;

    size_t testIndex = 0;
    double bestSpread = -1.0;
    const cv::Point2f center(imageSize.width * 0.5f, imageSize.height * 0.5f);
    for (size_t i = 0; i < imagePoints.size(); ++i)
    {
        double meanDistance = 0.0;
        for (const cv::Point2f& p : imagePoints[i])
        {
            meanDistance += hypot(p.x - center.x, p.y - center.y);
        }
        meanDistance /= static_cast<double>(imagePoints[i].size());
        if (meanDistance > bestSpread)
        {
            bestSpread = meanDistance;
            testIndex = i;
        }
    }

    const string testFile = (argc > 2) ? argv[2] : usedFiles[testIndex];
    cv::Mat testImg = cv::imread(testFile, cv::IMREAD_COLOR);
    if (testImg.empty())
    {
        cerr << "测试图读取失败: " << testFile << endl;
        return -1;
    }
    if (testImg.size() != imageSize)
    {
        cerr << "测试图尺寸与标定图不一致: " << testImg.cols << "x" << testImg.rows
                  << " != " << imageSize.width << "x" << imageSize.height << endl;
        return -1;
    }

    cv::Mat undistorted;
    cv::undistort(testImg, undistorted, cameraMatrix, distCoeffs);

    const int header = 40;
    cv::Mat canvas(testImg.rows + header, testImg.cols * 2, CV_8UC3,
                   cv::Scalar(30, 30, 30));
    testImg.copyTo(canvas(cv::Rect(0, header, testImg.cols, testImg.rows)));
    undistorted.copyTo(canvas(cv::Rect(testImg.cols, header, testImg.cols, testImg.rows)));
    cv::putText(canvas, "Original", cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX,
                0.7, cv::Scalar(0, 255, 0), 2);
    cv::putText(canvas, "Undistorted", cv::Point(testImg.cols + 12, 28),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

    cv::imwrite("undistorted.jpg", undistorted);
    cv::imwrite("undistorted_compare.jpg", canvas);

    cv::FileStorage fs("intrinsics.yml", cv::FileStorage::WRITE);
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "rms_reprojection_error" << rms;
    fs.release();

    cout << "\n测试图: " << testFile << endl;
    cout << "已保存: undistorted.jpg, undistorted_compare.jpg, intrinsics.yml"
              << endl;
    cout << "按任意键关闭窗口..." << endl;

    cv::imshow("Left: Original | Right: Undistorted", canvas);
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}
