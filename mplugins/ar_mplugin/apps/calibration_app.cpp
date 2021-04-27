//
//
//
//
//

#include <opencv2/opencv.hpp>
#include <fstream>
#include <thread>
#include <mutex>
#include <chrono>

using namespace cv;
using namespace std;

bool gRunning = true;
bool gSaveNext = false;
std::mutex gMutex;

void mouseCallback(int event, int x, int y, int flags, void* userdata);

bool imagesFromCamera(int _argc, char** _argv, std::vector<std::vector<cv::Point2f>>& _leftPoints, std::vector<std::vector<cv::Point2f>>& _rightPoints);

Size boardSize;
int width, height;

int main(int _argc, char** _argv) {
    if (_argc != 6) {
        std::cout << "Bad input arguments: " << std::endl;
        std::cout << "./calibration [cameraIdx] [nPointsBoardVert] [nPointsBoardHoriz] [squareSize	(mm)] [outCalibFile]" << std::endl;
        return -1;
    }
    std::vector<std::vector<cv::Point2f>> pointsLeft2D, pointsRight2D;
    if (!imagesFromCamera(_argc, _argv, pointsLeft2D, pointsRight2D)) {
        std::cout << "Something failed" << std::endl;
        return -1;
    }

    std::cout << "Image width: " << width << ", and height " << height << std::endl;
    //// Calibration!
    // Calibrate single cameras.
    float squareSize = atof(_argv[4]);
    std::cout << "Computing parameter" << std::endl;
    Mat matrixLeft = Mat::eye(3, 3, CV_64F), matrixRight = Mat::eye(3, 3, CV_64F);
    Mat distCoefLeft = Mat::zeros(8, 1, CV_64F), distCoefRight = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f>> pointsLeft(1);
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            pointsLeft[0].push_back(Point3f(float(j * squareSize), float(i * squareSize), 0));
        }
    }
    pointsLeft.resize(pointsLeft2D.size(), pointsLeft[0]);

    std::vector<cv::Mat> rotVectors, transVectors;

    double rmsLeft = calibrateCamera(pointsLeft, pointsLeft2D, Size(width / 2, height), matrixLeft, distCoefLeft, rotVectors, transVectors, CALIB_FIX_K4 | CALIB_FIX_K5);
    std::cout << "Rms left calibration: " << rmsLeft << std::endl;

    std::cout << "Stereo Camera calibrated! Saving files." << std::endl;
    FileStorage fs(_argv[5], FileStorage::WRITE);

    fs << "Matrix" << matrixLeft;
    fs << "DistCoeffs" << distCoefLeft;
    fs << "rs" << rotVectors;
    fs << "ts" << transVectors;

    std::cout << "Saved files." << std::endl;

    return 1;
}
//---------------------------------------------------------------------------------------------------------------------
void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    if (event == EVENT_LBUTTONDOWN) {
        gSaveNext = true;
    }
    else if (event == EVENT_RBUTTONDOWN) {
        std::cout << "Received stop signal" << std::endl;
        gRunning = false;
    }
}


bool imagesFromCamera(int _argc, char** _argv, std::vector<std::vector<cv::Point2f>>& _leftPoints, std::vector<std::vector<cv::Point2f>>& _rightPoints) {
    VideoCapture camera(atoi(_argv[1]));
    if (!camera.isOpened()) {
        std::cout << "Couldn't open camera\n";
        return false;
    }

    boardSize.height = atoi(_argv[2]);
    boardSize.width = atoi(_argv[3]);

    // Start!
    std::string displayerName = "Image displayer";
    namedWindow(displayerName, cv::WINDOW_FREERATIO);
    setMouseCallback(displayerName, mouseCallback);

    cv::Mat left;
    std::thread captureThread([&]() {
        Mat lLeft;
        while (gRunning) {
            camera >> lLeft;
            gMutex.lock();
            lLeft.copyTo(left);
            gMutex.unlock();
            //std::this_thread::sleep_for(std::chrono::milliseconds(10);
        }
    });

    while (gRunning) {
        Mat lLeft;
        gMutex.lock();
        left.copyTo(lLeft);
        gMutex.unlock();
        width = lLeft.cols;
        height = lLeft.rows;
        if (lLeft.rows == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        Mat leftGray;
        cvtColor(lLeft, leftGray, COLOR_BGR2GRAY);
        vector<Point2f> pointBufLeft;
        bool foundLeft = findChessboardCorners(leftGray, boardSize, pointBufLeft, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK);

        if (gSaveNext) {
            if (foundLeft) {
                cornerSubPix(leftGray, pointBufLeft, Size(7, 7), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
                _leftPoints.push_back(pointBufLeft);
                gSaveNext = false;
                std::cout << "Added a new pair of images for the calibration." << std::endl;
            }
        }

        drawChessboardCorners(lLeft, boardSize, pointBufLeft, foundLeft);
        imshow(displayerName, lLeft);
        waitKey(3);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "Chosen " << _leftPoints.size() << " images. Performing calibration with chosen images! Wait...." << std::endl;

    captureThread.join();
    return true;
}
