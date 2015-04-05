

//#include "stdafx.h"
#include <cstddef>
#include "OpenNI.h"
#include "iostream"
#include <opencv2/core/core.hpp> // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
//#include "NiTE.h"


using namespace cv;

int main() {

    RNG rng(12345);
    int c = 100;
    openni::Status rc = openni::STATUS_OK;
    openni::Device device;
    openni::VideoStream depth, color;
    openni::VideoFrameRef pFrame, dep;

    const char* deviceURI = openni::ANY_DEVICE;

    rc = openni::OpenNI::initialize();

    printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

    rc = device.open(deviceURI);
    if (rc != openni::STATUS_OK) {

        int a = 0;
        printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
        std::cin >> a;
        return 1;

    }

    rc = depth.create(device, openni::SENSOR_DEPTH);

    if (rc == openni::STATUS_OK) {

        rc = depth.start();
        if (rc != openni::STATUS_OK) {
            printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
            depth.destroy();
        }
    } else {
        printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
    }

    rc = color.create(device, openni::SENSOR_COLOR);
    if (rc == openni::STATUS_OK) {
        rc = color.start();
        if (rc != openni::STATUS_OK) {
            printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
            color.destroy();
        }
    } else {
        printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
    }

    if (!depth.isValid() || !color.isValid()) {
        printf("SimpleViewer: No valid streams. Exiting\n");
        openni::OpenNI::shutdown();
        return 2;
    }
    cv::namedWindow("depth", CV_WINDOW_KEEPRATIO);
    cv::namedWindow("OpenCV", CV_WINDOW_KEEPRATIO);
    cv::namedWindow("contour", CV_WINDOW_KEEPRATIO);

    cv::Mat frame = cv::Mat(cv::Size(320, 240), CV_8UC3);


    cv::Mat bw;
    while (1) {
        color.readFrame(&pFrame);
        depth.readFrame(&dep);
        openni::RGB888Pixel *pColor = (openni::RGB888Pixel *) pFrame.getData();
        openni::DepthPixel* pDepth = (openni::DepthPixel *) dep.getData();

        bw = cv::Mat(cv::Size(320, 240), CV_16UC1, (void*) pDepth);

        for (int i = 0; i < frame.rows; i++) {
            for (int j = 0; j < frame.cols; j++) {
                openni::RGB888Pixel pix = pColor[frame.cols * i + j];
                int d = pDepth[frame.cols * i + j];

                if (d < 1750 && d > 20) {
                    frame.at<cv::Vec3b>(i, j) = cv::Vec3b(pix.r, pix.g, pix.b);
                } else {
                    frame.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
                }


            }
        }

        /*blur depth data*/
        cv::Mat temp = cv::Mat::zeros(frame.size(), CV_8UC1);
        cv::cvtColor(frame, temp, CV_BGR2GRAY);
        //        cv::Mat temp2 = cv::Mat::zeros(frame.size(), CV_8UC1);
        cv::GaussianBlur(temp, temp, Size(11, 11), 0, 0);
        //        cv::convertScaleAbs(temp, temp2, 1, 0);
        //        temp.convertTo(temp2, CV_8UC1, 255.0/32768.0);

        Mat canny_output = cv::Mat::zeros(frame.size(), CV_8UC1);
        Mat contour = cv::Mat::zeros(frame.size(), CV_8UC3);
        vector<vector<Point> > contours;
        //        vector<Vec4i> hierarchy;

        /// Detect edges using canny
        unsigned int thresh = 20;
        Canny(temp, canny_output, thresh, thresh * 2, 3);
        /// Find contours
        findContours(canny_output, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);


        /*draw borders / contours*/
//        Scalar color(255, 255, 255);
        for (unsigned int i = 0; i < contours.size(); i++) {
            if (contours.at(i).size() > 100) {
                Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
                drawContours(contour, contours, i, color, 1, 8);
            }

        }

        cv::imshow("depth", bw);
        cv::imshow("OpenCV", frame);
        cv::imshow("contour", contour);
        
//        sleep(1);

        c = cv::waitKey(10);
        if (c == 27)
            break;
    }
    return 0;

}