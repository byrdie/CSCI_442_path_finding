

//#include "stdafx.h"
#include <cstddef>
#include "OpenNI.h"
#include "iostream"
#include <opencv2/core/core.hpp> // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
//#include "NiTE.h"

int main() {

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
    cv::namedWindow("OpenCV", 1);
    cv::Mat frame = cv::Mat(cv::Size(320, 240), CV_8UC3);
    //cv::Mat bw = cv::Mat(cv::Size(320,240), CV_16UC1);
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

                frame.at<cv::Vec3b>(i, j) = cv::Vec3b(pix.r, pix.g, pix.b);
            }
        }

        cv::imshow("depth", bw);
        cv::imshow("OpenCV", frame);

        c = cv::waitKey(10);
        if (c == 27)
            break;
    }
    return 0;

}