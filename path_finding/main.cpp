

#include "main.h"

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
    cv::namedWindow("birds-eye", CV_WINDOW_KEEPRATIO);


    cv::Mat frame = cv::Mat(cv::Size(320, 240), CV_8UC3);
    int frame_depth = 1750;



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

                if (d < frame_depth && d > 5) {
                    frame.at<cv::Vec3b>(i, j) = cv::Vec3b(pix.r, pix.g, pix.b);
                } else {
                    frame.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
                }


            }
        }

        /*blur depth data*/
        cv::Mat temp = cv::Mat::zeros(frame.size(), CV_8UC3);
        cv::Mat weightedFrame = cv::Mat(frame.size(), CV_32FC3);
        cv::Mat run_ave = cv::Mat::zeros(frame.size(), CV_8UC3);
        
        cv::GaussianBlur(frame, temp, Size(13, 13), 0, 0);
        
        
        cv::accumulateWeighted(temp, weightedFrame, 0.01);
        cv::convertScaleAbs(weightedFrame, run_ave, 1.0, 0.0);
        
        /*allocate memory for canny*/
        Mat canny_output = cv::Mat::zeros(frame.size(), CV_8UC1);
        Mat contour = cv::Mat::zeros(frame.size(), CV_8UC3);
        vector<vector<Point> > contours;

        /// Detect edges using canny
        unsigned int thresh = 20;
        Canny(run_ave, canny_output, thresh, thresh * 2, 3);
        /// Find contours
        findContours(canny_output, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

        cv::Mat birds_eye = cv::Mat::zeros(cv::Size(frame_depth, 320), CV_8UC3);

        /*draw borders / contours*/
        //        Scalar color(255, 255, 255);
        for (unsigned int i = 0; i < contours.size(); i++) {
            vector<Point> next_contour = contours.at(i);
            if (next_contour.size() > 200) {
                Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
                drawContours(contour, contours, i, color, 1, 8);

                /*draw onto birds-eye view*/
                for (unsigned int j = 0; j < next_contour.size(); j++) {
                    Point next_point = next_contour.at(j);
                    int x = next_point.x;
                    int y = next_point.y;
                    int z = pDepth[frame.cols * y + x];
                    birds_eye.at<cv::Vec3b>(x, z) = cv::Vec3b(255, 0, 0);
                }
            }

        }
        
        /*compute shortest path through obstacles*/
        Point * traject = search(birds_eye);
        
        /*draw the trajectory*/
//        Mat imgLines = Mat::zeros(img.size(), CV_8UC3); // allocate new memory
        int index = 1;
        Point last_point = traject[0];
        while(true){
            Point this_point = traject[index];
            
            if(this_point.x == 0 && this_point.y == 0){
                break;
            }
            
            line(birds_eye, this_point, last_point, Scalar(0, 0, 255), 2);    
            last_point = this_point;
            index++;
        }

        cv::imshow("depth", run_ave);
        cv::imshow("OpenCV", frame);
        cv::imshow("contour", contour);
        cv::imshow("birds-eye", birds_eye);

//                sleep(4);

        c = cv::waitKey(10);
        if (c == 27)
            break;
    }
    return 0;

}
