#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>

// dimensions of checkerboard
int CHECKERBOARD[2]{ 6, 9 };

int main()
{
    std::vector<cv::String> images;

//    path of checkerboard images
    std::string image_path = "/home/minn/CLionProjects/homography-calculation/calibrated-images";

//    get paths
    cv::glob(image_path, images);

    std::cout << "the number of images : " << images.size() << std::endl;
    for (auto loop : images)
    {
        std::cout << loop << std::endl;
    }
    if (images.size() == 0)
        std::cout << "image doesn't exist! \n" << std::endl;

//    2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f> > imgp;
//    chessboard corners
    std::vector<cv::Point2f> corner_pts;

    cv::Mat frame, gray;
    std::vector<cv::Mat> origin_frame, corners_frame;

//    vector to store the pixel coordinates of detected chessboard corners
    bool success;
//    looping over all the images in the directory
    for (int i = 0; i < images.size(); i++)
    {
        frame = cv::imread(images[i]);
        origin_frame.push_back(frame.clone());
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

//        finding chessboard corners
        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

        if (success)
        {
            cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

//            refining pixel coordinates for given 2d points.
            cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

//            displaying the detected corner points on the chessboard
            cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
            corners_frame.push_back(frame.clone());
            imgp.push_back(corner_pts);
        }
        cv::imshow("chessboard image", corners_frame[i]);
        cv::waitKey(0);
    }
    cv::destroyAllWindows();

    std::vector<cv::Point2f> calculated_points;
    cv::Mat H, transformed_image;

//    matrix of homography between image1 and image7
//    used calibrated images in camera-calibration repository
    for (int i = 0; i < imgp.size(); i ++) {
//        get homography matrix using chessboard corners
        H = cv::findHomography(imgp[i], imgp[(i + 1) % 2], cv::RANSAC);
//        perspective transform by homography matrix
        cv::perspectiveTransform(imgp[i], calculated_points, H); // chessboard corners
        cv::warpPerspective(origin_frame[i], transformed_image, H, origin_frame[i].size()); // image
        std::cout << "homography matrix:\n" << H << std::endl;

//        draw corners with calculated points on transformed image
        cv::drawChessboardCorners(transformed_image, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), calculated_points, success);

        cv::hconcat(corners_frame[i], transformed_image, transformed_image);
        cv::hconcat(transformed_image, corners_frame[(i + 1) % 2], transformed_image);
        cv::resize(transformed_image, transformed_image, cv::Size(transformed_image.cols * 0.5, transformed_image.rows * 0.5));

        cv::imshow("compare images", transformed_image);
        cv::imwrite("../transformed-images/compare_"+std::to_string(i)+=".jpg", transformed_image);
        cv::waitKey(0);
    }
    cv::destroyAllWindows();
    return 0;
}