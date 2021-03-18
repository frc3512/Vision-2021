// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.
                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)
Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.
This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

#include <cameraserver/CameraServer.h>

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

static bool saveCameraParams(const std::string& filename, cv::Size imageSize,
                             float aspectRatio, int flags,
                             const cv::Mat& cameraMatrix,
                             const cv::Mat& distCoeffs, double totalAvgErr) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) return false;

    std::time_t tt;
    std::time(&tt);
    struct tm* t2 = std::localtime(&tt);
    char buf[1024];
    std::strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;

    if (flags & cv::CALIB_FIX_ASPECT_RATIO) fs << "aspectRatio" << aspectRatio;

    if (flags != 0) {
        std::snprintf(
            buf, 1024, "flags: %s%s%s%s",
            flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
            flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
            flags & cv::CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
            flags & cv::CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;

    return true;
}

/**
 * Create ChAruco board to use
 */
void createCharucoBoard() {
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::CharucoBoard> board =
        cv::aruco::CharucoBoard::create(7, 5, 0.04f, 0.02f, dictionary);
    cv::Mat boardImage;
    board->draw(cv::Size(600, 500), boardImage, 10, 1);
    cv::imwrite("BoardImage.jpg", boardImage);
}

/**
 * Calibrate camera with ChAruco markers
 */
void calibration() {
    int squaresX = 7;
    int squaresY = 5;
    // Width in meters
    float squareLength = 0.029;
    float markerLength = 0.015;
    // 6x6 250 aruco markers
    int dictionaryId = 10;
    std::string outputFile = "CameraCalibration.txt";

    bool showChessboardCorners = true;

    int calibrationFlags = 2;

    // Could use fixed aspect ratio but this is good enough
    float aspectRatio = 1;

    // Principal point is outside image without this
    calibrationFlags |= cv::CALIB_FIX_PRINCIPAL_POINT;

    cv::Ptr<cv::aruco::DetectorParameters> detectorParams =
        cv::aruco::DetectorParameters::create();

    bool refindStrategy = true;

    cs::UsbCamera cvCamera =
        frc::CameraServer::GetInstance()->StartAutomaticCapture();
    cvCamera.SetResolution(640, 480);
    std::cout << "Camera should be started\n";

    cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
    cs::CvSource outputStream =
        frc::CameraServer::GetInstance()->PutVideo("Calibration", 640, 480);
    cs::MjpegServer server{"CalibrationView", 1128};

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(
            cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    // create charuco board object
    cv::Ptr<cv::aruco::CharucoBoard> charucoboard =
        cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength,
                                        markerLength, dictionary);
    cv::Ptr<cv::aruco::Board> board =
        charucoboard.staticCast<cv::aruco::Board>();

    // collect data from each frame
    std::vector<std::vector<std::vector<cv::Point2f> > > allCorners;
    std::vector<std::vector<int> > allIds;
    std::vector<cv::Mat> allImgs;
    cv::Size imgSize;
    cv::Mat image;
    std::clock_t startTime = std::clock();

    while (allIds.size() < 15) {
        cv::Mat imageCopy;

        cvSink.GrabFrame(image, 1);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners, rejected;

        // detect markers
        cv::aruco::detectMarkers(image, dictionary, corners, ids,
                                 detectorParams, rejected);

        // refind strategy to detect more markers
        cv::aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

        // interpolate charuco corners
        cv::Mat currentCharucoCorners, currentCharucoIds;
        if (ids.size() > 0)
            cv::aruco::interpolateCornersCharuco(
                corners, ids, image, charucoboard, currentCharucoCorners,
                currentCharucoIds);

        // draw results
        image.copyTo(imageCopy);
        if (ids.size() > 0) cv::aruco::drawDetectedMarkers(imageCopy, corners);

        if (currentCharucoCorners.total() > 0)
            cv::aruco::drawDetectedCornersCharuco(
                imageCopy, currentCharucoCorners, currentCharucoIds);

        putText(imageCopy, "Shooting a frame every 2 seconds",
                cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(255, 0, 0), 2);

        outputStream.PutFrame(imageCopy);
        server.SetSource(outputStream);

        if ((std::clock() - startTime) / CLOCKS_PER_SEC >= 2 &&
            ids.size() > 0) {
            std::cout << "Frame captured" << std::endl;
            allCorners.push_back(corners);
            allIds.push_back(ids);
            allImgs.push_back(image);
            imgSize = image.size();

            startTime = std::clock();
        }
    }

    if (allIds.size() < 1) {
        std::cerr << "Not enough captures for calibration!" << std::endl;
        std::abort();
    }

    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    double repError;

    if (calibrationFlags & cv::CALIB_FIX_ASPECT_RATIO) {
        cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        cameraMatrix.at<double>(0, 0) = aspectRatio;
    }

    // prepare data for calibration
    std::vector<std::vector<cv::Point2f> > allCornersConcatenated;
    std::vector<int> allIdsConcatenated;
    std::vector<int> markerCounterPerFrame;
    markerCounterPerFrame.reserve(allCorners.size());
    for (unsigned int i = 0; i < allCorners.size(); i++) {
        markerCounterPerFrame.push_back(static_cast<int>(allCorners[i].size()));
        for (unsigned int j = 0; j < allCorners[i].size(); j++) {
            allCornersConcatenated.push_back(allCorners[i][j]);
            allIdsConcatenated.push_back(allIds[i][j]);
        }
    }

    // calibrate camera using aruco markers
    double arucoRepErr;
    arucoRepErr = cv::aruco::calibrateCameraAruco(
        allCornersConcatenated, allIdsConcatenated, markerCounterPerFrame,
        board, imgSize, cameraMatrix, distCoeffs, cv::noArray(), cv::noArray(),
        calibrationFlags);

    // prepare data for charuco calibration
    int nFrames = static_cast<int>(allCorners.size());
    std::vector<cv::Mat> allCharucoCorners;
    std::vector<cv::Mat> allCharucoIds;
    std::vector<cv::Mat> filteredImages;
    allCharucoCorners.reserve(nFrames);
    allCharucoIds.reserve(nFrames);

    for (int i = 0; i < nFrames; i++) {
        // interpolate using camera parameters
        cv::Mat currentCharucoCorners, currentCharucoIds;
        cv::aruco::interpolateCornersCharuco(
            allCorners[i], allIds[i], allImgs[i], charucoboard,
            currentCharucoCorners, currentCharucoIds, cameraMatrix, distCoeffs);

        allCharucoCorners.push_back(currentCharucoCorners);
        allCharucoIds.push_back(currentCharucoIds);
        filteredImages.push_back(allImgs[i]);
    }

    if (allCharucoCorners.size() < 4) {
        std::cerr << "Not enough corners for calibration" << std::endl;
        std::abort();
    }

    // calibrate camera using charuco
    repError = cv::aruco::calibrateCameraCharuco(
        allCharucoCorners, allCharucoIds, charucoboard, imgSize, cameraMatrix,
        distCoeffs, rvecs, tvecs, calibrationFlags);

    bool saveOk =
        saveCameraParams(outputFile, imgSize, aspectRatio, calibrationFlags,
                         cameraMatrix, distCoeffs, repError);
    if (!saveOk) {
        std::cerr << "Cannot save output file" << std::endl;
        std::abort();
    }

    std::cout << "Rep Error: " << repError << std::endl;
    std::cout << "Rep Error Aruco: " << arucoRepErr << std::endl;
    std::cout << "Calibration saved to " << outputFile << std::endl;

    // show interpolated charuco corners for debugging
    if (showChessboardCorners) {
        startTime = std::clock();
        for (unsigned int frame = 0; frame < filteredImages.size(); frame++) {
            cv::Mat imageCopy = filteredImages[frame].clone();
            if (allIds[frame].size() > 0) {
                if (allCharucoCorners[frame].total() > 0) {
                    cv::aruco::drawDetectedCornersCharuco(
                        imageCopy, allCharucoCorners[frame],
                        allCharucoIds[frame]);
                }
                outputStream.PutFrame(imageCopy);

                if ((std::clock() - startTime) / CLOCKS_PER_SEC >= 5) {
                    frame++;
                    std::cout << "Debugging through Frames";
                    std::cout << "Frame: " << frame << std::endl;

                    startTime = std::clock();
                }
            }
        }
    }
}
