// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include <cameraserver/CameraServer.h>

#include <ctime>
#include <iostream>

#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

/**
 * Read camera distortion parameters from ChAruco calibration
 *
 * @param filename Location of file
 * @param camMatrix Camera matrix
 * @param distCoeffs
 */
static bool readCameraParameters(std::string filename, cv::Mat& camMatrix,
                                 cv::Mat& distCoeffs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

/**
 * Create Aruco board to use
 */
void createArucoBoard() {
    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::drawMarker(dictionary, 23, 8 * 300, markerImage, 1);
    cv::imwrite("marker23.png", markerImage);
}

/**
 * Thing get pose send pose
 */
int main() {
    cs::UsbCamera cvCamera =
        frc::CameraServer::GetInstance()->StartAutomaticCapture();
    cvCamera.SetResolution(640, 480);

    cv::Mat cameraMatrix, distCoeffs;

    cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
    cs::MjpegServer server{"Aruco Pose Estimation", 1128};
    cs::CvSource outputStream =
        frc::CameraServer::GetInstance()->PutVideo("Calibration", 640, 480);

    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
    auto poseEntry = inst.GetEntry("/Vision/target-pose");
    auto timestampEntry = inst.GetEntry("/Vision/timestamp");

    // camera parameters are read from somewhere
    if (!readCameraParameters("CameraCalibration.txt", cameraMatrix,
                              distCoeffs)) {
        throw "Camera Parameters didn't load";
    }

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    while (true) {
        std::clock_t time = std::clock();
        cv::Mat image, imageCopy;
        cvSink.GrabFrame(image, 1);
        image.copyTo(imageCopy);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);

        // Set pose to zero if nothing is seen
        poseEntry.SetDoubleArray({0, 0, 0});

        // if at least one marker detected
        if (ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix,
                                                 distCoeffs, rvecs, tvecs);
            cv::Vec3d newRvecs;
            cv::Rodrigues(rvecs, newRvecs);

            // draw axis for each marker
            for (int i = 0; i < ids.size(); i++)
                cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs,
                                    rvecs[i], tvecs[i], 0.1);

            poseEntry.SetDoubleArray(
                {tvecs.at(0)[0], tvecs.at(0)[1], newRvecs[0]});
            timestampEntry.SetDouble(time / CLOCKS_PER_SEC);
        }
        outputStream.PutFrame(imageCopy);
        server.SetSource(outputStream);
    }
}
