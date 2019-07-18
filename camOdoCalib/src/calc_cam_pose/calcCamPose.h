#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "../camera_models/include/Camera.h"
#include "ethz_apriltag/Tag36h11.h"
#include "ethz_apriltag/TagDetector.h"

enum PatternType
{
    APRIL,
    CHESS,
    CIRCLE
};

void FindTargetCorner(cv::Mat &img_raw, const PatternType &pt,
                      std::vector<cv::Point3f> &p3ds,
                      std::vector<cv::Point2f> &p2ds);
bool EstimatePose(const std::vector<cv::Point3f> &p3ds,
                  const std::vector<cv::Point2f> &p2ds, const double &fx,
                  const double &cx, const double &fy, const double &cy,
                  Eigen::Matrix3d &Rwc, Eigen::Vector3d &twc, cv::Mat &img_raw,const CameraPtr &cam);
bool calcCamPose(const double &timestamps, const cv::Mat &image,
                 const CameraPtr &cam, Eigen::Matrix4d &Twc);