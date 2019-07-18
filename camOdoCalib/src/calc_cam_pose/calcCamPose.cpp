#include "calcCamPose.h"

void FindTargetCorner(cv::Mat &img_raw, const PatternType &pt,
                      std::vector<cv::Point3f> &p3ds,
                      std::vector<cv::Point2f> &p2ds)
{
  const int col = 9;
  const int row = 6;
  if (CHESS == pt)
  {
    // std::cout << "CHESSBOARD\n";
    const float square_size = 0.575; // unit:  m
    cv::Size pattern_size(col, row);
    std::vector<cv::Point2f> corners;
    if (cv::findChessboardCorners(img_raw, pattern_size, corners))
    {
      cv::cornerSubPix(
          img_raw, corners, cv::Size(11, 11), cv::Size(-1, -1),
          cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
      if (corners.size() == col * row)
      {
        int count = 0;
        for (int i = 0; i < row; i++)
        {
          for (int j = 0; j < col; j++)
          {
            // Todo: change 3d-coordinate
            p3ds.emplace_back(
                cv::Point3f(j * square_size, i * square_size, 0.0));
            p2ds.emplace_back(cv::Point2f(corners[count].x, corners[count].y));
            count++;
          }
        }
      }
      else
      {
        std::cout << "Chessboard config is not correct with image\n";
      }
    }
    else
    {
      std::cout << "No chessboard detected in image\n";
    }
  }
  else if (APRIL == pt)
  {
    //const int april_rows = 6;
    const int april_cols = 6;
    const double tag_sz = 0.055;
    const double tag_spacing_sz = 0.0715; // 0.055 + 0.0165

    AprilTags::TagCodes tagCodes(AprilTags::tagCodes36h11);
    AprilTags::TagDetector detector(tagCodes);
    std::vector<AprilTags::TagDetection> detections =
        detector.extractTags(img_raw);
    //if (detections.size() == april_rows * april_cols)
    if (detections.size() > 20)
    {
      std::sort(detections.begin(), detections.end(),
                AprilTags::TagDetection::sortByIdCompare);
      cv::Mat tag_corners(4 * detections.size(), 2, CV_32F);
      for (unsigned i = 0; i < detections.size(); i++)
      {
        for (unsigned j = 0; j < 4; j++)
        {
          tag_corners.at<float>(4 * i + j, 0) = detections[i].p[j].first;
          tag_corners.at<float>(4 * i + j, 1) = detections[i].p[j].second;
        }
      }
      cv::cornerSubPix(
          img_raw, tag_corners, cv::Size(2, 2), cv::Size(-1, -1),
          cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
      cv::cvtColor(img_raw, img_raw, CV_GRAY2BGR);

      // draw axis
      /*double center_u = tag_corners.at<float>(0, 0);
      double center_v = tag_corners.at<float>(0, 1);
      cv::Point2f center(center_u, center_v);
      double xaxis_u = tag_corners.at<float>(20, 0);
      double xaxis_v = tag_corners.at<float>(20, 1);
      cv::Point2f xaxis(xaxis_u + (xaxis_u - center_u) * 0.5,
                        xaxis_v + (xaxis_v - center_v) * 0.5);
      double yaxis_u = tag_corners.at<float>(120, 0);
      double yaxis_v = tag_corners.at<float>(120, 1);
      cv::Point2f yaxis(yaxis_u + (yaxis_u - center_u) * 0.5,
                        yaxis_v + (yaxis_v - center_v) * 0.5);
      cv::Point2f zaxis(center_u - (yaxis_u - center_u) * 0.5,
                        center_v - (xaxis_v - center_v) * 0.5);
      cv::line(img_raw, center, xaxis, cv::Scalar(0, 0, 255), 2);
      cv::putText(img_raw, "X", xaxis, CV_FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(0, 0, 255));
      cv::line(img_raw, center, yaxis, cv::Scalar(0, 255, 0), 2);
      cv::putText(img_raw, "Y", yaxis, CV_FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(0, 255, 0));
      cv::line(img_raw, center, zaxis, cv::Scalar(255, 0, 0), 2);
      cv::putText(img_raw, "Z", zaxis, CV_FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(255, 0, 0));*/

      int index = 0;
      for (auto &tag : detections)
      {
        unsigned int id = tag.id;
        unsigned int tag_row = id / april_cols;
        unsigned int tag_col = id % april_cols;

        cv::circle(img_raw, cv::Point2f(tag.cxy.first, tag.cxy.second), 3,
                   cv::Scalar(0, 255, 0));
        cv::putText(img_raw, std::to_string(id),
                    cv::Point2f(tag.cxy.first, tag.cxy.second),
                    CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));

        p3ds.emplace_back(cv::Point3f(tag_spacing_sz * tag_col,
                                      tag_spacing_sz * tag_row, 0.0));
        p2ds.emplace_back(cv::Point2f(tag_corners.at<float>(index, 0),
                                      tag_corners.at<float>(index, 1)));
        // cv::circle(img_raw, p2ds.back(), 3, cv::Scalar(0, 255, 0));
        // std::cout << index++ << " " << p3ds.back() << std::endl;
        // cv::imshow("apriltag_detection", img_raw);
        // cv::waitKey(0);
        ++index;
        p3ds.emplace_back(cv::Point3f(tag_spacing_sz * tag_col + tag_sz,
                                      tag_spacing_sz * tag_row, 0.0));
        p2ds.emplace_back(cv::Point2f(tag_corners.at<float>(index, 0),
                                      tag_corners.at<float>(index, 1)));
        // cv::circle(img_raw, p2ds.back(), 3, cv::Scalar(0, 255, 0));
        // std::cout << index++ << " " << p3ds.back() << std::endl;
        // cv::imshow("apriltag_detection", img_raw);
        // cv::waitKey(0);
        ++index;
        p3ds.emplace_back(cv::Point3f(tag_spacing_sz * tag_col + tag_sz,
                                      tag_spacing_sz * tag_row + tag_sz, 0.0));
        p2ds.emplace_back(cv::Point2f(tag_corners.at<float>(index, 0),
                                      tag_corners.at<float>(index, 1)));
        // cv::circle(img_raw, p2ds.back(), 3, cv::Scalar(0, 255, 0));
        // std::cout << index++ << " " << p3ds.back() << std::endl;
        // cv::imshow("apriltag_detection", img_raw);
        // cv::waitKey(0);
        ++index;
        p3ds.emplace_back(cv::Point3f(tag_spacing_sz * tag_col,
                                      tag_spacing_sz * tag_row + tag_sz, 0.0));
        p2ds.emplace_back(cv::Point2f(tag_corners.at<float>(index, 0),
                                      tag_corners.at<float>(index, 1)));
        // cv::circle(img_raw, p2ds.back(), 3, cv::Scalar(0, 255, 0));
        // std::cout << index++ << " " << p3ds.back() << std::endl;
        // cv::imshow("apriltag_detection", img_raw);
        // cv::waitKey(0);
        ++index;
      }
    }
  }
  else
  {
    std::cout << "Pattern type not supported yet.\n";
  }
}

bool EstimatePose(const std::vector<cv::Point3f> &p3ds,
                  const std::vector<cv::Point2f> &p2ds, const double &fx,
                  const double &cx, const double &fy, const double &cy,
                  Eigen::Matrix4d &Twc, cv::Mat &img_raw,const CameraPtr &cam)
{
  if (p3ds.size() != p2ds.size() || p3ds.size() < 4)
  {
    return false;
  }

  cv::Mat_<float> K =
      (cv::Mat_<float>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
  cv::Mat_<float> dist = (cv::Mat_<float>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
  cv::Mat cv_r, cv_t;
  cv::Mat inliers;
  cv::solvePnP(p3ds, p2ds, K, dist, cv_r, cv_t);
  cv::Mat rotation;
  cv::Rodrigues(cv_r, rotation);
  Eigen::Matrix3d Rcw;
  cv::cv2eigen(rotation, Rcw);
  Eigen::Vector3d tcw;
  cv::cv2eigen(cv_t, tcw);
  Twc.block<3, 3>(0, 0) = Rcw.inverse();
  Twc.block<3, 1>(0, 3) = -Rcw.inverse() * tcw;

  std::vector<Eigen::Vector3d> axis;
  axis.push_back(Rcw * Eigen::Vector3d(0, 0, 0) + tcw);
  axis.push_back(Rcw * Eigen::Vector3d(0.5, 0, 0) + tcw);
  axis.push_back(Rcw * Eigen::Vector3d(0, 0.5, 0) + tcw);
  axis.push_back(Rcw * Eigen::Vector3d(0, 0, 0.5) + tcw);
  std::vector<Eigen::Vector2d> imgpts(4);
  for (int i = 0; i < 4; ++i)
  {
    cam->spaceToPlane(axis[i], imgpts[i]);
  }
  // cv::projectPoints(axis, cv_r, cv_t, K, dist, imgpts);
  cv::line(img_raw, cv::Point2f(imgpts[0](0), imgpts[0](1)), cv::Point2f(imgpts[1](0), imgpts[1](1)), cv::Scalar(255, 0, 0), 2);//BGR
  cv::line(img_raw, cv::Point2f(imgpts[0](0), imgpts[0](1)), cv::Point2f(imgpts[2](0), imgpts[2](1)), cv::Scalar(0, 255, 0), 2);
  cv::line(img_raw, cv::Point2f(imgpts[0](0), imgpts[0](1)), cv::Point2f(imgpts[3](0), imgpts[3](1)), cv::Scalar(0, 0, 255), 2);
  cv::putText(img_raw, "X", cv::Point2f(imgpts[1](0), imgpts[1](1)), CV_FONT_HERSHEY_SIMPLEX, 0.5,cv::Scalar(255, 0, 0));
  cv::putText(img_raw, "Y", cv::Point2f(imgpts[2](0), imgpts[2](1)), CV_FONT_HERSHEY_SIMPLEX, 0.5,cv::Scalar(0, 255, 0));
  cv::putText(img_raw, "Z", cv::Point2f(imgpts[3](0), imgpts[3](1)), CV_FONT_HERSHEY_SIMPLEX, 0.5,cv::Scalar(0, 0, 255));

  cv::putText(
      img_raw, "t_wc: (m) " + std::to_string(Twc(0, 3)) + " " + std::to_string(Twc(1, 3)) + " " + std::to_string(Twc(2, 3)),
      cv::Point2f(50, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0));

  return true;
}

bool calcCamPose(const double &timestamps, const cv::Mat &image,
                 const CameraPtr &cam, Eigen::Matrix4d &Twc)
{
  cv::Mat img_raw = image.clone();
  if (img_raw.channels() == 3)
  {
    cv::cvtColor(img_raw, img_raw, CV_BGR2GRAY);
  }

  std::vector<cv::Point3f> p3ds;
  std::vector<cv::Point2f> p2ds;
  // FindTargetCorner(img_raw, CHESS, p3ds, p2ds);
  FindTargetCorner(img_raw, APRIL, p3ds, p2ds);

  std::vector<double> p = cam->getK();
  // std::cout << p[0] << " " << p[1] << " " << p[2] << " " << p[3] << std::endl;
  std::vector<cv::Point2f> un_pts;
  for (int i = 0, iend = (int)p2ds.size(); i < iend; ++i)
  {
    Eigen::Vector2d a(p2ds[i].x, p2ds[i].y);
    Eigen::Vector3d b;
    cam->liftProjective(a, b);
    un_pts.push_back(
        cv::Point2f(p[0] * b.x() / b.z() + p[2], p[1] * b.y() / b.z() + p[3]));
    // std::cout << "p2ds: " << p2ds[i] << std::endl;
    // std::cout << "un_pts: " << un_pts[i] << std::endl;
  }

  if (EstimatePose(p3ds, un_pts, p[0], p[2], p[1], p[3], Twc, img_raw,cam))
  {
     cv::imshow("apriltag_detection & camPose_calculation", img_raw);
     cv::waitKey(1);
    return true;
  }
  else
  {
    return false;
  }
}