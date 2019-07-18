#ifndef SIMULATEODOCAM_H
#define SIMULATEODOCAM_H

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <math.h>
#include <fstream>

struct odo_data
{
    double time;
    double lin_vel;
    double ang_vel;
    double distance;
    double theta;
};
struct cam_data
{
    double start_t;
    double end_t;
    double tcl_length;
    double theta_y;
    double deltaTheta;
    Eigen::Matrix3d Rcl;
    Eigen::Vector3d tlc;
    Eigen::Matrix3d R_x;//used for correcting the angle between y aixs and plane
};

void readParaFromFile(std::string filename, std::vector<double>& paras);
void generateDatas( std::vector<double> paras,
                  std::vector<Eigen::Matrix4d> &Twc_odos, 
                  std::vector<odo_data> &odoDatas,
                  Eigen::Matrix4d &Trc,
                  double &delta_t_odo);

void saveDatas(std::vector<Eigen::Matrix4d> Twc_odos, std::vector<odo_data> odoDatas,
                  Eigen::Matrix4d Trc,
                  double delta_t_odo,
                  std::string *path );




#endif