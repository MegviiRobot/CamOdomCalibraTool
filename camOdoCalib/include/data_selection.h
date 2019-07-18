#ifndef DATA_SELECTION_H
#define DATA_SELECTION_H

#endif

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>

#include <fstream>
#include <Eigen/Dense>  

class data_selection
{
public:
    struct odo_data
    {
        double time;
        double v_left;
        double v_right;
    };
    struct cam_data
    {
        double start_t;
        double end_t;
        //double theta_y;
        double deltaTheta;
        Eigen::Vector3d axis;
        Eigen::Matrix3d Rcl;
        Eigen::Vector3d tlc; 
    };
    /*
   * \brief The sync_data struct. Used for
   * storing synchronized data.
   */
  struct sync_data {
    // Period
    double T;
    // Left and right wheel velocities
    double velocity_left;
    double velocity_right;
    // double velocity;
    //camera data : x y yaw , x  y from tlc (not tcl)
    double scan_match_results[3];// correct lx ly by R_x
    // Estimated rototranslation based on odometry params.
    double o[3];
    // Estimated disagreement  sm - est_sm
    double est_sm[3];
    double err_sm[3]; //  s  - (-) l (+) o (+) l
    // Other way to estimate disagreement:   l (+) s  - o (+) l
    double err[3];
    int mark_as_outlier;
    //tcl_cam and qcl_cam are original data(not correted by R_x)
    Eigen::Vector3d tcl_cam;//06/06  
    Eigen::Quaterniond qcl_cam;
    double angle;
    Eigen::Vector3d axis;
    double startTime;

  };
    void startPosAlign(std::vector<odo_data>& odoDatas, std::vector<cam_data>& camDatas);
    void selectData(std::vector<odo_data> &odoDatas, std::vector<cam_data> &camDatas,
        std::vector<data_selection::sync_data> &sync_result);

    void camOdoAlign(std::vector<odo_data> &odoDatas, std::vector<cam_data> &camDatas,std::vector<sync_data> &sync_result);
     

    data_selection();

    
private:
};