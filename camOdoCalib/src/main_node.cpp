/*******************************************************
 * Copyright (C) 2019, SLAM Group, Megvii-R
 *******************************************************/

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <stdio.h>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <thread>
#include <fstream>//zdf 
#include <math.h>
#include <chrono>

#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/image_encodings.h>

#include "camera_models/include/Camera.h"
#include "camera_models/include/CameraFactory.h"
#include "calc_cam_pose/calcCamPose.h"

#include "solveQyx.h" 

typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;

std::queue<OdomConstPtr> odo_buf;
std::queue<sensor_msgs::ImageConstPtr> img_buf;
std::mutex m_buf;
bool hasImg = false ;//zdf
std::vector<data_selection::odo_data> odoDatas;
std::vector<data_selection::cam_data> camDatas;

//record the first frame calculated successfully
bool fisrt_frame = true;
Eigen::Matrix3d Rwc0;
Eigen::Vector3d twc0;
//decide if the frequent is decreased
bool halfFreq = false;
int frame_index = 0;

void wheel_callback(const OdomConstPtr &odo_msg)
{
  double time = odo_msg->header.stamp.toSec();
  Eigen::Vector3d linear = { odo_msg->twist.twist.linear.x,
    odo_msg->twist.twist.linear.y,
    odo_msg->twist.twist.linear.z };
    Eigen::Vector3d angular = { odo_msg->twist.twist.angular.x,
      odo_msg->twist.twist.angular.y,
      odo_msg->twist.twist.angular.z };
      data_selection::odo_data odo_tmp;

    odo_tmp.time = time;
    odo_tmp.v_left = linear[0] / 0.1 - angular[2]*0.56 / (2*0.1);// linear velcity of x axis 
    odo_tmp.v_right = linear[0] / 0.1 + angular[2]*0.56 / (2*0.1);// angular velcity of z axis
    odoDatas.push_back(odo_tmp);
  }

void image_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
  if(!halfFreq)
  {
    m_buf.lock();
    img_buf.push(img_msg);
    m_buf.unlock(); 
  }
  else
  {
    frame_index++;
    if(frame_index % 2 ==  0)
    {
      m_buf.lock();
      img_buf.push(img_msg);
      m_buf.unlock(); 
    }
   } 
}

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
  cv_bridge::CvImageConstPtr ptr;
  if (img_msg->encoding == "8UC1")
  {
    sensor_msgs::Image img;
    img.header = img_msg->header;
    img.height = img_msg->height;
    img.width = img_msg->width;
    img.is_bigendian = img_msg->is_bigendian;
    img.step = img_msg->step;
    img.data = img_msg->data;
    img.encoding = "mono8";
    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  }
  else
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

  cv::Mat img = ptr->image.clone();

  return img;
}

// extract images with same timestamp from two topics
void calc_process(const CameraPtr &cam)
{
  Eigen::Matrix3d Rwl;
  Eigen::Vector3d twl;
  double t_last = 0.0;//time of last image
  bool first = true; //judge if last frame was calculated successfully 

  std::cout << std::endl << "images counts waiting for processing: " << std::endl;
  while (1)
  {
        cv::Mat image;
        std_msgs::Header header;
        double time = 0;
        m_buf.lock();
        if (!img_buf.empty())
        {
          time = img_buf.front()->header.stamp.toSec();
          header = img_buf.front()->header;
          image = getImageFromMsg(img_buf.front());
          img_buf.pop();
        }
        m_buf.unlock();
        if (!image.empty())
        {
          hasImg = true;
          Eigen::Matrix4d Twc;

          int leftImg = img_buf.size();
          std::cout.width(5);
          std::cout << leftImg;

          bool isCalOk = calcCamPose(time, image, cam, Twc);

          std::cout << "\b\b\b\b\b";
          if(!isCalOk)//zdf
          {
            first = true;
            continue;
          }
          Eigen::Matrix3d Rwc = Twc.block<3, 3>(0, 0);
          Eigen::Vector3d twc = Twc.block<3, 1>(0, 3);

          if(fisrt_frame)
          {
            Rwc0 = Rwc;
            twc0 = twc;
            fisrt_frame = false;
            continue;
          }
          //judge if the last frame was calculated successfully
          if (!first)
          {
            //Todo zdf
            // Eigen::Vector3d eulerAngle_wc = Rwc.eulerAngles(2,1,0);//ZYX
            // Eigen::Vector3d eulerAngle_wl = Rwl.eulerAngles(2,1,0);
            // double theta_y = eulerAngle_wc[1] - eulerAngle_wl[1];

            Eigen::Matrix3d Rcl = Rwc.inverse() * Rwl;
            Eigen::Quaterniond q_cl(Rcl);

            Eigen::AngleAxisd rotation_vector(q_cl);
            Eigen::Vector3d axis = rotation_vector.axis();
            double deltaTheta_cl = rotation_vector.angle();  
            if(axis(1)>0)
            {
              deltaTheta_cl *= -1;
              axis *= -1;
            } 

            //Eigen::Vector3d tcl = -Rwc.inverse() * (twc - twl);
            Eigen::Vector3d tlc = -Rwl.inverse() * (twl - twc);

            data_selection::cam_data cam_tmp;
            cam_tmp.start_t = t_last;
            cam_tmp.end_t = time;
            //cam_tmp.theta_y = theta_y;
            cam_tmp.deltaTheta = -deltaTheta_cl; // cam_tmp.deltaTheta is deltaTheta_lc
            cam_tmp.axis = axis;
            cam_tmp.Rcl =  Rcl;
            cam_tmp.tlc =  tlc;
            camDatas.push_back(cam_tmp);
          }
          t_last = time;
          Rwl = Rwc;
          twl = twc;
          first = false;
        }
        else
        {
            if(hasImg)
            {
              SolveQyx cSolveQyx;

              std::cout << "============ calibrating... ===============" << std::endl;
              data_selection ds;
              std::vector<data_selection::sync_data> sync_result;
              ds.selectData(odoDatas,camDatas,sync_result);

              //first estimate the Ryx and correct tlc of camera
              Eigen::Matrix3d Ryx;
              cSolveQyx.estimateRyx(sync_result,Ryx);
              cSolveQyx.correctCamera(sync_result,camDatas,Ryx);

              //calibrate r_L  r_R  axle  lx  ly  yaw
              cSolver cSolve;
              cSolver::calib_result paras;//radius_l,radius_r,axle,l[3]
              cSolve.calib(sync_result, 4,paras); // by svd

               //secondly estimate the Ryx
              cSolveQyx.estimateRyx(sync_result,Ryx);

              //refine all the extrinal parameters
              cSolveQyx.refineExPara(sync_result,paras,Ryx);
              
              break;
          }  
      }

    }

    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_fusion");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    std::string config_file = argv[1];
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
      std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    std::string WHEEL_TOPIC, IMAGE_TOPIC;
    fsSettings["wheel_topic"] >> WHEEL_TOPIC;
    fsSettings["image_topic"] >> IMAGE_TOPIC;

    CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(config_file);

    //the following three rows are to run the calibrating project through playing bag package
    ros::Subscriber sub_imu = n.subscribe(WHEEL_TOPIC, 500, wheel_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 200, image_callback);
    std::thread calc_thread = std::thread{calc_process, std::ref(camera)};
    
     ros::spin();
     return 0;
}
