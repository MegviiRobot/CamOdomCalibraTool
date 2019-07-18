#include "simulateOdoCam.h"
#include <random>

#define PI 3.1415926

void readParaFromFile(std::string filename, std::vector<double>& paras)
{
  double freq_cam , freq_odo;
  double v_left,v_right;
  double radius_l , radius_r, axle;
  double euler_z , euler_y , euler_x, t0, t1, t2;//Rrc, trc;

   cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    return;
  }
  freq_cam = static_cast<double>(fs["freq_cam"]);
  freq_odo = static_cast<double>(fs["freq_odo"]);
  paras.push_back(freq_cam);
  paras.push_back(freq_odo);

  v_left = static_cast<double>(fs["velocity_left"]);
  v_right = static_cast<double>(fs["velocity_right"]);
  paras.push_back(v_left);
  paras.push_back(v_right);

  radius_l = static_cast<double>(fs["radius_l"]);
  radius_r = static_cast<double>(fs["radius_r"]);
  axle = static_cast<double>(fs["axle"]);
  paras.push_back(radius_l);
  paras.push_back(radius_r);
  paras.push_back(axle);

  euler_z = static_cast<double>(fs["euler_z"]);
  euler_y = static_cast<double>(fs["euler_y"]);
  euler_x = static_cast<double>(fs["euler_x"]);
  t0 = static_cast<double>(fs["t0"]);
  t1 = static_cast<double>(fs["t1"]);
  t2 = static_cast<double>(fs["t2"]);
  paras.push_back(euler_z);
  paras.push_back(euler_y);
  paras.push_back(euler_x);
  paras.push_back(t0);
  paras.push_back(t1);
  paras.push_back(t2);

}

void generateDatas( std::vector<double> paras,
                  std::vector<Eigen::Matrix4d> &Twc_odos, 
                  std::vector<odo_data> &odoDatas,
                  Eigen::Matrix4d &Trc,
                  double &delta_t_odo)
{
  if(paras.empty()) return;

  double freq_cam = paras[0] , freq_odo = paras[1];
  double v_left = paras[2],v_right = paras[3];
  double radius_l = paras[4] , radius_r = paras[5], axle = paras[6];
  double euler_z = paras[7] , euler_y = paras[8] , euler_x = paras[9], t0 = paras[10], t1 = paras[11], t2 = paras[12];//Rrc, trc;

  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(euler_x,Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(euler_y,Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(euler_z,Eigen::Vector3d::UnitZ()));             
  Eigen::Quaterniond qrc = yawAngle*pitchAngle*rollAngle;
  Eigen::Matrix3d Rrc = qrc.matrix();
  Eigen::Vector3d trc = {t0,t1,t2};
  Trc = Eigen::Matrix4d::Identity();
  Trc.block<3,3>(0,0) = Rrc;
  Trc.block<3,1>(0,3) = trc;

  // Define random generator with Gaussian distribution
  const double mean = 0.0;//均值
  const double stddev = 0.05;//标准差
  std::default_random_engine generator;
  std::normal_distribution<double> dist(mean, stddev);
  
  //double delta_t_cam = 1.0 / freq_cam;
  delta_t_odo = 1.0 / freq_odo; //0.01
  int datas_num = 3000;
  double  theta = 0.0;

  Eigen::Matrix4d Twl_odo = Eigen::Matrix4d::Identity();
  for (double t = 0; t < double(datas_num*delta_t_odo); t += delta_t_odo)
  {
    double v_left_tmp = v_left * cos(t) ;
    double v_right_tmp = v_right * sin(t);
    double lin_vel = 0.5*(v_left_tmp*radius_l + v_right_tmp*radius_r);
    double ang_vel = (v_right_tmp*radius_r - v_left_tmp*radius_l) / axle;

    std::cout << "v: " << v_left_tmp << " " << v_right_tmp << std::endl;
    v_left_tmp += dist(generator);
    v_right_tmp += dist(generator);
    std::cout << "v2: " << v_left_tmp <<" " << v_right_tmp << std::endl << std::endl;
    double lin_vel_noise = 0.5*(v_left_tmp*radius_l + v_right_tmp*radius_r);
    double ang_vel_noise = (v_right_tmp*radius_r - v_left_tmp*radius_l) / axle;

    theta = ang_vel * delta_t_odo;
    Eigen::AngleAxisd roll_tmp(Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitch_tmp(Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yaw_tmp(Eigen::AngleAxisd(theta,Eigen::Vector3d::UnitZ()));             
    Eigen::Quaterniond qlc_odo = yaw_tmp*pitch_tmp*roll_tmp;
    double t1,t2;
    if (fabs(theta) > 1e-12) 
    {
      t1 = sin(theta) / theta; //cos
      t2 = (1 - cos(theta)) / theta; //sin
    }
    else {
      t1 = 1;
      t2 = 0;
    }

    odo_data odo_tmp;
    odo_tmp.time = t;
    odo_tmp.lin_vel = lin_vel ;
    odo_tmp.ang_vel = ang_vel;
    //std::cout << "2 lin ang: " << odo_tmp.lin_vel << " " << odo_tmp.ang_vel << " " <<std::endl;
    odo_tmp.distance = odo_tmp.lin_vel * delta_t_odo;
    odo_tmp.theta = odo_tmp.ang_vel*delta_t_odo;
    odoDatas.push_back(odo_tmp);

    /*Eigen::Vector3d twc_odo = { r*sin(theta),r*cos(theta)  ,0};
    Eigen::Matrix4d Twc_odo = Eigen::Matrix4d::Identity();
    Twc_odo.block<3,3>(0,0) = qwc_odo.matrix();
    Twc_odo.block<3,1>(0,3) = twc_odo;*/
    Eigen::Matrix4d Tlc_odo = Eigen::Matrix4d::Identity();
    Tlc_odo.block<3,3>(0,0) = qlc_odo.matrix();
    Tlc_odo.block<2,1>(0,3) = Eigen::Vector2d(lin_vel*delta_t_odo*t1 , lin_vel*delta_t_odo*t2);

    Eigen::Matrix4d Twc_odo = Twl_odo*Tlc_odo;
    Twc_odos.push_back(Twc_odo);

    Twl_odo = Twc_odo;    
 }

}

void saveDatas(std::vector<Eigen::Matrix4d> Twc_odos, std::vector<odo_data> odoDatas,
                  Eigen::Matrix4d Trc,
                  double delta_t_odo,
                  std::string *path )
{
  if(Twc_odos.size() == 0)   {   std::cout << "!!!! Twc_odos.size() == 0 !!!" << std::endl;  return;  }

  std::vector<cam_data> camDatas;
  int n = Twc_odos.size();
  double t = 0.0;
  bool first = true;
  int count = 0;
  Eigen::Matrix4d Twl_cam;

  const double mean = 0.0;//均值
  const double stddev = 0.0005;//标准差
  std::default_random_engine generator;
  std::normal_distribution<double> dist(mean, stddev);
  for (int i = 0; i < n; i++)
  {
    Eigen::Matrix4d Twc_cam = Twc_odos[i] * Trc;
    // std::cout << "Twc_odo : " << Twc_odos[i] << std::endl;
    //std::cout << "Twc_cam : " << Twc_cam.block<3,3>(0,0).eulerAngles(2,1,0) << std::endl << std::endl;

    //add noise
    Eigen::AngleAxisd roll_tmp(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitch_tmp(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yaw_tmp(Eigen::AngleAxisd(dist(generator),Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond delta_q = yaw_tmp*pitch_tmp*roll_tmp;
    Eigen::Vector3d delta_t(dist(generator), dist(generator), 0);
    // Twc_cam.block<3,3>(0,0) = Twc_cam.block<3,3>(0,0) * delta_q.matrix();
    // Twc_cam.block<3,1>(0,3) = Twc_cam.block<3,1>(0,3) + delta_t;

    if(first) {  t +=  delta_t_odo;   Twl_cam = Twc_cam;   count++;   first = false ;   continue;   }

    if(count % 2 == 0)
    {
      Eigen::Matrix4d Tlc_cam = Twl_cam.inverse() * Twc_cam;
      Eigen::Matrix3d Rlc = Tlc_cam.block<3,3>(0,0);
      Eigen::Vector3d tlc = Tlc_cam.block<3,1>(0,3);

      Eigen::Vector3d eulerAngle_wc = Twc_cam.block<3,3>(0,0).eulerAngles(2,1,0);//ZYX
      Eigen::Vector3d eulerAngle_wl = Twl_cam.block<3,3>(0,0).eulerAngles(2,1,0);
      double theta_y = eulerAngle_wc[0] - eulerAngle_wl[0];
      Eigen::Quaterniond qlc(Rlc);
      Eigen::AngleAxisd rotation_vector(qlc);
      double deltaTheta = double(rotation_vector.angle());

      double angle_x = -0.012;
      Eigen::Quaterniond q_x(cos(angle_x/2.0),sin(angle_x/2.0),0,0);

      cam_data cam_tmp;
      cam_tmp.start_t = t - 2*delta_t_odo;
      cam_tmp.end_t = t;
      cam_tmp.tcl_length = sqrt(tlc[0]*tlc[0] + tlc[1]*tlc[1] + tlc[2]*tlc[2]);
      cam_tmp.theta_y = theta_y;
      cam_tmp.deltaTheta = deltaTheta;
      cam_tmp.Rcl = Rlc.inverse();
      cam_tmp.tlc = tlc;
      //cam_tmp.R_x = Eigen::Matrix3d::Identity();
      cam_tmp.R_x = q_x.matrix();
      camDatas.push_back(cam_tmp);

      Twl_cam = Twc_cam;
    }

    t += delta_t_odo; 
    count++;
  }
   
  //save camDatas and odoDatas
  std::string file_cam = path[0];
  std::string file_odo = path[1];
  std::ofstream ofile_cam(file_cam.c_str());
  std::ofstream ofile_odo(file_odo.c_str());
  ofile_cam << "start_t,end_t,tcl_length,theta_y,deltaTheta,qcl,tlc,q_x" << std::endl;
  for (unsigned int i = 0; i < camDatas.size(); ++i)
  {
     cam_data &t = camDatas[i];
     Eigen::Quaterniond qcl(t.Rcl);
     Eigen::Quaterniond qx(t.R_x);
     ofile_cam <<std::fixed<< t.start_t<<"," <<t.end_t<<","<<t.tcl_length<<","<<t.theta_y<<","<<t.deltaTheta<<","<<qcl.w()<<","<<
     qcl.x()<<","<<qcl.y()<<","<<qcl.z()<<","<<t.tlc[0]<<","<<t.tlc[1]<<","<<t.tlc[2]<<","<<qx.w()<<","<<qx.x()<<","<<qx.y()<<
     ","<<qx.z()<<std::endl;
   }
   ofile_cam.close();
   ofile_odo <<"time,lin_vel,ang_vel,distance,theta"<<std::endl;
   for (unsigned int i = 0; i < odoDatas.size(); ++i)
   {
      odo_data &t = odoDatas[i];
      ofile_odo << std::fixed << t.time <<","<<t.lin_vel<<","<<t.ang_vel<<","<<t.distance<<","<<t.theta<<std::endl;
  }
  ofile_odo.close();

}

//read timestamps of camera, paint the graph of d-value of timestamp
/*void piantTimestamp(std::string filename)
{
  std::ifstream f;
  f.open(filename.c_str());

  if(!f.is_open())
  {
    std::cerr << " can't open cam pose file "<<std::endl;
    return;
  }
  std::string s;
  std::getline(f,s);
  bool first = true;
  double t_last = 0.0;
  std::vector<double> delta_times;
  while (!f.eof()) 
  {
    std::getline(f,s);
    if(! s.empty())
    {
      std::stringstream ss;
      ss << s;
      double t_cur;
      s >> t_cur;

      if(first) { t_last = t_cur; first = false; continue; }

      delta_times.push_back(t_cur - t_last);

      t_last = t_cur;
    }
}*/