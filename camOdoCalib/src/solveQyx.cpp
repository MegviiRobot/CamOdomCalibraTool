#include "solveQyx.h"
#include <Eigen/Cholesky>

#define Pi 3.1415926

SolveQyx::SolveQyx(){}

bool SolveQyx::estimateRyx(std::vector<data_selection::sync_data> sync_result, Eigen::Matrix3d &Ryx)
{
  size_t motionCnt = sync_result.size( );
  Eigen::MatrixXd M(motionCnt*4, 4); 
  M.setZero();

  for(size_t i = 0; i < sync_result.size(); ++i)
  {
    const Eigen::Vector3d& axis = sync_result[i].axis;

    Eigen::Matrix4d M_tmp;
    M_tmp << 0, -1-axis[2], axis[1], -axis[0],
                        axis[2]+1, 0, -axis[0], -axis[1],
                        -axis[1], axis[0], 0 , 1-axis[2],
                        axis[0], axis[1], -1+axis[2], 0;
    M.block<4,4>(i*4, 0) = sin(sync_result[i].angle)*M_tmp;
  }
  //M.conservativeResize((id-1)*4,4);

          //TODO:: M^T * M
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU |Eigen::ComputeFullV);

    Eigen::Vector4d v1 = svd.matrixV().block<4,1>(0,2);
    Eigen::Vector4d v2 = svd.matrixV().block<4,1>(0,3);

    double lambda[2];      // solution from  ax^2+ bx + c = 0
    if( !SolveConstraintqyx(v1,v2,lambda[0],lambda[1]))
    {
        std::cout << "# ERROR: Quadratic equation cannot be solved due to negative determinant." << std::endl;
        return false;
    }

   // choose one lambda
    Eigen::Matrix3d R_yxs[2];
    double yaw[2];

    for( int i = 0; i<2;++i)
    {
        double t = lambda[i] * lambda[i] * v1.dot(v1) + 2 * lambda[i] * v1.dot(v2) + v2.dot(v2);

              // solve constraint ||q_yx|| = 1
        double lambda2 = sqrt(1.0/t);
        double lambda1 = lambda[i] * lambda2;

        Eigen::Quaterniond q_yx;
        q_yx.coeffs() = lambda1 * v1 + lambda2 * v2; // x,y,z,w

        R_yxs[i] = q_yx.toRotationMatrix();
        double roll,pitch;
        mat2RPY(R_yxs[i], roll, pitch, yaw[i]);
        //std::cout<<"roll: "<<roll<<" pitch: "<<pitch<<" yaw: "<<yaw[i]<<std::endl;
    }

    // q_yx  means yaw is zero. we choose the smaller yaw
    if(fabs(yaw[0]) < fabs(yaw[1]) )
    {
      Ryx = R_yxs[0];
    }else
    {
      Ryx = R_yxs[1];
    }
    return true;
}

void SolveQyx::correctCamera(std::vector<data_selection::sync_data> &sync_result, std::vector<data_selection::cam_data> &camDatas,Eigen::Matrix3d Ryx)
{
  if(sync_result.size() != camDatas.size())
  {
    std::cerr << "ERROR!! correctCamera: sync_result.size() != camDatas.size()" << std::endl;
    return;
  }
  std::vector<data_selection::sync_data> sync_tmp;
  std::vector<data_selection::cam_data> cam_tmp;
  for (unsigned int i = 0; i < sync_result.size(); ++i)
  {
    Eigen::Vector3d tlc_cam = camDatas[i].tlc;
    Eigen::Vector3d tlc_corrected = Ryx * tlc_cam;
    if(tlc_corrected(1)*tlc_cam(2) < 0)
      continue;

    sync_result[i].scan_match_results[0] = tlc_corrected[0];
    sync_result[i].scan_match_results[1] = tlc_corrected[1];
    sync_tmp.push_back(sync_result[i]);
    cam_tmp.push_back(camDatas[i]);
  }
  sync_result.swap(sync_tmp);
  camDatas.swap(cam_tmp);
}

void SolveQyx::refineExPara(std::vector<data_selection::sync_data> sync_result,
                    cSolver::calib_result &internelPara,Eigen::Matrix3d Ryx)
{
    std::cout << std::endl << "there are  "<< sync_result.size() << " datas for refining extrinsic paras" << std::endl;
    std::vector<Eigen::Quaterniond> q_cam , q_odo;
    std::vector<Eigen::Vector3d> t_cam , t_odo;
    double r_L = internelPara.radius_l, r_R = internelPara.radius_r, axle = internelPara.axle;
    Eigen::Vector2d trc;
    trc << internelPara.l[0] , internelPara.l[1];
    for (int i = 0; i < int(sync_result.size()) - 3; ++i)
    {
        q_cam.push_back(sync_result[i].qcl_cam);
        t_cam.push_back(sync_result[i].tcl_cam);

        double vel_L = sync_result[i].velocity_left, vel_R = sync_result[i].velocity_right;
        double v = 0.5 * (r_L* vel_L + r_R * vel_R)                                 ,   omega = (r_R * vel_R - r_L * vel_L) / axle;
        Eigen::Quaterniond qlc_odo;
        Eigen::Vector3d tlc_odo, tcl_odo;
        double o_theta = omega * sync_result[i].T;
        double t1,t2;
        if (fabs(o_theta) > 1e-12) 
        {
          t1 = sin(o_theta) / o_theta;
          t2 = (1 - cos(o_theta)) / o_theta;
        }
        else {
          t1 = 1;
          t2 = 0;
        }
        Eigen::Vector3d eulerAngle(0.0,0.0,o_theta);
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitZ()));             
        qlc_odo = yawAngle*pitchAngle*rollAngle;
        tlc_odo = {v * sync_result[i].T * t1, v * sync_result[i].T * t2, 0.0};
        tcl_odo = -qlc_odo.matrix().inverse() * tlc_odo;
        Eigen::Quaterniond qcl_odo(qlc_odo.matrix().inverse());
        
        q_odo.push_back(qcl_odo);
        t_odo.push_back(tcl_odo);

    }

    Eigen::Matrix3d Rrc = Eigen::AngleAxisd(internelPara.l[2], Eigen::Vector3d::UnitZ() ) * Ryx;
    Eigen::Vector3d rotation_vector_rc ;
    q2Euler_zyx(Eigen::Quaterniond(Rrc) , rotation_vector_rc);
    std::cout << std::endl;
    std::cout <<  "before refine: Rrc(YPR) = " << rotation_vector_rc[0] <<  " " <<rotation_vector_rc[1] << " " << rotation_vector_rc[2] <<std::endl;
    std::cout << "before refine: trc =  " << trc[0] << "  " << trc[1] << std::endl;

    Eigen::Matrix4d Trc = Eigen::Matrix4d::Identity();
    Trc.block<3,3>(0,0) = Rrc;
    Trc.block<2,1>(0,3) = trc;
    refineEstimate(Trc, 1.0 ,q_odo,t_odo,q_cam,t_cam);
    Eigen::Vector3d Rrc_zyx;
    q2Euler_zyx(Eigen::Quaterniond(Trc.block<3,3>(0,0)) , Rrc_zyx);
    std::cout << std::endl << "after refine: Rrc(YPR) = " << Rrc_zyx[0] << "  " << Rrc_zyx[1] << "  " << Rrc_zyx[2] << std::endl;
    std::cout << "after refine trc = " << Trc(0,3) << "  " << Trc(1,3) << std::endl;

    //2019.06.22
    internelPara.l[0] = Trc(0,3);
    internelPara.l[1] = Trc(1,3);
    internelPara.l[2] = Rrc_zyx[0];

    double laser_std_x, laser_std_y, laser_std_th;
    cSolver cs;
    cs.estimate_noise(sync_result, internelPara, laser_std_x, laser_std_y, laser_std_th);

    /* Now compute the FIM */
    // 论文公式 9 误差的协方差
//  std::cout <<'\n' << "Noise: " << '\n' << laser_std_x << ' ' << laser_std_y
//            << ' ' << laser_std_th << std::endl;

    Eigen::Matrix3d laser_fim = Eigen::Matrix3d::Zero();
    laser_fim(0,0) = (float)1 / (laser_std_x * laser_std_x);
    laser_fim(1,1) = (float)1 / (laser_std_y * laser_std_y);
    laser_fim(2,2) = (float)1 / (laser_std_th * laser_std_th);

    Eigen::Matrix3d laser_cov = laser_fim.inverse();

    std::cout << '\n' << "-------Errors (std dev)-------" << '\n'
        << "cam-odom x: " << 1000 * sqrt(laser_cov(0,0)) << " mm" << '\n'
        << "cam-odom y: " << 1000 * sqrt(laser_cov(1,1)) << " mm" << '\n'
        << "cam-odom yaw: " << rad2deg(sqrt(laser_cov(2,2))) << " deg" << std::endl;

  // TODO
  // Compute 6*6 FIM
    Eigen::MatrixXd fim = Eigen::MatrixXd::Zero(6,6);
    fim = cs.compute_fim(sync_result,internelPara,laser_fim);
    Eigen::Matrix<double, 6,6> state_cov = fim.inverse();
    std::cout << '\n' << "-------Uncertainty-------" << '\n';
    std::cout << "Uncertainty Left wheel radius : "<< 1000 * sqrt(state_cov(0,0)) <<" mm \n";
    std::cout << "Uncertainty Right wheel radius : "<< 1000 * sqrt(state_cov(1,1)) <<" mm \n";
    std::cout << "Uncertainty Axle between wheels : "<< 1000 * sqrt(state_cov(2,2)) <<" mm \n";
    std::cout << "Uncertainty cam-odom-x : "<< 1000 * sqrt(state_cov(3,3)) <<" mm \n";
    std::cout << "Uncertainty cam-odom-y : "<< 1000 * sqrt(state_cov(4,4)) <<" mm \n";
    std::cout << "Uncertainty cam-odom-yaw : "<< rad2deg( sqrt(state_cov(5,5)) )<<" deg \n";
    std::cout << std::endl;
}

void SolveQyx::q2Euler_zyx(Eigen::Quaterniond q, Eigen::Vector3d &res)
{
  double r11 = 2*(q.x()*q.y() + q.w()*q.z());
  double r12 = q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z();
  double r21 = -2*(q.x()*q.z() - q.w()*q.y());
  double r31 = 2*(q.y()*q.z() + q.w()*q.x());
  double r32 =  q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z();
  res[2] = atan2( r31, r32 );//yaw
  res[1] = asin ( r21 );//pitch
  res[0] = atan2( r11, r12 );//roll
}

//#define LOSSFUNCTION
void SolveQyx::refineEstimate(Eigen::Matrix4d &Trc, double scale,
    const std::vector<Eigen::Quaterniond > &quats_odo,
    const std::vector<Eigen::Vector3d> &tvecs_odo,
    const std::vector<Eigen::Quaterniond> &quats_cam,
    const std::vector<Eigen::Vector3d> &tvecs_cam)
  {
   Eigen::Quaterniond q(Trc.block<3,3>(0,0));
   double q_coeffs[4] = {q.w(),q.x(),q.y(),q.z()};
   double t_coeffs[3] = {Trc(0,3),Trc(1,3),Trc(2,3)};
   ceres::Problem problem;
   for(size_t i = 0; i< quats_odo.size(); ++i)
   {
    ceres::CostFunction * costfunction =
    new ceres::AutoDiffCostFunction<CameraOdomErr, 6,4,3>(
            new CameraOdomErr(quats_odo.at(i) , tvecs_odo.at(i), quats_cam.at(i) , tvecs_cam.at(i) ) );   //  residual : 6 ,  rotation: 4

#ifdef LOSSFUNCTION
      //ceres::LossFunctionWrapper* loss_function(new ceres::HuberLoss(1.0), ceres::TAKE_OWNERSHIP);
    ceres::LossFunction * loss_function = new ceres::HuberLoss(1.0);
    problem.AddResidualBlock(costfunction, loss_function, q_coeffs, t_coeffs);
#else
    problem.AddResidualBlock(costfunction, NULL, q_coeffs, t_coeffs);
#endif

  }
  ceres::LocalParameterization* quaternionParameterization = new ceres::QuaternionParameterization;
  problem.SetParameterization(q_coeffs,quaternionParameterization);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.max_num_iterations = 100;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, & summary);
  q = Eigen::Quaterniond(q_coeffs[0],q_coeffs[1],q_coeffs[2],q_coeffs[3]);

  Trc.block<3,3>(0,0) = q.toRotationMatrix();
  Trc.block<3,1>(0,3) << t_coeffs[0],t_coeffs[1],t_coeffs[2];

}

bool SolveQyx::estimateRyx2(const std::vector<Eigen::Quaterniond> &quats_odo,
  const std::vector<Eigen::Vector3d> &tvecs_odo,
  const std::vector<Eigen::Quaterniond > &quats_cam,
  const std::vector<Eigen::Vector3d > &tvecs_cam,
  Eigen::Matrix3d &R_yx)
{
  size_t motionCnt = quats_odo.size( );

  Eigen::MatrixXd M(motionCnt*4, 4);
  M.setZero();

  for(size_t i = 0; i < quats_odo.size(); ++i)
  {
    const Eigen::Quaterniond& q_odo = quats_odo.at(i);
          //const Eigen::Vector3d& t_odo = tvecs_odo.at(i);
    const Eigen::Quaterniond& q_cam = quats_cam.at(i);
          //const Eigen::Vector3d& t_cam = tvecs_cam.at(i);

    M.block<4,4>(i*4, 0) = QuaternionMultMatLeft(q_odo) - QuaternionMultMatRight(q_cam);
  }

          //TODO:: M^T * M
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU |Eigen::ComputeFullV);

    Eigen::Vector4d v1 = svd.matrixV().block<4,1>(0,2);
    Eigen::Vector4d v2 = svd.matrixV().block<4,1>(0,3);

    double lambda[2];      // solution from  ax^2+ bx + c = 0
    if( !SolveConstraintqyx(v1,v2,lambda[0],lambda[1]))
    {
        std::cout << "# ERROR: Quadratic equation cannot be solved due to negative determinant." << std::endl;
        return false;
    }

   // choose one lambda
    Eigen::Matrix3d R_yxs[2];
    double yaw[2];

    for( int i = 0; i<2;++i)
    {
        double t = lambda[i] * lambda[i] * v1.dot(v1) + 2 * lambda[i] * v1.dot(v2) + v2.dot(v2);

              // solve constraint ||q_yx|| = 1
        double lambda2 = sqrt(1.0/t);
        double lambda1 = lambda[i] * lambda2;

        Eigen::Quaterniond q_yx;
        q_yx.coeffs() = lambda1 * v1 + lambda2 * v2; // x,y,z,w

        R_yxs[i] = q_yx.toRotationMatrix();
        double roll,pitch;
        mat2RPY(R_yxs[i], roll, pitch, yaw[i]);
        std::cout<<"roll: "<<roll<<" pitch: "<<pitch<<" yaw: "<<yaw[i]<<std::endl;
    }

    // q_yx  means yaw is zero. we choose the smaller yaw
    if(fabs(yaw[0]) < fabs(yaw[1]) )
    {
      R_yx = R_yxs[0];
    }else
    {
      R_yx = R_yxs[1];
    }
    return true;
}

/*
 *    constraint for q_yx:
 *                         xy = -zw
 *     this can transfrom to a  equation  ax^2+ bx + c = 0
 */
  bool SolveQyx::SolveConstraintqyx(const Eigen::Vector4d t1, const Eigen::Vector4d t2, double& x1, double& x2)
  {
    double a = t1(0) * t1(1)+ t1(2)*t1(3);
    double b = t1(0) * t2(1)+ t1(1)*t2(0)+t1(2) * t2(3)+ t1(3)*t2(2);
    double c =  t2(0) * t2(1)+ t2(2)*t2(3);

    if ( std::fabs(a) < 1e-10)
    {
      x1 = x2 = -c/b;
      return true;
  }
  double delta2 = b*b - 4.0 * a * c;

  if(delta2 < 0.0) return false;

  double delta = sqrt(delta2);

  x1 = (-b + delta)/(2.0 * a);
  x2 = (-b - delta)/(2.0 * a);

  return true;

}