#include "data_selection.h"

#define PI 3.1415926
typedef std::vector<data_selection::odo_data>  OdomPtr;
typedef std::vector<data_selection::cam_data> CamPtr;

data_selection::data_selection(){}

void data_selection::selectData(OdomPtr &odoDatas, CamPtr &camDatas,std::vector<data_selection::sync_data> &sync_result)
{
    startPosAlign(odoDatas,camDatas);
    //get rid of cam data ( deltaTheta = nan, tlc_length<1e-4,axis(1)<0.96), and align cam and odo data
    camOdoAlign(odoDatas,camDatas,sync_result);  

    //get rid of the odo data whose distance is less than 1e-4 and whose theta sign isn't same
    std::vector<data_selection::sync_data> vec_sync_tmp;
    OdomPtr odo_matches;
    CamPtr cam_matches;
    int size = std::min(camDatas.size(), odoDatas.size());

    int nFlagDiff = 0;
    for (int i = 0; i < size; ++i)
    {
        if(fabs(odoDatas[i].v_left) < 1e-3 || fabs(odoDatas[i].v_right) < 1e-3)
            continue;
        //the sign is opposite
        if((odoDatas[i].v_right - odoDatas[i].v_left) * camDatas[i].deltaTheta < 0.0)// rL == rR
            {nFlagDiff++; continue; }
        odo_matches.push_back(odoDatas[i]);
        cam_matches.push_back(camDatas[i]);
        vec_sync_tmp.push_back(sync_result[i]);
    }
    std::cout << "nFlagDiff = " << nFlagDiff <<std::endl;
    sync_result.swap(vec_sync_tmp);
    camDatas.swap(cam_matches);
    odoDatas.swap(odo_matches);

}
void data_selection::camOdoAlign(OdomPtr &odoDatas, CamPtr &camDatas,
    std::vector<data_selection::sync_data> &sync_result)
{
    int id_odo = 2;
    OdomPtr odoDatas_tmp;
    CamPtr camDatas_tmp;

    for (int i = 3; unsigned(i) < camDatas.size() - 3; ++i)
    {
        if( std::isnan(camDatas[i].deltaTheta) )
            continue;

        double tlc_length = camDatas[i].tlc.norm();
        if(tlc_length < 1e-4)
            continue;       
        if(camDatas[i].axis(1) > -0.96)
            continue;

        data_selection::odo_data   odo_start, odo_end;
        id_odo -= 2;
        double t_interval = fabs(odoDatas[id_odo].time - camDatas[i].start_t);
        while(fabs(odoDatas[++id_odo].time - camDatas[i].start_t) < t_interval)
            t_interval = fabs(odoDatas[id_odo].time - camDatas[i].start_t);
        int odo_start_id = id_odo - 1;
        odo_start = odoDatas[odo_start_id];

        id_odo -= 2;
        t_interval = fabs(odoDatas[id_odo].time - camDatas[i].end_t);
        while(fabs(odoDatas[++id_odo].time - camDatas[i].end_t) < t_interval)
            t_interval = fabs(odoDatas[id_odo].time - camDatas[i].end_t);
        int odo_end_id = id_odo - 1;
        odo_end = odoDatas[odo_end_id];

        if(odo_end_id - odo_start_id == 0)
            continue;

        
        //odo data: insert value
        double v_left_start , v_left_end;
        double v_right_start, v_right_end;
        
        if(camDatas[i].start_t > odo_start.time)
        {
            float alpha = (camDatas[i].start_t - odoDatas[odo_start_id].time)
            / (odoDatas[odo_start_id+1].time - odoDatas[odo_start_id].time);
            alpha = fabs(alpha);
            v_left_start = (1-alpha) * odoDatas[odo_start_id].v_left + alpha * odoDatas[odo_start_id+1].v_left; 
            v_right_start =  (1-alpha) * odoDatas[odo_start_id].v_right + alpha * odoDatas[odo_start_id+1].v_right; 
        }
        else if(camDatas[i].start_t < odo_start.time)
        {
            float alpha = (odoDatas[odo_start_id].time - camDatas[i].start_t)
            / (odoDatas[odo_start_id].time - odoDatas[odo_start_id-1].time);
            alpha = fabs(alpha);
            v_left_start = (1-alpha) * odoDatas[odo_start_id].v_left + alpha * odoDatas[odo_start_id-1].v_left;
            v_right_start =  (1-alpha) * odoDatas[odo_start_id].v_right + alpha * odoDatas[odo_start_id-1].v_right;
        }
        else
        {
            v_left_start = odo_start.v_left;
            v_right_start = odo_start.v_right;
        }
        odoDatas[odo_start_id].time = camDatas[i].start_t;
        odoDatas[odo_start_id].v_left = v_left_start;
        odoDatas[odo_start_id].v_right = v_right_start;

        if(camDatas[i].end_t > odo_end.time)
        {
            float alpha = (camDatas[i].end_t - odoDatas[odo_end_id].time)
            / (odoDatas[odo_end_id+1].time - odoDatas[odo_end_id].time);
            alpha = fabs(alpha);
            v_left_end = (1-alpha) * odoDatas[odo_end_id].v_left + alpha * odoDatas[odo_end_id+1].v_left;
            v_right_end =  (1-alpha) * odoDatas[odo_end_id].v_right + alpha * odoDatas[odo_end_id+1].v_right;
        }
        else if(camDatas[i].end_t < odo_end.time)
        {
            float alpha = (odoDatas[odo_end_id].time - camDatas[i].end_t)
            / (odoDatas[odo_end_id].time - odoDatas[odo_end_id-1].time);
            alpha = fabs(alpha);
            v_left_end = (1-alpha) * odoDatas[odo_end_id].v_left + alpha * odoDatas[odo_end_id-1].v_left;
            v_right_end =  (1-alpha) * odoDatas[odo_end_id].v_right + alpha * odoDatas[odo_end_id-1].v_right;
        }
        else
        {
            v_left_end = odo_end.v_left;
            v_right_end = odo_end.v_right;
        }
        odoDatas[odo_end_id].time = camDatas[i].end_t;
        odoDatas[odo_end_id].v_left = v_left_end;
        odoDatas[odo_end_id].v_right = v_right_end;

        //get the average ang_vel and lin_vel between camDatas[i].start_t and camDatas[i].end_t
        data_selection::odo_data   odo_tmp;//odo_tmp
        odo_tmp.time = odoDatas[odo_start_id].time;//odo_tmp
        data_selection::sync_data sync_tmp;
        if(odo_end_id - odo_start_id > 1)
        {
            double dis_left_sum = 0.0, dis_right_sum = 0.0;
            for(int j = odo_start_id; j < odo_end_id; j++)
            {
                dis_left_sum += (odoDatas[j+1].time - odoDatas[j].time) * (odoDatas[j+1].v_left + odoDatas[j].v_left) / 2.0;
                dis_right_sum += (odoDatas[j+1].time - odoDatas[j].time) * (odoDatas[j+1].v_right + odoDatas[j].v_right) / 2.0;
            }
            double T = camDatas[i].end_t - camDatas[i].start_t;
            odo_tmp.v_left = dis_left_sum / T;
            odo_tmp.v_right = dis_right_sum / T;
            
            sync_tmp.T = T; 
            sync_tmp.velocity_left = dis_left_sum / T;
            sync_tmp.velocity_right = dis_right_sum / T;
            
               // 1: robot      camera    robot      camera
               //         x               z        |            x              x         
                //        y               -x      |            y              z 
                //       z               y         |            z               y              

            sync_tmp.scan_match_results[0] = camDatas[i].tlc[0];
            sync_tmp.scan_match_results[1] = camDatas[i].tlc[2];
            sync_tmp.scan_match_results[2] = camDatas[i].deltaTheta;

            sync_tmp.tcl_cam = -camDatas[i].Rcl * camDatas[i].tlc;
            sync_tmp.qcl_cam = Eigen::Quaterniond(camDatas[i].Rcl);
            sync_tmp.angle = -camDatas[i].deltaTheta; // should be cl
            sync_tmp.axis = camDatas[i].axis;
            sync_tmp.startTime = camDatas[i].start_t;

        }
        else if(odo_end_id - odo_start_id == 1)
        {   
            double T = camDatas[i].end_t - camDatas[i].start_t;
            double ave_v_left = (v_left_start + v_left_end) / 2.0;
            double ave_v_right = (v_right_start + v_right_end) / 2.0;

            odo_tmp.v_left = ave_v_left;
            odo_tmp.v_right = ave_v_right;

            sync_tmp.T = T;
            sync_tmp.velocity_left = ave_v_left;
            sync_tmp.velocity_right = ave_v_right;
            
              // 1: robot      camera    robot      camera
               //         x               z        |            x              x         
                //        y               -x      |            y              z 
                //       z               y         |            z               y

            sync_tmp.scan_match_results[0] = camDatas[i].tlc[0];
            sync_tmp.scan_match_results[1] = camDatas[i].tlc[2];
            //double angle_tmp = (camDatas[i].theta_y > 0.0?1:(-1)) * camDatas[i].deltaTheta;
            sync_tmp.scan_match_results[2] = camDatas[i].deltaTheta;

            sync_tmp.tcl_cam = -camDatas[i].Rcl * camDatas[i].tlc;
            sync_tmp.qcl_cam = Eigen::Quaterniond(camDatas[i].Rcl);
            sync_tmp.angle = -camDatas[i].deltaTheta; // should be cl
            sync_tmp.axis = camDatas[i].axis;
            sync_tmp.startTime = camDatas[i].start_t;
        
        }

        camDatas_tmp.push_back(camDatas[i]);
        odoDatas_tmp.push_back(odo_tmp);//odo_tmp
        sync_result.push_back(sync_tmp);

        id_odo--;
    }
    camDatas.swap(camDatas_tmp);
    odoDatas.swap(odoDatas_tmp);
}


void data_selection::startPosAlign(OdomPtr& odoDatas, CamPtr& camDatas)
{
    double t0_cam = camDatas[0].start_t;
    double t0_odo = odoDatas[0].time;
    if(t0_cam > t0_odo)
    {
        double delta_t = fabs(odoDatas[0].time - t0_cam);

        int odo_start = 0;
        while(fabs(odoDatas[++odo_start].time - t0_cam) < delta_t) delta_t = fabs(odoDatas[odo_start].time - t0_cam);
        int odo_aligned = odo_start - 1;
        for (int i = 0; i < odo_aligned; ++i)
            odoDatas.erase(odoDatas.begin());

        std::cout << std::fixed << "aligned start position1: "<< camDatas[0].start_t << "  " << odoDatas[0].time <<std::endl;
        return;
    }
    else if(t0_odo > t0_cam)
    {
        double delta_t = fabs(camDatas[0].start_t - t0_odo);

        int cam_start = 0;
        while(fabs(camDatas[++cam_start].start_t - t0_odo) < delta_t) delta_t = fabs(camDatas[cam_start].start_t - t0_odo);
        int cam_aligned = cam_start - 1;
        for (int i = 0; i < cam_aligned; ++i)
            camDatas.erase(camDatas.begin());

        std::cout << std::fixed << "aligned start position2 : "<< camDatas[0].start_t << "  " << odoDatas[0].time <<std::endl;
        return;
    }
    else
        return;

}