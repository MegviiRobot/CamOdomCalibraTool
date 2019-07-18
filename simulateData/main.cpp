#include "simulateOdoCam.h"

int main(int argc, char** argv)
{
    if(argc != 3)
    {
        std::cerr << "usage: ./simulate output_cam_bag_filename output_odo_bag_filename" << std::endl;
        return 0;
    }

    std::string path[2];
    path[0] = argv[1], path[1] = argv[2];

    std::string config_file = "config.yaml";
    std::vector<double> paras;
    readParaFromFile(config_file, paras);

    std::vector<Eigen::Matrix4d> Twc_odos;
     std::vector<odo_data> odoDatas;
     Eigen::Matrix4d Trc;
     double delta_t_odo;

    generateDatas(paras,  Twc_odos, odoDatas,Trc,delta_t_odo);

    saveDatas(Twc_odos,odoDatas,Trc,delta_t_odo,path );


    return 0;
}