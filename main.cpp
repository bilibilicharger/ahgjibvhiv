/*
 * @Author: Jianguo Zhao
 * @Date: 2023-09-18 16:14:04
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2023-10-21 10:06:58
 * @FilePath: /hapbg/src/main.cpp
 */
#include <iostream>


#include "dsf.h"
#include "maximum_points.h"
#include "tsdsf.h"
#include "tsmp.h"


void dsf(int model, string &bg_in_path, string &bg_dsf_txt,
         string &f_in_pcd, string &f_dsf_pcd, string &f_dsf_txt)
{
    cout << "-----dsf-------" << endl;
    Dsf dsf;
    if (model)
    {
        dsf.make_bg_txt(bg_in_path, bg_dsf_txt);
    }
    dsf.filter(f_in_pcd, f_dsf_pcd, bg_dsf_txt, f_dsf_txt);

    cout << "dsf time: " << dsf.total_time << endl;
    sleep(5);
    cout << "-----dsf-------" << endl;
}



void mp(int model, string &bg_mp_path, string &bg_mp_txt,
        string &f_in_pcd, string &f_mp_pcd, string &f_mp_txt)
{

    cout << "-----mp-------" << endl;

    Maximum_points mp;
    if (model)
    {
        mp.make_bg_txt(bg_mp_path, bg_mp_txt);
    }
    mp.filter(f_in_pcd, f_mp_pcd, bg_mp_txt, f_mp_txt);
    cout << "mp time: " << mp.total_time << endl;
    sleep(5);
    cout << "-----mp-------" << endl;
}



void fundsf()
{
    int number = 0;
    int model = 1;
    //    number = atoi(argv[1]);
    //    model = atoi(argv[2]);
    cout << number << " " << model << endl;
    string bg_dsf_path = "/media/ljq/software/shy/DAIR/single-infrastructure-side-velodyne";

    // 生成背景表
    string bg_dsf_txt = "/media/ljq/software/shy/DAIR/single-infrastructure-side-velodyne/bgmp2.txt";


    // 输入待过滤的帧
    string f_in_pcd = "/media/ljq/software/shy/DAIR/single-infrastructure-side-velodyne";

    // 输出过滤后的帧g
    string f_dsf_pcd = "/media/ljq/software/shy/DAIR/lidar";


    // 输出过滤后帧的点数
    string f_dsf_txt = "/media/ljq/software/shy/DAIR/point_mp2.txt";
    // 添加背景帧

    // string bg_dsf_path = "/media/ljq/software/shy/traffic_data/source2";

    // // 生成背景表
    // string bg_dsf_txt = "/media/ljq/software/shy/traffic_data/dsf/bgdsf2.txt";


    // // 输入待过滤的帧
    // string f_in_pcd = "/media/ljq/software/shy/traffic_data/待过滤/2";

    // // 输出过滤后的帧g
    // string f_dsf_pcd = "/media/ljq/software/shy/traffic_data/dsf/2";


    // // 输出过滤后帧的点数
    // string f_dsf_txt = "/media/ljq/software/shy/traffic_data/dsf/point_dsf2.txt";

    dsf(model, bg_dsf_path, bg_dsf_txt, f_in_pcd, f_dsf_pcd, f_dsf_txt);

}


void funmp() {
    int number = 0;
    int model = 1;

    cout << number << " " << model << endl;
    // 添加背景帧
    string bg_mp_path = "/media/ljq/software/shy/DAIR/single-infrastructure-side-velodyne"; 
    // 生成背景表
    string bg_mp_txt = "/media/ljq/software/shy/DAIR/bgmp2.txt";

    // 输入待过滤的帧
    string f_in_pcd = "/media/ljq/software/shy/DAIR/single-infrastructure-side-velodyne";

    // 输出过滤后的帧g
    string f_mp_pcd = "/media/ljq/software/shy/DAIR/lidar";

    // 输出过滤后帧的点数
    string f_mp_txt = "/media/ljq/software/shy/DAIR/point_mp2.txt";

    mp(model, bg_mp_path, bg_mp_txt, f_in_pcd, f_mp_pcd, f_mp_txt);

}


void funtsdsf() {
    string file_name="mine";
    string num="2";
    TSDSF tddsf;
    File pcd;
    pcd.set("/media/ljq/software/shy/traffic_data/source"+num);
    std::vector<std::string> pcd_path = pcd.get_pcd();
    tddsf.FrameAdd(pcd_path);
    tddsf.MakingBgMatrix(file_name,num);
    
    tddsf.ReadBgMatrix();
    File front_path;
    front_path.set("/media/ljq/software/shy/traffic_data/待过滤/"+num);
    
    std::vector<std::string> front_pcd = front_path.get_pcd();
    tddsf.Filtering(front_pcd);
}

void funtsmp() {
    TSMP tdmp;
    string num="";
    File pcd;
    pcd.set("/media/ljq/software/shy/DAIR/single-infrastructure-side-velodyne"+num);
    std::vector<std::string> pcd_path = pcd.get_pcd();
    tdmp.FrameAdd(pcd_path);
    tdmp.MakingBgMatrix(num);
    
    tdmp.ReadBgMatrix();
    File front_path;
    front_path.set("/media/ljq/software/shy/DAIR/single-infrastructure-side-velodyne"+num);
    
    std::vector<std::string> front_pcd = front_path.get_pcd();
    tdmp.Filtering(front_pcd);
}
int main(int argc, char **argv)
{
    //    if (argc < 3) {
    //        cout << "number: 1, 代表第几个文件夹";
    //        cout << "model: 1， 代表是否做背景表1,做， 0不做" << endl;
    //        exit(-1);
    //    }

    //     int number = 101000;
    //     int model = 1;
    // //    number = atoi(argv[1]);
    // //    model = atoi(argv[2]);
    //     cout << number << " " << model << endl;
    funmp();
    // fundsf();
    // funtsdsf();
    // funtsmp();
    return 0;
}
