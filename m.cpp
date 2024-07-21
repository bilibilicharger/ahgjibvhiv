#include "maximum_points.h"
#include<string>
using namespace std;

std::string Maximum_points::get_name(string name)
{
    size_t lastSlashPos = name.find_last_of('/');
    
    if (lastSlashPos != std::string::npos) {
        // 从最后一个'/'字符的位置加1开始提取文件名
        std::string x = name.substr(lastSlashPos + 1);
        
        // 找到文件名中的第一个'.'字符的位置
        size_t dotPos = x.find_first_of('.');
        if (dotPos != std::string::npos) {
            // 从文件名的开头到第一个'.'字符之前提取文件名
            x = x.substr(0, dotPos);
        }
        // x=addZeros(x);
        std::cout << "提取的文件名为: " << x << std::endl;
        return x;
    } 
    else {
        std::cerr << "未找到文件名" << std::endl;
        return "0";
    }
    
}
void Maximum_points::preGoing()
{
    // 数据集路径
    File ob;
    ob.set(bg_in_path);
    vector<string> filesPath = ob.get_allPcd();
    // for (int i = 0; i < filesPath.size(); ++i)
    // {
    //     cout << filesPath[i] << endl;
    // }
    // cout << "filespath.size: " << filesPath.size() << endl;

    // 多线程帧聚合
    // pcl::PointCloud<pcl::PointXYZ> cloud;
    // readPcdDirectory(filesPath, cloud, 15);

    // 将聚合后的帧进行点统计
    vector<vector<vector<int>>> all_la(ver_size * 2, vector<vector<int>>(hor_size * 2, vector<int>(polar_size, 0)));
    
    for (int i = 0; i < 6000; ++i)
    {
        cout << i << " "<<filesPath[i] << endl;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::io::loadPCDFile(filesPath[i], cloud);
        vector<vector<vector<bool>>> all_flag(ver_size * 2, vector<vector<bool>>(hor_size * 2, vector<bool>(polar_size, false)));
        for (int nIndex = 0; nIndex < cloud.size(); ++nIndex)
        {
            
            int hor_index = cal_hor_degree(cloud, nIndex) / (100 * hor_angle_resolution); // 水平角
            int ver_index = cal_ver_degree(cloud, nIndex) / (100 * ver_angle_resolution); // 高度角
            int polar_index = point_distance(cloud, nIndex) / polar_resolution; // 极轴

            if (hor_index >= hor_size || hor_index <= -hor_size || ver_index <= -ver_size || ver_index >= ver_size || polar_index < 0)
            {
                continue;
            }

            hor_index += hor_size;
            ver_index += ver_size;
            if (hor_index >= 2 * hor_size || ver_index >= 2 * ver_size || hor_index < 0 || ver_index < 0)
            {
                cout << "------------------------2----------------------" << endl;
                cout << "hor_index: " << hor_index << endl;
                cout << "ver_index: " << ver_index << endl;
                cout << "polar_index: " << polar_index << endl;
                continue;
            }
            
            if (polar_index >= polar_size)
            {
                if (!all_flag[ver_index][hor_index][polar_size - 1]) {
                    all_la[ver_index][hor_index][polar_size - 1]++;
                    all_flag[ver_index][hor_index][polar_size - 1] = true;
                }
                
            }
            else
            {
                if (!all_flag[ver_index][hor_index][polar_index]) {
                    all_la[ver_index][hor_index][polar_index]++;
                    all_flag[ver_index][hor_index][polar_index] = true;
                }

            }

        }



    }    



    cout << "----------------------------------------------" << endl;
    std::ofstream fout(bg_txt); // 会自动清空文件


    

    for (int i = 0; i < all_la.size(); ++i)
    {
        for (int j = 0; j < all_la[i].size(); ++j)
        {
            // 取一个间隔的最大点
            int max_value = *max_element(all_la[i][j].begin(), all_la[i][j].end());
            if (max_value > 0)
            {
                int max_index =
                    max_element(all_la[i][j].begin(), all_la[i][j].end()) - all_la[i][j].begin();
                int eps = 3;
                int t1 = max(max_index - eps, 0);
                int t2 = min(max_index + eps, polar_size - 1);
                int flag = 1;

                int res_index = t1;

                bool one_flag = false;
                for (size_t k = t1; k <= t2; k++)
                {
                    if (!one_flag && all_la[i][j][k] > 1 && k <= max_index)
                    {
                        res_index = k;
                        one_flag = true;
                    }

                    max_value += all_la[i][j][k];
                }

                if (max_value >= 100)
                {
                    // if (res_index > 3)
                    // {
                    //     res_index -= 2;
                    // }
                    if (res_index <= 0) {
                        res_index = 0;
                    }
                    fout << res_index << "\t";
                }
                else
                {
                    fout << 0 << "\t";
                }
            }
            else
            {
                fout << 0 << "\t";
            }
            fout << "\n";
        }
    }
    fout.close();

    cout << "----1-----" << endl;
    sleep(3);
    cout << "----2-----" << endl;
}

void Maximum_points::multiGoing()
{
    cout << "----3-----" << endl;
    ifstream file;
    file.open(bg_txt, ios::in);

    if (!file.is_open())
    {
        cerr << "can't find the txt_file";
        exit(0);
    }
    vector<vector<int>> result_sub(ver_size * 2, vector<int>(hor_size * 2, 0));
    for (int i = 0; i < result_sub.size(); ++i)
    {
        for (int j = 0; j < result_sub[i].size(); ++j)
        {
            file >> result_sub[i][j];
        }
    }
    file.close();

    File ob;
    ob.set(f_in_pcd);
    vector<string> pcd_path = ob.get_pcd();
    for (int i = 0; i < pcd_path.size(); ++i)
    {
        cout << pcd_path[i] << endl;
    }

    cout << "pcd_path.size: " << pcd_path.size() << endl;

    ofstream file_writer(f_out_txt, ios_base::out);

    for (int i = 0; i < pcd_path.size(); ++i)
    {
        cout << pcd_path[i] << endl;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        string name = get_name(pcd_path[i]);
        pcl::io::loadPCDFile(pcd_path[i], cloud);
        pcl::PointCloud<pcl::PointXYZ> cloud_out;
        // 将背景中体素中的点的个数进行计算
        auto beg_t = system_clock::now();
        for (int nIndex = 0; nIndex < cloud.size(); ++nIndex)
        {
            int hor_index = cal_hor_degree(cloud, nIndex) / (100 * hor_angle_resolution); // 水平角
            int ver_index = cal_ver_degree(cloud, nIndex) / (100 * ver_angle_resolution); // 高度角
            int polar_index = point_distance(cloud, nIndex) / polar_resolution;           // 极轴

            if (cloud[nIndex].z > 3 || polar_index > polar_size)
            {
                continue;
            }
            if (hor_index >= hor_size || hor_index < -hor_size || ver_index < -ver_size || ver_index >= ver_size || polar_index < 0)
            {
                continue;
            }

            hor_index += hor_size;
            ver_index += ver_size;

            if (result_sub[ver_index][hor_index] == 0 || result_sub[ver_index][hor_index] - polar_index > 2)
            {
                cloud_out.points.push_back(cloud.points[nIndex]);
            }
        }

        auto end_t = system_clock::now(); // 结束时间
        duration<double> diff = end_t - beg_t;
        printf("performTest total time: ");
        cout << diff.count() << endl;
        total_time += diff.count();

        cloud_out.width = cloud_out.size();
        cloud_out.height = 1;

        pcl::PCDWriter writer;
        if (f_out_pcd[f_out_pcd.size() - 1] != '/')
        {
            f_out_pcd += '/';
        }
        string filtered_paths = f_out_pcd +name+ ".pcd";
        writer.write(filtered_paths, cloud_out);

        file_writer << cloud.size() << "\t" << cloud_out.size() << "\n";
    }
    file_writer.close();
}

void Maximum_points::make_bg_txt(string &bg_in_path1, string &bg_txt1)
{
    bg_in_path = bg_in_path1;
    bg_txt = bg_txt1;
    preGoing();
}
void Maximum_points::filter(string &f_in_pcd1, string &f_out_pcd1, string &bg_txt1, string &f_out_txt1)
{
    f_in_pcd = f_in_pcd1;
    f_out_pcd = f_out_pcd1;
    bg_txt = bg_txt1;
    f_out_txt = f_out_txt1;
    multiGoing();
}
