#include "dsf.h"
// float Dsf::arr[3][2] = {{-60, 60}, {-60, 60}, {-2, 3}};
void Dsf::make_bg_txt(string &bg_in_path1, string &bg_txt1)
{
    bg_in_path = bg_in_path1;
    bg_txt = bg_txt1;
    preGoing();
}
void Dsf::filter(string &f_in_pcd1, string &f_out_pcd1, string &bg_txt1, string &f_out_txt1)
{
    f_in_pcd = f_in_pcd1;
    f_out_pcd = f_out_pcd1;
    bg_txt = bg_txt1;
    f_out_txt = f_out_txt1;
    multiGoing();
}

// 排序并删除vector中的重复元素
template <typename T>
inline void Dsf::deduplication(T &c)
{
    sort(c.begin(), c.end());
    typename T::iterator new_end = unique(c.begin(), c.end()); //"删除"相邻的重复元素
    c.erase(new_end, c.end());                                 // 删除(真正的删除)重复的元素
}
// 需要放在deduplication后面（需用到deduplication函数）
void Dsf::threshold_learning(vector<std::size_t> &all_label, vector<size_t> &voxelx_x,
                             vector<int> &voxelIndex)
{
    // voxelx_x放的是x-x米内占有点云的voxel的索引,数值会有重复的，所以需要删除重复
    deduplication(voxelx_x);
    // v1存放的是该索引voxel内的点云数
    vector<std::size_t> v1;
    for (int i = 0; i < voxelx_x.size(); i++)
    {
        v1.push_back(all_label[voxelx_x[i]]);
    }
    // 对v1进行排序，得到的数值类似 1,1,1,2,2,2,3,3,3,4,4,5,5,6
    sort(v1.begin(), v1.end()); // v1 already sorted
    // tmp对v1进行复制
    vector<std::size_t> tmp = v1;
    // 对tmp排序并删除重复元素，得到类似1,2,3,4,5,6,7,9(并非连续), tmp就为所得图片的横轴坐标, 论文中的nums/cube
    deduplication(tmp);

    // 对v1的重复个数进行统计放到v2中，v2也就是图片中的纵轴，即论文中的frequency
    vector<std::size_t> v2(tmp.size(), 1);
    int j = 0;
    for (int i = 0; i < v1.size(); i++)
    {
        v1[i] == v1[i + 1] ? v2[j] = v2[j] + 1 : j++;
    }

    //    plt::plot(tmp, v2);
    //    plt::show();

    // 论文实现的方法，判断F(i+1)>F(i)，从而得到i的值，也就是代码中的tmpIndex值
    int tmpIndex = 0;
    for (tmpIndex; tmpIndex < v2.size(); tmpIndex++)
    {
        if (v2[tmpIndex] < v2[tmpIndex + 1])
            break;
    }

    for (size_t i = 0; i < voxelx_x.size(); i++)
    {
        // 将背景点的voxel索引push到voxelIndex中
        // 下面这个是阈值
        if (all_label[voxelx_x[i]] > tmp[tmpIndex + 1])
        {
            voxelIndex.push_back(voxelx_x[i]);
        }
    } // 下面这个是阈值
    cout << tmp[tmpIndex + 1] << endl;
}

void Dsf::preGoing()
{
    File ob;
    ob.set(bg_in_path);
    vector<string> filesPath = ob.get_allPcd();
    // for (int i = 0; i < filesPath.size(); ++i)
    // {
    //     cout << filesPath[i] << endl;
    // }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    readPcdDirectory(filesPath, cloud, 15);
    // 带通滤波
    passThrough(cloud);
    cout << cloud.size() << endl;

    // 设置体素格子大小
    x_size = (arr[0][1] - arr[0][0]) / resolution;
    y_size = (arr[1][1] - arr[1][0]) / resolution;
    z_size = (arr[2][1] - arr[2][0]) / resolution;
    size_t voxel_number = x_size * y_size * z_size;
    cout << "voxel_number=" << voxel_number << endl;

    // 为每个点云附体素编号，并在相应体素中记录其点云索引
    vector<size_t> all_label(voxel_number, 0);

    vector<std::size_t> voxel0_5;
    vector<std::size_t> voxel5_10;
    vector<std::size_t> voxel10_20;
    vector<std::size_t> voxel20_30;
    vector<std::size_t> voxel30_40;
    vector<std::size_t> voxel40_50;
    vector<std::size_t> voxel50_60;

    // 将背景中体素中的点的个数进行计算
    for (int nIndex = 0; nIndex < cloud.size(); ++nIndex)
    {
        int x_index = (cloud[nIndex].x - arr[0][0]) / resolution;
        int y_index = (cloud[nIndex].y - arr[1][0]) / resolution;
        int z_index = (cloud[nIndex].z - arr[2][0]) / resolution;

        size_t voxel_index = z_index * (x_size * y_size) + y_index * x_size + x_index;
        all_label[voxel_index]++;

        // if (cloud[nIndex].x > -5 && cloud[nIndex].x < 5 && cloud[nIndex].y > -5 && cloud[nIndex].y < 5)
        //     voxel0_5.push_back(voxel_index);
        // else if (cloud[nIndex].x > -10 && cloud[nIndex].x < 10 && cloud[nIndex].y > -10 && cloud[nIndex].y < 10)
        //     voxel5_10.push_back(voxel_index);
        // else if (cloud[nIndex].x > -20 && cloud[nIndex].x < 20 && cloud[nIndex].y > -20 && cloud[nIndex].y < 20)
        //     voxel10_20.push_back(voxel_index);
        // else if (cloud[nIndex].x > -30 && cloud[nIndex].x < 30 && cloud[nIndex].y > -30 && cloud[nIndex].y < 30)
        //     voxel20_30.push_back(voxel_index);
        // else if (cloud[nIndex].x > -40 && cloud[nIndex].x < 40 && cloud[nIndex].y > -40 && cloud[nIndex].y < 40)
        //     voxel30_40.push_back(voxel_index);
        // else if (cloud[nIndex].x > -50 && cloud[nIndex].x < 50 && cloud[nIndex].y > -50 && cloud[nIndex].y < 50)
        //     voxel40_50.push_back(voxel_index);
        // else
        //     voxel50_60.push_back(voxel_index);
    }

    cout << "ok here" << endl;

    //    // all_label统计显示
    //    map<std::size_t , int> form;
    //    check(all_label, form);
    //    vector<size_t> form_x, form_y;
    //    map<std::size_t , int>::iterator it;
    //    cout << "form.size=" << form.size() << endl;
    //    for (it=form.begin(); it!=form.end() ; ++it) {
    //        form_x.push_back((*it).first);
    //        form_y.push_back((*it).second);
    //    }
    //
    //    plt::plot(form_x, form_y);
    //    plt::show();

    // {
    //     // 存放背景点的cube的索引
    //     vector<int> voxelIndex;
    //     // threshold learning
    //     threshold_learning(all_label, voxel0_5, voxelIndex);
    //     threshold_learning(all_label, voxel5_10, voxelIndex);
    //     threshold_learning(all_label, voxel10_20, voxelIndex);
    //     threshold_learning(all_label, voxel20_30, voxelIndex);
    //     threshold_learning(all_label, voxel30_40, voxelIndex);
    //     threshold_learning(all_label, voxel40_50, voxelIndex);
    //     threshold_learning(all_label, voxel50_60, voxelIndex);

    //     // 若voxel内点云个数大于阈值放入voxelIndex中，排序为后面的二分法铺垫
    //     sort(voxelIndex.begin(), voxelIndex.end());

    //     vector2txt(voxelIndex, bg_txt);
    // }
    vector<int> voxelIndex;
    for (int i = 0; i < all_label.size(); ++i) {
        if (all_label[i] > 400) {
            voxelIndex.push_back(i);
        }
    }
    vector2txt(voxelIndex, bg_txt);

}

void Dsf::multiGoing()
{

    vector<int> voxelIndex;
    txt2vector(bg_txt, voxelIndex);

    File ob;
    ob.set(f_in_pcd);
    vector<string> pcd_path = ob.get_pcd();
    for (int i = 0; i < pcd_path.size(); ++i)
    {
        cout << pcd_path[i] << endl;
    }

    cout << "pcd_path.size：" << pcd_path.size() << endl;

    ofstream file_writer(f_out_txt, ios_base::out);

    // 设置体素格子大小
    x_size = (arr[0][1] - arr[0][0]) / resolution;
    y_size = (arr[1][1] - arr[1][0]) / resolution;
    z_size = (arr[2][1] - arr[2][0]) / resolution;

    for (int i = 0; i < pcd_path.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud, cloud_out;
        pcl::io::loadPCDFile(pcd_path[i], cloud);
        auto beg_t = system_clock::now();
        passThrough(cloud);

        //    int cIndex;
        for (int nIndex = 0; nIndex < cloud.size(); ++nIndex)
        {
            int x_index = (cloud[nIndex].x - arr[0][0]) / resolution;
            int y_index = (cloud[nIndex].y - arr[1][0]) / resolution;
            int z_index = (cloud[nIndex].z - arr[2][0]) / resolution;

            size_t voxel_index = z_index * (x_size * y_size) + y_index * x_size + x_index;
            if (!binary_search(voxelIndex.begin(), voxelIndex.end(), voxel_index))
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

        string filtered_path = f_out_pcd + to_string(i + 1) + ".pcd";
        writer.write(filtered_path, cloud_out);

        file_writer << cloud.size() << "\t" << cloud_out.size() << "\n";
    }
    file_writer.close();
}