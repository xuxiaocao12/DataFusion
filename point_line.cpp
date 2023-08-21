#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/correspondence_estimation.h>
#include<vector>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;//必须有！！！！！定义pointcloud::Ptr的pointcloud


void duiying_point(pointcloud::Ptr input_cloud, pointcloud::Ptr target_cloud)//对应点连线
{
	//初始化对象
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>core;
	core.setInputSource(input_cloud);
	core.setInputTarget(target_cloud);
	pcl::Correspondences all_correspondences;//全部对应点
	//core.determineCorrespondences(all_correspondences,6);//确定输入点云与目标点云之间的对应关系：
	core.determineReciprocalCorrespondences(all_correspondences);   //确定输入点云与目标点云之间的交互对应关系。
	////////////////////////////////////////////////////////////
	float sum = 0.0, sum_x = 0.0, sum_y = 0.0, sum_z = 0.0, rmse, rmse_x, rmse_y, rmse_z;
	vector<float>point_distance;
	for (size_t j = 0; j < all_correspondences.size(); j++) 
	{
		sum += all_correspondences[j].distance;
		point_distance.push_back(all_correspondences[j].distance);//push_back函数，在vector容器尾部添加all_correspondences[j].distance。即point_distance为对应点的容器	
		//vector<int> vec;
		//vec.push_back(10);在vector容器尾部添加10
		sum_x += pow((target_cloud->points[all_correspondences[j].index_match].x - input_cloud->points[all_correspondences[j].index_query].x), 2);
		sum_y += pow((target_cloud->points[all_correspondences[j].index_match].y - input_cloud->points[all_correspondences[j].index_query].y), 2);
		sum_z += pow((target_cloud->points[all_correspondences[j].index_match].z - input_cloud->points[all_correspondences[j].index_query].z), 2);
	}
	rmse = sqrt(sum / all_correspondences.size());     //均方根误差。//方差，开根号为标准差
	rmse_x = sqrt(sum_x / all_correspondences.size()); //X方向均方根误差
	rmse_y = sqrt(sum_y / all_correspondences.size()); //Y方向均方根误差
	rmse_z = sqrt(sum_z / all_correspondences.size()); //Z方向均方根误差
	vector<float>::iterator max = max_element(point_distance.begin(), point_distance.end());//获取最大距离的对应点//迭代器(iterator为容器vector的迭代器)检查容器内元素并遍历。
	vector<float>::iterator min = min_element(point_distance.begin(), point_distance.end());//获取最小距离的对应点//max_element查询最大值所在位置，min_element查询最小值
	cout << "匹配点对个数" << all_correspondences.size() << endl;
	cout << "距离最大值" << sqrt(*max) * 100 << "厘米" << endl;
	cout << "距离最小值" << sqrt(*min) * 100 << "厘米" << endl;

	cout << "均方根误差" << rmse << "米" << endl;
	cout << "X均方根误差" << rmse_x << "米" << endl;
	cout << "Y均方根误差" << rmse_y << "米" << endl;
	cout << "Z均方根误差" << rmse_z << "米" << endl;
//////////////////////////////////////////////////////////////////////////
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer0(new pcl::visualization::PCLVisualizer("Show"));//将viewer定义为智能共享指针
	//-----------给点云添加颜色-------------------------
	viewer0->setBackgroundColor(0.3, 0.3, 0.3);
	viewer0->addCoordinateSystem(0.1);//添加坐标轴
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>source_color(input_cloud, 255, 0, 0);//源点云绿色
	viewer0->addPointCloud<pcl::PointXYZ>(input_cloud, source_color, "source cloud");//将点云添加到视窗
	viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source cloud");//设置点云再视窗中的显示方式，渲染属性，大小

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(target_cloud, 0, 255, 0);//目标点云红色
	viewer0->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
	viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud");
	
	//对应关系可视化
	viewer0->addCorrespondences<pcl::PointXYZ>(input_cloud, target_cloud, all_correspondences, "correspondence");
	//viewer->initCameraParameters();
	while (!viewer0->wasStopped())
	{
		viewer0->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}	
}

