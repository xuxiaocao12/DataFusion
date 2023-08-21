/********************************************************************************
** @Copyright(c) $year$ $registered organization$ All Rights Reserved.
** @auth： taify
** @date： 2021/01/12
** @desc： myicp_helpers源文件
** @Ver : V1.0.0
*********************************************************************************/

#include "myicp_helpers.h"
//calNearestPointPairs 计算最邻近点对
void calNearestPointPairs
(   Eigen::Matrix4f H, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, 
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud_mid, 
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree, 
	double &error)
{
	double err = 0.0;
	pcl::transformPointCloud(*source_cloud, *source_cloud, H);//用变换矩阵H来转换source_cloud
	std::vector<int>indexs(source_cloud->size());//定义一个名为indexs的容器

#pragma omp parallel for reduction(+:err) //采用openmmp加速
	for (int i = 0; i < source_cloud->size(); ++i)
	{
		std::vector<int>index(1);
		std::vector<float>distance(1);
		kdtree->nearestKSearch(source_cloud->points[i], 1, index, distance);//------------
		/*nearestKSearch最近点搜索定义：（转到定义阅读源码）
		int
		nearestKSearch(const PointT &point,     （指向给定的查询点）
		    int k,                              （k要搜索的邻居数 ）
			std::vector<int> &k_indices,        （k_indices相邻点的结果索引）
			std::vector<float> &k_sqr_distances) const;     （k_sqr_distances到相邻点的合成平方距离）*/
		err = err + sqrt(distance[0]);
		indexs[i] = index[0];
	}

	pcl::copyPointCloud(*target_cloud, indexs, *target_cloud_mid);//当indexs.size () == target_cloud.points.size ()，则target_cloud_mid=target_cloud
	error = err / source_cloud->size();
}
