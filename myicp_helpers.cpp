/********************************************************************************
** @Copyright(c) $year$ $registered organization$ All Rights Reserved.
** @auth�� taify
** @date�� 2021/01/12
** @desc�� myicp_helpersԴ�ļ�
** @Ver : V1.0.0
*********************************************************************************/

#include "myicp_helpers.h"
//calNearestPointPairs �������ڽ����
void calNearestPointPairs
(   Eigen::Matrix4f H, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, 
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud_mid, 
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree, 
	double &error)
{
	double err = 0.0;
	pcl::transformPointCloud(*source_cloud, *source_cloud, H);//�ñ任����H��ת��source_cloud
	std::vector<int>indexs(source_cloud->size());//����һ����Ϊindexs������

#pragma omp parallel for reduction(+:err) //����openmmp����
	for (int i = 0; i < source_cloud->size(); ++i)
	{
		std::vector<int>index(1);
		std::vector<float>distance(1);
		kdtree->nearestKSearch(source_cloud->points[i], 1, index, distance);//------------
		/*nearestKSearch������������壺��ת�������Ķ�Դ�룩
		int
		nearestKSearch(const PointT &point,     ��ָ������Ĳ�ѯ�㣩
		    int k,                              ��kҪ�������ھ��� ��
			std::vector<int> &k_indices,        ��k_indices���ڵ�Ľ��������
			std::vector<float> &k_sqr_distances) const;     ��k_sqr_distances�����ڵ�ĺϳ�ƽ�����룩*/
		err = err + sqrt(distance[0]);
		indexs[i] = index[0];
	}

	pcl::copyPointCloud(*target_cloud, indexs, *target_cloud_mid);//��indexs.size () == target_cloud.points.size ()����target_cloud_mid=target_cloud
	error = err / source_cloud->size();
}
