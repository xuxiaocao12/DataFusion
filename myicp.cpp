/********************************************************************************
** @Copyright(c) $year$ $registered organization$ All Rights Reserved.
** @auth： taify
** @date： 2021/01/12
** @desc： myicp源文件
** @Ver : V1.0.0
*********************************************************************************/

#include "myicp.h"
#include "myicp_helpers.h"

MyICP::MyICP()//构造函数
{

}

MyICP::~MyICP()//析构函数
{

}

void MyICP::setSourceCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	source_cloud = cloud;
}

void MyICP::setTargetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	target_cloud = cloud;
}

void MyICP::setLeafSize(float size)//设置体素滤波网格尺寸
{
	leaf_size = size;
}

void MyICP::setMinError(float error)//设置最小误差
{
	min_error = error;
}

void MyICP::setMaxIters(int iters)//设置最大迭代次数
{
	max_iters = iters;
}

void MyICP::setEpsilon(float eps)//设置配准误差，Epsilon伊普西隆ε
{
	epsilon = eps;
}

void MyICP::downsample()//下采样滤波
{
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);//设置体素网格尺寸
	voxel_grid.setInputCloud(source_cloud);
	source_cloud_downsampled.reset(new pcl::PointCloud<pcl::PointXYZ>);//指针成员函数reset使用：p.reset(q)，令智能指针p中存放指针q，即将p中内置指针换为q
	voxel_grid.filter(*source_cloud_downsampled);
	voxel_grid.setInputCloud(target_cloud);
	target_cloud_downsampled.reset(new pcl::PointCloud<pcl::PointXYZ>);
	voxel_grid.filter(*target_cloud_downsampled);
	std::cout << "原始点云经下采样滤波，点数从 " << source_cloud->size() << " 到 " << source_cloud_downsampled->size() << endl;
	std::cout << "目标点云经下采样滤波，点数从 " << target_cloud->size() << " 到 " << target_cloud_downsampled->size() << endl;
}

void MyICP::registration()
{
	std::cout << "icp registration loading..." << std::endl<<"please wait a moment..." << std::endl;

	Eigen::Matrix3f R_12 = Eigen::Matrix3f::Identity();//Identity Matrix单位矩阵。用单位矩阵对变量进行初始化
	Eigen::Vector3f T_12 = Eigen::Vector3f::Zero();
	Eigen::Matrix4f H_12 = Eigen::Matrix4f::Identity();

	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_mid(new pcl::PointCloud<pcl::PointXYZ>());

	//建立kd树
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(target_cloud_downsampled);//在类的private中定义target_cloud_downsampled属性，故可全局调用

	double error = INT_MAX, score = INT_MAX;//INT_MAX=2^32-1=2147483647，INT_MIN = -2 ^ 32 = -2147483648 常量，表示最大、最小整数

	Eigen::Matrix4f H_final = H_12;
	int iters = 0;

	//开始迭代，直到满足条件
	while (error > min_error && iters < max_iters)//当大于最小误差，小于最大迭代次数
	{
		iters++;
		double last_error = error;

		//计算最邻近点对
		calNearestPointPairs(H_12, source_cloud_downsampled, target_cloud_downsampled, target_cloud_mid, kdtree, error);
		error = error / source_cloud_downsampled->points.size();///////////////add myself/////////////////////////////

		if (last_error - error < epsilon)
			break;

		//计算点云中心坐标
		Eigen::Vector4f source_centroid, target_centroid_mid;
		pcl::compute3DCentroid(*source_cloud_downsampled, source_centroid);
		pcl::compute3DCentroid(*target_cloud_mid, target_centroid_mid);

		//去中心化
		Eigen::MatrixXf souce_cloud_demean, target_cloud_demean;
		pcl::demeanPointCloud(*source_cloud_downsampled, source_centroid, souce_cloud_demean);
		pcl::demeanPointCloud(*target_cloud_mid, target_centroid_mid, target_cloud_demean);

		//计算W=q1*q2^T
		Eigen::Matrix3f W = (souce_cloud_demean*target_cloud_demean.transpose()).topLeftCorner(3, 3);

		//SVD分解得到新的旋转矩阵和平移矩阵
		Eigen::JacobiSVD<Eigen::Matrix3f> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3f U = svd.matrixU();
		Eigen::Matrix3f V = svd.matrixV();

		if (U.determinant()*V.determinant() < 0)
		{
			for (int x = 0; x < 3; ++x)
				V(x, 2) *= -1;
		}

		R_12 = V * U.transpose();
		T_12 = target_centroid_mid.head(3) - R_12 * source_centroid.head(3);
		H_12 << R_12, T_12, 0, 0, 0, 1;
		H_final = H_12 * H_final; //更新变换矩阵

		//std::cout.precision(10);//cout.precison():控制浮点型数据精度,设置精度为10位//add myself/////////////
		std::cout << "迭代次数:" << iters << "  " << "error:" << error << std::endl;
	}
	transformation_matrix << H_final;
}

void MyICP::saveICPCloud(const std::string filename)
{
	icp_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*source_cloud, *icp_cloud, transformation_matrix); //点云变换icp_cloud
	pcl::io::savePCDFileBinary(filename, *icp_cloud);
}

void MyICP::getTransformationMatrix()
{
	std::cout << "transformation_matrix:" << std::endl << transformation_matrix << std::endl;
}

void MyICP::getScore()//对应点间距离平方和的平均值，即MSE
{
	double fitness_score = 0.0;
	pcl::KdTreeFLANN <pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(target_cloud);

#pragma omp parallel for reduction(+:fitness_score) //采用openmmp加速
	for (int i = 0; i < icp_cloud->points.size(); ++i)
	{
		std::vector<int> nn_indices(1);
		std::vector<float> nn_dists(1);
		kdtree.nearestKSearch(icp_cloud->points[i], 1, nn_indices, nn_dists);
		fitness_score += nn_dists[0];//下行改为，开平方
		//fitness_score = fitness_score + sqrt(nn_dists[0]);//开方，应该是标准差
	}
	//std::cout.precision(9);//cout.precison():控制浮点型数据精度,设置精度为10位//add myself////////
	std::cout << "score:" << std::endl << fitness_score / icp_cloud->points.size() << std::endl;
}

void MyICP::visualize()
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("windows_2cloud123"));//将viewer定义为智能共享指针
	//pcl::visualization::PCLVisualizer viewer("registration Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(source_cloud, 0, 255, 0); 	//原始点云绿色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(target_cloud, 255, 0, 0); 	//目标点云红色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(icp_cloud, 0, 0, 255); 	//匹配好的点云蓝色

	viewer->setBackgroundColor(0.3, 0.3, 0.3);
	viewer->addPointCloud(source_cloud, src_h, "source cloud0");
	viewer->addPointCloud(target_cloud, tgt_h, "target cloud0");
	viewer->addPointCloud(icp_cloud, final_h, "result cloud0");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source cloud0");//设置点大小
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud0");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "result cloud0");
	viewer->addCoordinateSystem(0.1);//添加坐标轴
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	/*while (!viewer0->wasStopped())
	{
		viewer0->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}*/
}
