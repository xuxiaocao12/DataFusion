/********************************************************************************
** @Copyright(c) $year$ $registered organization$ All Rights Reserved.
** @auth�� taify
** @date�� 2021/01/12
** @desc�� myicpԴ�ļ�
** @Ver : V1.0.0
*********************************************************************************/

#include "myicp.h"
#include "myicp_helpers.h"

MyICP::MyICP()//���캯��
{

}

MyICP::~MyICP()//��������
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

void MyICP::setLeafSize(float size)//���������˲�����ߴ�
{
	leaf_size = size;
}

void MyICP::setMinError(float error)//������С���
{
	min_error = error;
}

void MyICP::setMaxIters(int iters)//��������������
{
	max_iters = iters;
}

void MyICP::setEpsilon(float eps)//������׼��Epsilon������¡��
{
	epsilon = eps;
}

void MyICP::downsample()//�²����˲�
{
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);//������������ߴ�
	voxel_grid.setInputCloud(source_cloud);
	source_cloud_downsampled.reset(new pcl::PointCloud<pcl::PointXYZ>);//ָ���Ա����resetʹ�ã�p.reset(q)��������ָ��p�д��ָ��q������p������ָ�뻻Ϊq
	voxel_grid.filter(*source_cloud_downsampled);
	voxel_grid.setInputCloud(target_cloud);
	target_cloud_downsampled.reset(new pcl::PointCloud<pcl::PointXYZ>);
	voxel_grid.filter(*target_cloud_downsampled);
	std::cout << "ԭʼ���ƾ��²����˲��������� " << source_cloud->size() << " �� " << source_cloud_downsampled->size() << endl;
	std::cout << "Ŀ����ƾ��²����˲��������� " << target_cloud->size() << " �� " << target_cloud_downsampled->size() << endl;
}

void MyICP::registration()
{
	std::cout << "icp registration loading..." << std::endl<<"please wait a moment..." << std::endl;

	Eigen::Matrix3f R_12 = Eigen::Matrix3f::Identity();//Identity Matrix��λ�����õ�λ����Ա������г�ʼ��
	Eigen::Vector3f T_12 = Eigen::Vector3f::Zero();
	Eigen::Matrix4f H_12 = Eigen::Matrix4f::Identity();

	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_mid(new pcl::PointCloud<pcl::PointXYZ>());

	//����kd��
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(target_cloud_downsampled);//�����private�ж���target_cloud_downsampled���ԣ��ʿ�ȫ�ֵ���

	double error = INT_MAX, score = INT_MAX;//INT_MAX=2^32-1=2147483647��INT_MIN = -2 ^ 32 = -2147483648 ��������ʾ�����С����

	Eigen::Matrix4f H_final = H_12;
	int iters = 0;

	//��ʼ������ֱ����������
	while (error > min_error && iters < max_iters)//��������С��С������������
	{
		iters++;
		double last_error = error;

		//�������ڽ����
		calNearestPointPairs(H_12, source_cloud_downsampled, target_cloud_downsampled, target_cloud_mid, kdtree, error);
		error = error / source_cloud_downsampled->points.size();///////////////add myself/////////////////////////////

		if (last_error - error < epsilon)
			break;

		//���������������
		Eigen::Vector4f source_centroid, target_centroid_mid;
		pcl::compute3DCentroid(*source_cloud_downsampled, source_centroid);
		pcl::compute3DCentroid(*target_cloud_mid, target_centroid_mid);

		//ȥ���Ļ�
		Eigen::MatrixXf souce_cloud_demean, target_cloud_demean;
		pcl::demeanPointCloud(*source_cloud_downsampled, source_centroid, souce_cloud_demean);
		pcl::demeanPointCloud(*target_cloud_mid, target_centroid_mid, target_cloud_demean);

		//����W=q1*q2^T
		Eigen::Matrix3f W = (souce_cloud_demean*target_cloud_demean.transpose()).topLeftCorner(3, 3);

		//SVD�ֽ�õ��µ���ת�����ƽ�ƾ���
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
		H_final = H_12 * H_final; //���±任����

		//std::cout.precision(10);//cout.precison():���Ƹ��������ݾ���,���þ���Ϊ10λ//add myself/////////////
		std::cout << "��������:" << iters << "  " << "error:" << error << std::endl;
	}
	transformation_matrix << H_final;
}

void MyICP::saveICPCloud(const std::string filename)
{
	icp_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*source_cloud, *icp_cloud, transformation_matrix); //���Ʊ任icp_cloud
	pcl::io::savePCDFileBinary(filename, *icp_cloud);
}

void MyICP::getTransformationMatrix()
{
	std::cout << "transformation_matrix:" << std::endl << transformation_matrix << std::endl;
}

void MyICP::getScore()//��Ӧ������ƽ���͵�ƽ��ֵ����MSE
{
	double fitness_score = 0.0;
	pcl::KdTreeFLANN <pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(target_cloud);

#pragma omp parallel for reduction(+:fitness_score) //����openmmp����
	for (int i = 0; i < icp_cloud->points.size(); ++i)
	{
		std::vector<int> nn_indices(1);
		std::vector<float> nn_dists(1);
		kdtree.nearestKSearch(icp_cloud->points[i], 1, nn_indices, nn_dists);
		fitness_score += nn_dists[0];//���и�Ϊ����ƽ��
		//fitness_score = fitness_score + sqrt(nn_dists[0]);//������Ӧ���Ǳ�׼��
	}
	//std::cout.precision(9);//cout.precison():���Ƹ��������ݾ���,���þ���Ϊ10λ//add myself////////
	std::cout << "score:" << std::endl << fitness_score / icp_cloud->points.size() << std::endl;
}

void MyICP::visualize()
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("windows_2cloud123"));//��viewer����Ϊ���ܹ���ָ��
	//pcl::visualization::PCLVisualizer viewer("registration Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(source_cloud, 0, 255, 0); 	//ԭʼ������ɫ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(target_cloud, 255, 0, 0); 	//Ŀ����ƺ�ɫ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(icp_cloud, 0, 0, 255); 	//ƥ��õĵ�����ɫ

	viewer->setBackgroundColor(0.3, 0.3, 0.3);
	viewer->addPointCloud(source_cloud, src_h, "source cloud0");
	viewer->addPointCloud(target_cloud, tgt_h, "target cloud0");
	viewer->addPointCloud(icp_cloud, final_h, "result cloud0");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source cloud0");//���õ��С
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud0");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "result cloud0");
	viewer->addCoordinateSystem(0.1);//���������
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
