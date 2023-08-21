#pragma once
#include <QtWidgets/QMainWindow>
#include "ui_DataFusion.h"
//保证不出警告
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

#include <QFileDialog>
#include <vtkRenderWindow.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>



class DataFusion : public QMainWindow
{
	Q_OBJECT

public:
	DataFusion(QWidget* parent = Q_NULLPTR);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;


	//白光---
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1;
	//boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_in1;// (new pcl::PointCloud<pcl::PointXYZ>);
	//三坐标---
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2;
	//boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_in2;// (new pcl::PointCloud<pcl::PointXYZ>);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	void initialVtkWidget();      //设置VTK可视化
	void CoordinateTransform();   //坐标转换
	void threePointas();          //求三基准点
	void exeComm();               //向另一exe传数据
	void loadCloud(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);//读取txt格式点云
	void leastSquaresFit_Plane1();//最小二乘拟合平面 //AT*A*X=ATb-->X=(AT*A)逆*b//矩阵解法
	void leastSquaresFit_Plane2();
	void leastSquaresFit_Plane3();
	void leastSquaresFit_Plane4();
	void RANSAC_plan();           //随机采样一致性拟合平面

	void threeCoordinate();       //加载三坐标机点云
	void whiteLight();            //加载白光点云
	void KDtreeSerch();           //kd树邻近搜索&加权平均融合
	void KDSerch_leastSquaresFit_Fusioin(); //kd搜索&加权最小二乘融合

	Eigen::Matrix3d Rd1;      //旋转矩阵------------
	Eigen::RowVector3d Qr;    //平移向量 
	double s = 0;             //残余标准差
	
	

private:
	Ui::DataFusionClass ui;
	double a01, a11, a21 = 0;//坐标测量机
	double a02, a12, a22 = 0;
	double a03, a13, a23 = 0;
	double a04, a14, a24 = 0;
	double a01_b, a11_b, a21_b = 0;//白光干涉仪
	double a02_b, a12_b, a22_b = 0;
	double a03_b, a13_b, a23_b = 0;
	double a04_b, a14_b, a24_b = 0;

	Eigen::Matrix<double, 3, 1> X;//z= aox+ a1y+ a2三方程联立求解，矩阵求解
	Eigen::Matrix<double, 3, 1> X1;
	Eigen::Matrix<double, 3, 1> X2;
	Eigen::Matrix<double, 3, 1> X_b;//白光干涉仪
	Eigen::Matrix<double, 3, 1> X1_b;
	Eigen::Matrix<double, 3, 1> X2_b;

	void DataFusion::every_K_Points();//每隔k点采样&双屏显示
	void DataFusion::addGaussNoise();//添加高斯噪声
	void DataFusion::errorEvaluation();//加权最小二乘误差评价
	void GenerateCircularPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& circleCloud, float R );// 构造圆形平面,cloud为生成的点云，R为圆的半径

private slots:
	void shouCloud();	//普通显示点云-读取点云并显示
	void MyVisualization(pcl::PointCloud < pcl::PointXYZ>::Ptr cloud_in);//单窗口可视化-z向渲染

	void shouCloud_normalized();  //归一化显示
};
