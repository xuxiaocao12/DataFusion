#pragma once
#include <QtWidgets/QMainWindow>
#include "ui_DataFusion.h"
//��֤��������
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


	//�׹�---
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1;
	//boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_in1;// (new pcl::PointCloud<pcl::PointXYZ>);
	//������---
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2;
	//boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_in2;// (new pcl::PointCloud<pcl::PointXYZ>);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	void initialVtkWidget();      //����VTK���ӻ�
	void CoordinateTransform();   //����ת��
	void threePointas();          //������׼��
	void exeComm();               //����һexe������
	void loadCloud(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);//��ȡtxt��ʽ����
	void leastSquaresFit_Plane1();//��С�������ƽ�� //AT*A*X=ATb-->X=(AT*A)��*b//����ⷨ
	void leastSquaresFit_Plane2();
	void leastSquaresFit_Plane3();
	void leastSquaresFit_Plane4();
	void RANSAC_plan();           //�������һ�������ƽ��

	void threeCoordinate();       //���������������
	void whiteLight();            //���ذ׹����
	void KDtreeSerch();           //kd���ڽ�����&��Ȩƽ���ں�
	void KDSerch_leastSquaresFit_Fusioin(); //kd����&��Ȩ��С�����ں�

	Eigen::Matrix3d Rd1;      //��ת����------------
	Eigen::RowVector3d Qr;    //ƽ������ 
	double s = 0;             //�����׼��
	
	

private:
	Ui::DataFusionClass ui;
	double a01, a11, a21 = 0;//���������
	double a02, a12, a22 = 0;
	double a03, a13, a23 = 0;
	double a04, a14, a24 = 0;
	double a01_b, a11_b, a21_b = 0;//�׹������
	double a02_b, a12_b, a22_b = 0;
	double a03_b, a13_b, a23_b = 0;
	double a04_b, a14_b, a24_b = 0;

	Eigen::Matrix<double, 3, 1> X;//z= aox+ a1y+ a2������������⣬�������
	Eigen::Matrix<double, 3, 1> X1;
	Eigen::Matrix<double, 3, 1> X2;
	Eigen::Matrix<double, 3, 1> X_b;//�׹������
	Eigen::Matrix<double, 3, 1> X1_b;
	Eigen::Matrix<double, 3, 1> X2_b;

	void DataFusion::every_K_Points();//ÿ��k�����&˫����ʾ
	void DataFusion::addGaussNoise();//��Ӹ�˹����
	void DataFusion::errorEvaluation();//��Ȩ��С�����������
	void GenerateCircularPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& circleCloud, float R );// ����Բ��ƽ��,cloudΪ���ɵĵ��ƣ�RΪԲ�İ뾶

private slots:
	void shouCloud();	//��ͨ��ʾ����-��ȡ���Ʋ���ʾ
	void MyVisualization(pcl::PointCloud < pcl::PointXYZ>::Ptr cloud_in);//�����ڿ��ӻ�-z����Ⱦ

	void shouCloud_normalized();  //��һ����ʾ
};
