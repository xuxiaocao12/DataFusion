#include "DataFusion.h"
//#include"test.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ascii_io.h>

#include <pcl/kdtree/kdtree_flann.h> // ks-tree头文件
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>//采样、提取
#include <pcl/filters/random_sample.h>//采取固定数量的点云
#include <pcl/common/random.h>
#include <pcl/common/generate.h> // 生成高斯分布的点云
#include <pcl/common/distances.h>//计算距离
#include <pcl/sample_consensus/sac_model_plane.h> // 点到平面调用函数所在
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>
//#include <pcl/keypoints/uniform_sampling.h> // 均匀采样
#include <vector>
#include <Eigen/Dense>
#include<string>
#include<stdexcept>
#include <pcl/console/parse.h>
#include<qmessagebox.h>
#include <boost/algorithm/string/split.hpp> // for split
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
//#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#pragma comment(lib, "user32.lib")
#pragma comment  (lib,"Gdi32.lib")
#include "windows.h"
#include <qt_windows.h>
#include <QTextCodec>
#include <future>

//test-----------
#include <vtkNew.h>
#include <vtkPointSource.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/angles.h>


using namespace std;
#pragma execution_character_set("utf-8")

struct Point {
    double x, y, z;
};

DataFusion::DataFusion(QWidget* parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
    //QPalette palette; 
    //palette.setColor(QPalette::Window, QColor(25, 35, 45));//设置主窗口背景颜色
    //this->setPalette(palette);

     // 设置按钮样式及悬浮、按下时的状态
    /*ui.comboBox->setStyleSheet("QComboBox{background-color: rgb(225, 225, 225);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
        "QComboBox:hover{background-color:rgb(229, 241, 251); color: black;}"
        "QComboBox:pressed{background-color:rgb(204, 228, 247);border-style: inset;}");*/
    ui.textEdit->setStyleSheet("QTextEdit{background-color: rgb(255, 255, 255);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
        "QTextEdit:hover{background-color:rgb(229, 241, 251); color: black;}"
        "QTextEdit:pressed{background-color:rgb(204, 228, 247);border-style: inset;}"); 

    ui.pushButton_zeroView->setStyleSheet("QPushButton{background-color: rgb(225, 225, 225);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
        "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
        "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}");
    ui.pushButton_plan1->setStyleSheet("QPushButton{background-color: rgb(225, 225, 225);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
        "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
        "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}");
    ui.pushButton_plan2->setStyleSheet("QPushButton{background-color: rgb(225, 225, 225);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
        "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
        "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}");
    ui.pushButton_plan3->setStyleSheet("QPushButton{background-color: rgb(225, 225, 225);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
        "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
        "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}");
    ui.pushButton_plan4->setStyleSheet("QPushButton{background-color: rgb(225, 225, 225);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
        "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
        "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}");
    ui.pushButton_CoordinateTransform->setStyleSheet("QPushButton{background-color: rgb(225, 225, 225);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
        "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
        "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}");

    ui.pushButton->setStyleSheet("QPushButton{background-color: rgb(225, 225, 225);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
        "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
        "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}");
    ui.pushButton_2->setStyleSheet("QPushButton{background-color: rgb(225, 225, 225);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
        "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
        "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}");
    ui.pushButton_3->setStyleSheet("QPushButton{background-color: rgb(225, 225, 225);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
        "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
        "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}");
    ui.pushButton_8->setStyleSheet("QPushButton{background-color: rgb(225, 225, 225);border:2px groove gray;border-radius:10px;padding:2px 4px;border-style: outset;}"
        "QPushButton:hover{background-color:rgb(229, 241, 251); color: black;}"
        "QPushButton:pressed{background-color:rgb(204, 228, 247);border-style: inset;}");
    
   
	initialVtkWidget();
	//connect(ui.pushButton, SIGNAL(triggered()), this, SLOT(shouCloud()));
    //connect(ui->toolButton_31read_fileplot, &QToolButton::clicked, this, [=]() {});
	connect(ui.pushButton_view, &QPushButton::clicked, this, &DataFusion::shouCloud);//显示
    connect(ui.pushButton_zeroView, &QPushButton::clicked, this, &DataFusion::shouCloud_normalized);//归一化显示
    connect(ui.pushButton_CoordinateTransform, &QPushButton::clicked, this, &DataFusion::CoordinateTransform);
    connect(ui.pushButton_plan1, &QPushButton::clicked, this, &DataFusion::leastSquaresFit_Plane1);//最小二乘拟合平面
    connect(ui.pushButton_plan2, &QPushButton::clicked, this, &DataFusion::leastSquaresFit_Plane2);
    connect(ui.pushButton_plan3, &QPushButton::clicked, this, &DataFusion::leastSquaresFit_Plane3);
    connect(ui.pushButton_plan4, &QPushButton::clicked, this, &DataFusion::leastSquaresFit_Plane4);
    connect(ui.pushButton_3points, &QPushButton::clicked, this, &DataFusion::threePointas);
    
    connect(ui.pushButton,   &QPushButton::clicked, this, &DataFusion::whiteLight);         //加载白光数据
    connect(ui.pushButton_2, &QPushButton::clicked, this, &DataFusion::threeCoordinate);    //加载三坐标机数据
    connect(ui.pushButton_3, &QPushButton::clicked, this, &DataFusion::KDtreeSerch);        //kd-树搜索+加权平均融合
    connect(ui.pushButton_8, &QPushButton::clicked, this, &DataFusion::KDSerch_leastSquaresFit_Fusioin); //kd-树搜索+加权最小二乘融合

    connect(ui.pushButton_6, &QPushButton::clicked, this, &DataFusion::errorEvaluation);
    
    //connect(ui.pushButton_6, &QPushButton::clicked, [=] () { 
    //    
    //        //// -------------------------生成位于球面上的点云---------------------------
    //        //vtkNew<vtkPointSource> pointSource;
    //        //pointSource->SetCenter(0.0, 0.0, 0.0);
    //        //pointSource->SetNumberOfPoints(100000);
    //        //pointSource->SetRadius(0.005);
    //        //pointSource->SetDistributionToShell();  // 设置点分布在球面上。
    //        //pointSource->Update();
    //        //// ---------------------------转为PCD点云并保存----------------------------
    //        //vtkSmartPointer<vtkPolyData> polydata = pointSource->GetOutput(); // 获取VTK中的PolyData数据
    //        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //        //pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);
    //        //pcl::PCDWriter w;
    //        //w.writeBinaryCompressed("sphere.pcd", *cloud);
    //        //// -------------------------------结果可视化-------------------------------
    //        //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    //        //viewer->setBackgroundColor(0, 0, 0);
    //        //viewer->setWindowName("生成球形点云");
    //        //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z"); // 按照z字段进行渲染
    //        //viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud");
    //        //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud"); // 设置点云大小
    //        //while (!viewer->wasStopped())
    //        //{
    //        //    viewer->spinOnce(100);
    //        //    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    //        //}
    //}); 

    connect(ui.pushButton_7, &QPushButton::clicked, [=]() {

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            GenerateCircularPlane(cloud,  0.001);
            // ---------------------------转为PCD点云并保存----------------------------  
            pcl::PCDWriter w;
            w.writeBinaryCompressed("圆面.pcd", *cloud);
            // -------------------------------结果可视化-------------------------------
            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
            viewer->setBackgroundColor(0, 0, 0);
            viewer->setWindowName("生成圆面点云");
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z"); // 按照z字段进行渲染
            viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud"); // 设置点云大小

            while (!viewer->wasStopped())
            {
                viewer->spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(100000));
            }
        });

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	/* 
     pcl::PointCloud<pcl::PointXYZ> ::Ptr source1;
	 pcl::PointCloud<pcl::PointXYZ> ::Ptr target1;
	 windows_2cloud( source1,  target1);*/
}

void DataFusion::every_K_Points()//每隔k点采样&双屏显示
{   
    // -------------------------------读取点云数据-------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in100(new pcl::PointCloud<pcl::PointXYZ>);
     //-------------加载点云-------------------
    QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", ".", "Open PCD files(*.txt)");
    if (!fileName.isEmpty()) {
        std::string file_name = fileName.toStdString();
        loadCloud(file_name, cloud_in100);//---------调用读取函数------------   
    }
    //pcl::PCDReader reader;
    //reader.read("banqiu.pcd", *cloud_in100);
    cout << "The points data:  " << cloud_in100->points.size() << endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out00(new pcl::PointCloud<pcl::PointXYZ>);
    // 实现从点云中每every_k_points个点采样一次
    int every_k_points = 10000; // 采样间隔
    // pcl::Indices indices;
    std::vector<int> indices;
    for (size_t i = 0; i < cloud_in100->size(); i += every_k_points)
    {
        indices.push_back(i);
    }
    cout << "banqiu_OUT.size:" << indices.size() << endl;
    pcl::copyPointCloud(*cloud_in100, indices, *cloud_out00);//提取已知索引点
    pcl::io::savePCDFileASCII("yuanmian_OUT.pcd", *cloud_out00);
    // for (int i = 0; i < cloud_in100->points.size(); i += 200)
    // {
    //     pcl::PointXYZ Points0;//创建个点
    //     Points0.x = cloud_in100->points[i].x;
    //     Points0.y = cloud_in100->points[i].y;
    //     Points0.z = cloud_in100->points[i].z;
    //     cloud_out00->points.push_back(Points0);
    // }
    //---------显示点云-----------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer0(new pcl::visualization::PCLVisualizer("显示点云"));
    int v1(0), v2(0);
    viewer0->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer0->setBackgroundColor(0, 0, 0, v1);
    viewer0->addText("point clouds", 10, 10, "v1_text", v1);
    viewer0->createViewPort(0.5, 0.0, 1, 1.0, v2);
    viewer0->setBackgroundColor(0.1, 0.1, 0.1, v2);
    viewer0->addText("filtered point clouds", 10, 10, "v2_text", v2);
          
    viewer0->addPointCloud<pcl::PointXYZ>(cloud_in100, "sample cloud", v1);
    viewer0->addPointCloud<pcl::PointXYZ>(cloud_out00, "cloud_filtered", v2);
    viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
    viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_filtered", v2);
    //viewer->addCoordinateSystem(1.0);
    //viewer->initCameraParameters();
    while (!viewer0->wasStopped())
    {
        viewer0->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    
    /*
        if (pcl::io::loadPCDFile<pcl::PointXYZ>("tree.pcd", *cloud) == -1) {// --------加载点云---------
            PCL_ERROR("读取源标点云失败 \n");return;
        }
        //-----------------------采样固定的点云数量-------------------------
        pcl::RandomSample<pcl::PointXYZ> rs_src;
        rs_src.setInputCloud(cloud);
        rs_src.setSample(3000);// 设置要采样的点云个数
        //rs_src.setSeed(8);   // 设置随机函数的种子点
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rsf(new pcl::PointCloud<pcl::PointXYZ>);
        rs_src.filter(*cloud_rsf);
    */
    //软件窗口显示
    //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud_in100, "z"); // 按照z字段进行渲染
    //viewer->updatePointCloud<pcl::PointXYZ>(cloud_in100, fildColor, "cloud");                          //viewer->updatePointCloud(cloud, "cloud");
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");//设置点云再视窗中的显示方式，渲染属性，大小
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> fusion_color(cloud_out00, 0, 255, 255);//创建一个自定义的颜色处理器PointCloudHandlerCustom对象，设置颜色
    //viewer->addPointCloud<pcl::PointXYZ>(cloud_out00, fusion_color, "fusion_cloud");
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "fusion_cloud");
    //viewer->addCoordinateSystem(0.1);//添加坐标轴
    //viewer->resetCamera();
    //ui.qvtkWidget->update();  
}

void DataFusion::addGaussNoise()//添加高斯噪声
{
    // -------------------------------读取点云数据-------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in100(new pcl::PointCloud<pcl::PointXYZ>);
    //-------------加载点云-------------------
    QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", "./pointsDate/", "Open PCD files(*.txt *.pcd)");
    if (!fileName.isEmpty()) {
        if (fileName.back() == 't') {          //打开txt---
            loadCloud(fileName.toStdString(), cloud_in100);//调用读取函数---   
        }
        else if (fileName.back() == 'd') {    //打开txt---
            pcl::PCLPointCloud2 cloud2;
            Eigen::Vector4f origin;
            Eigen::Quaternionf orientation;
            int pcd_version;
            int data_type;
            unsigned int data_idx;
            int offset = 0;
            pcl::PCDReader rd;
            rd.readHeader(fileName.toStdString(), cloud2, origin, orientation, pcd_version, data_type, data_idx);//(out) data_type数据类型（0=ASCII，1=二进制，2=二进制压缩）
            //pcd文件用记事本打开，正常的是ASCII，乱码的是二进制
            if (data_type == 0)//打开ASCII
            {
                pcl::io::loadPCDFile(fileName.toStdString(), *cloud_in100);
            }
            else if (data_type == 2 || data_type == 1)//打开二进制或二进制压缩!!
            {
                pcl::PCDReader reader;
                reader.read<pcl::PointXYZ>(fileName.toStdString(), *cloud_in100);
            }
        }        
    }
    //pcl::PCDReader reader;
    //reader.read("banqiu.pcd", *cloud_in100);
    pcl::PointCloud<pcl::PointXYZ>::Ptr gauss_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // 设置XYZ各纬度的均值和标准差
    double xmean = 0, ymean = 0, zmean = 0;
    double xstddev = 300 * pow(10, -9), ystddev = 300 * pow(10, -9), zstddev = 300 * pow(10,-9);//pow(x,n)x的n次方
    // ---------------------------生成高斯分布的点云数据---------------------------------------
    pcl::common::CloudGenerator<pcl::PointXYZ, pcl::common::NormalGenerator<double> > generator;
    uint32_t seed = static_cast<uint32_t> (time(NULL));
    pcl::common::NormalGenerator<double>::Parameters x_params(xmean, xstddev, seed++);
    generator.setParametersForX(x_params);
    pcl::common::NormalGenerator<double>::Parameters y_params(ymean, ystddev, seed++);
    generator.setParametersForY(y_params);
    pcl::common::NormalGenerator<double>::Parameters z_params(zmean, zstddev, seed++);
    generator.setParametersForZ(z_params);
    generator.fill((*cloud_in100).width, (*cloud_in100).height, *gauss_cloud);
    // ---------------------------添加高斯分布的随机噪声--------------------------------------
    for (size_t i = 0; i < cloud_in100->points.size(); ++i)
    {
        gauss_cloud->points[i].x += cloud_in100->points[i].x;
        gauss_cloud->points[i].y += cloud_in100->points[i].y;
        gauss_cloud->points[i].z += cloud_in100->points[i].z;
    }
    pcl::io::savePCDFileASCII("banqiu_GaussNoise.pcd", *gauss_cloud);

     //---------显示点云-----------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer0(new pcl::visualization::PCLVisualizer("显示点云"));
    int v1(0), v2(0);
    viewer0->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer0->setBackgroundColor(0, 0, 0, v1);
    viewer0->addText("point clouds", 10, 10, "v1_text", v1);
    viewer0->createViewPort(0.5, 0.0, 1, 1.0, v2);
    viewer0->setBackgroundColor(0.1, 0.1, 0.1, v2);
    viewer0->addText("filtered point clouds", 10, 10, "v2_text", v2);

    viewer0->addPointCloud<pcl::PointXYZ>(cloud_in100, "sample cloud", v1);
    viewer0->addPointCloud<pcl::PointXYZ>(gauss_cloud, "cloud_filtered", v2);
    viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
    viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_filtered", v2);
    //viewer->addCoordinateSystem(1.0);
    //viewer->initCameraParameters();
    while (!viewer0->wasStopped())
    {
        viewer0->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    //软件窗口显示
    //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud_in100, "z"); // 按照z字段进行渲染
    //viewer->updatePointCloud<pcl::PointXYZ>(cloud_in100, fildColor, "cloud");                          //viewer->updatePointCloud(cloud, "cloud");
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");//设置点云再视窗中的显示方式，渲染属性，大小
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> fusion_color(cloud_out00, 0, 255, 255);//创建一个自定义的颜色处理器PointCloudHandlerCustom对象，设置颜色
    //viewer->addPointCloud<pcl::PointXYZ>(cloud_out00, fusion_color, "fusion_cloud");
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "fusion_cloud");
    //viewer->addCoordinateSystem(0.1);//添加坐标轴
    //viewer->resetCamera();
    //ui.qvtkWidget->update();  
}

void DataFusion::errorEvaluation() {    //加权最小二乘误差评价
    //pcl::PointXYZ pmin, pmax;
    //// 获取给定点云集合中的最大距离，并返回最小坐标点和最大坐标点。
    //auto result = pcl::getMaxSegment(*cloud, pmin, pmax);
    //cout << result << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in100(new pcl::PointCloud<pcl::PointXYZ>);
    //-------------加载点云-------------------
    QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", "./pointsDate/", "Open PCD files(*.txt *.pcd)");
    if (!fileName.isEmpty()) {
        if (fileName.back() == 't') {          //打开txt---
            loadCloud(fileName.toStdString(), cloud_in100);//调用读取函数---   
        }
        else if (fileName.back() == 'd') {    //打开txt---
            pcl::PCLPointCloud2 cloud2;
            Eigen::Vector4f origin;
            Eigen::Quaternionf orientation;
            int pcd_version;
            int data_type;
            unsigned int data_idx;
            int offset = 0;
            pcl::PCDReader rd;
            rd.readHeader(fileName.toStdString(), cloud2, origin, orientation, pcd_version, data_type, data_idx);//(out) data_type数据类型（0=ASCII，1=二进制，2=二进制压缩）
            //pcd文件用记事本打开，正常的是ASCII，乱码的是二进制
            if (data_type == 0)//打开ASCII
            {
                pcl::io::loadPCDFile(fileName.toStdString(), *cloud_in100);
            }
            else if (data_type == 2 || data_type == 1)//打开二进制或二进制压缩!!
            {
                pcl::PCDReader reader;
                reader.read<pcl::PointXYZ>(fileName.toStdString(), *cloud_in100);
            }
        }
    }
    //vector<double> Vi;//每点的残差
    //double Radius = 0.005;
    //double sumVi;
    //pcl::PointXYZ center = { 0,0,0 };
    //for (size_t i = 0; i < cloud_in100->points.size(); ++i)
    //{
    //    pcl::PointXYZ p2;
    //    p2.x = cloud_in100->points[i].x;
    //    p2.y = cloud_in100->points[i].y;
    //    p2.z = cloud_in100->points[i].z;
    //    // 计算欧氏距离的平方
    //    //double sqdis = pcl::squaredEuclideanDistance(center, p2); 
    //    // 计算欧氏距离
    //    double dis = pcl::euclideanDistance(center, p2) - Radius;
    //    Vi.push_back(pow(dis, 2));
    //    sumVi += pow(dis,2); 
    //}
    //double sigma = sqrt(sumVi / (cloud_in100->size() - 2));
    //cout << "sigma: " << sigma << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1002(new pcl::PointCloud<pcl::PointXYZ>);
    //-------------加载点云-------------------
    QString fileName2 = QFileDialog::getOpenFileName(this, "Open PointCloud", "./pointsDate/", "Open PCD files(*.txt *.pcd)");
    if (!fileName2.isEmpty()) {
        if (fileName2.back() == 't') {          //打开txt---
            loadCloud(fileName2.toStdString(), cloud_in1002);//调用读取函数---   
        }
        else if (fileName2.back() == 'd') {    //打开txt---
            pcl::PCLPointCloud2 cloud2;
            Eigen::Vector4f origin;
            Eigen::Quaternionf orientation;
            int pcd_version;
            int data_type;
            unsigned int data_idx;
            int offset = 0;
            pcl::PCDReader rd;
            rd.readHeader(fileName2.toStdString(), cloud2, origin, orientation, pcd_version, data_type, data_idx);//(out) data_type数据类型（0=ASCII，1=二进制，2=二进制压缩）
            //pcd文件用记事本打开，正常的是ASCII，乱码的是二进制
            if (data_type == 0)//打开ASCII
            {
                pcl::io::loadPCDFile(fileName2.toStdString(), *cloud_in1002);
            }
            else if (data_type == 2 || data_type == 1)//打开二进制或二进制压缩!!
            {
                pcl::PCDReader reader;
                reader.read<pcl::PointXYZ>(fileName2.toStdString(), *cloud_in1002);
            }
        }
    }
        Eigen::Vector4d centroid;                    // 质心
        Eigen::Matrix3d covariance_matrix;           // 协方差矩阵
        // 计算归一化协方差矩阵和质心
        pcl::computeMeanAndCovarianceMatrix(*cloud_in1002, covariance_matrix, centroid);
        // 计算协方差矩阵的特征值与特征向量
        Eigen::Matrix3d eigenVectors;
        Eigen::Vector3d eigenValues;
        pcl::eigen33(covariance_matrix, eigenVectors, eigenValues);
        // 查找最小特征值的位置
        Eigen::Vector3d::Index minRow, minCol;
        eigenValues.minCoeff(&minRow, &minCol);
        // 获取平面方程：AX+BY+CZ+D = 0的系数
        Eigen::Vector3d normal = eigenVectors.col(minCol);
        double D = -normal.dot(centroid.head<3>());
        cout << "平面模型系数为：\n"
            << "A=" << normal[0] << "\n"
            << "B=" << normal[1] << "\n"
            << "C=" << normal[2] << "\n"
            << "D=" << D << "\n" << endl;

    vector<double> Vi;//每点的残差
    double sumVi;
    for (size_t i = 0; i < cloud_in100->points.size(); ++i) {
            pcl::PointXYZ p2;
            p2.x = cloud_in100->points[i].x;
            p2.y = cloud_in100->points[i].y;
            p2.z = cloud_in100->points[i].z;
            double dis = pcl::pointToPlaneDistance(p2, normal[0], normal[1], normal[2], D);//点到面距离
            Vi.push_back(pow(dis, 2));
            sumVi += pow(dis,2); 
    }
    double sigma = sqrt(sumVi / (cloud_in100->size() - 2));
    cout << "sigma: " << sigma << endl;  
}

// 构造圆形平面,cloud为生成的点云，R为圆的半径
void DataFusion::GenerateCircularPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& circleCloud, float R  )
{
    for (float radius = 0; radius < R; radius += 0.00005)
    {
        for (float r = 0; r < radius; r += 0.00005)
        {
            for (float angle = 0; angle <= 360.0; angle += 0.5)
            {
                pcl::PointXYZ basicPoints;

                basicPoints.x = radius * sinf(pcl::deg2rad(angle)) + 3;
                basicPoints.y = radius * cosf(pcl::deg2rad(angle)) + 3;
                basicPoints.z = -3;
                circleCloud->points.push_back(basicPoints);
            }
        }
    }
}

void DataFusion::initialVtkWidget()
{
    //boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer0(new pcl::visualization::PCLVisualizer("Show_one_cloud"));//将viewer定义为智能共享指针
    //viewer0->setBackgroundColor(0, 0, 0);
    //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z"); // 按照z字段进行渲染
    //viewer0->addPointCloud<pcl::PointXYZ>(source, fildColor, "source cloud");
    //viewer0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source cloud");//设置点云再视窗中的显示方式，渲染属性，大小
    //viewer0->addCoordinateSystem(0.1);//添加坐标轴
    
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);//----------此处必不可少！！否则读取报错！！-----------
    cloud_in1.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_in2.reset(new pcl::PointCloud<pcl::PointXYZ>);

    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    viewer->setBackgroundColor(0.3, 0.3, 0.3);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z"); // 按照z字段进行渲染
    viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "cloud"); //viewer->addPointCloud(cloud, "cloud");
    //viewer->addCoordinateSystem(0.0000001);//添加坐  标轴
    ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());//将渲染输出到插件
    viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());//将插件的交互器传递给PCLVisualizer
    ui.qvtkWidget->update();
}

void DataFusion::CoordinateTransform()//坐标转换
{
    //MatrixXf m = MatrixXf::Random(3, 3);
    //m.row(i);//矩阵第i行
    //m.col(j);//矩阵第j列
    //m.transpose();//转置
    //m.conjugate();//共轭
    //m.adjoint(); //共轭转置
    //m.minCoeff();//所有元素中最小元素
    //m.maxCoeff();//所有元素中最大元素
    //m.trace();//迹，对角元素的和
    //m.sum(); //所有元素求和
    //m.prod(); //所有元素求积
    //m.mean(); //所有元素求平均
    //m.dot(m2); //点乘
    //m.cross(m2);//叉乘
    Eigen::MatrixXd P(3, 3);
    Eigen::MatrixXd Q(3, 3);
    P << X(0,0),   X(1, 0),  X(2, 0), 
         X1(0,0),  X1(0, 0), X1(0, 0),
         X2(0, 0), X2(0, 0), X2(0, 0);
    Q << X_b(0, 0),  X_b(1, 0),  X_b(2, 0),
         X1_b(0, 0), X1_b(0, 0), X1_b(0, 0),
         X2_b(0, 0), X2_b(0, 0), X2_b(0, 0);

   /* P << 0.0007,   14.4434, 1.0137,
         -12.4965, -7.2159, 1.0143,
         12.5023,  -7.2172, 1.0142;   
    Q << 12.6758, 21.2550, 3.4897,
         0.3638,  -0.4422, 1.8681,
         24.9864, 0.3144, -2.4070;*/
    //Eigen::Vector3f up = Eigen::Vector3f::Zero();
    Eigen::RowVector3d up;
    Eigen::RowVector3d ux;
    up << (P(0, 0) + P(1, 0) + P(2, 0)) / 3, (P(0, 1) + P(1, 1) + P(2, 1)) / 3, (P(0, 2) + P(1, 2) + P(2, 2)) / 3;
    ux << (Q(0, 0) + Q(1, 0) + Q(2, 0)) / 3, (Q(0, 1) + Q(1, 1) + Q(2, 1)) / 3, (Q(0, 2) + Q(1, 2) + Q(2, 2)) / 3;   
   cout << "up,ux" << up << endl << ux << endl << endl;
    //去中心化,平均值为0，对标准差无要求
   Eigen::MatrixXd P1(3, 3);
   Eigen::MatrixXd Q1(3, 3);
   //访问矩阵中的元素
   for (int i = 0; i < 3; i++)
   {
       for (int j = 0; j < 3; j++)
       {
           P1(i, j) = P(i, j) - up(j);
           Q1(i, j) = Q(i, j) - ux(j);
       }
   }       
   cout << endl << "PQ" << P1 << endl << endl << Q1 << endl; 
   Eigen::MatrixXd sigma(3, 3); //协方差
   Eigen::MatrixXd sigma_mi(3, 3);
   Eigen::MatrixXd M(3, 3);
   Eigen::MatrixXd Rq(4, 4);
   Eigen::MatrixXd E = Eigen::MatrixXd::Identity(3, 3);      // E.setIdentity(3, 3);
   sigma = P1.transpose() * Q1 / 3;
   sigma_mi = sigma - sigma.transpose();//Aij
   M = sigma + sigma.transpose() -sigma.trace()* E;
   // 由协方差构造4 * 4对称矩阵  
    Rq << sigma.trace() ,sigma_mi(1, 2) ,sigma_mi(2, 0) ,sigma_mi(0, 1),
          sigma_mi(1, 2), M(0, 0),M(0, 1), M(0, 2),
          sigma_mi(2, 0), M(1, 0),M(1, 1), M(1, 2),
          sigma_mi(0, 1), M(2, 0),M(2, 1), M(2, 2);
    cout << endl << "Rq" << Rq << endl;

    //Eigen::MatrixXd mRq = Rq.transpose() * Rq;//构成中心对其的协方差矩阵
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(Rq);//eigen_solver(mRq)
    Eigen::VectorXd eigenvalues = eigen_solver.eigenvalues();  //特征值
    Eigen::MatrixXd eigenvectors = eigen_solver.eigenvectors();//特特征向量
    Eigen::VectorXd q = eigenvectors.col(3);// 因为特征值一般按从小到大排列,所以col(0)就是最小特征值对应的特征向量
    //cout << endl << "特征向量" << eigenvectors.col(0) << endl << endl << eigenvectors.col(1) << endl << endl << eigenvectors.col(2) << endl << endl << q << endl;
    cout << endl << "特征向量" <<  endl << q << endl;
    
    Eigen::Quaterniond q_odom_curr_tmp;//声明一个Eigen类的四元数:
    q_odom_curr_tmp.w() = q(0);
    q_odom_curr_tmp.x() = q(1);
    q_odom_curr_tmp.y() = q(2);
    q_odom_curr_tmp.z() = q(3);  
    //cout << endl << q_odom_curr_tmp.x() << endl << q_odom_curr_tmp.y() << endl << q_odom_curr_tmp.z() << endl << q_odom_curr_tmp.w() << endl;
    Eigen::Matrix3d Rd;    //旋转矩阵
    Eigen::Matrix3d V;     //残差矩阵
    //Eigen::RowVector3d Qr; //平移向量 
    //Rd = q_odom_curr_tmp.matrix(); 
    Rd = q_odom_curr_tmp.normalized().toRotationMatrix();//四元数转为旋转矩阵//先归一化再转为旋转矩阵      
    //Eigen::Matrix3d Rd1;      //------------旋转矩阵------------
    Rd1 = Rd.transpose();
    cout << endl << "Rd" << endl<< Rd << endl << endl<< "Rd.transpose()" << endl << Rd.transpose() << endl;
    cout << endl << "Rd1旋转矩阵" << endl << Rd1 << endl;

    Qr = ux - up*Rd1;        //------------平移矩阵------------//计算时不支持混合浮点float和double!!!//报错!切忌!!!!Matrix3d-Matrix3f 
    V = (P * Rd1 + Qr)- Q;
    
    for (int i = 0; i < V.rows(); i++)
    {
        for (int j = 0; j < V.cols(); j++)
        {
            s += V(i, j) * V(i, j);
        }
    }
    s = sqrt(s / V.size());  //------------残差标准差------------  
    //cout << endl << "daxiao" << V.size() << endl<<V.rows() << endl << V.cols() << endl << endl;
    cout << endl << endl << "平移向量"  << endl << Qr << endl << endl << "残差矩阵" << V << endl <<endl<<"残差标准差"<<s<<endl;  

    Eigen::Quaterniond q0 = Eigen::Quaterniond(Rd1);//矩阵转四元数
    q0.normalize();   
    cout << "矩阵转四元数:"<<endl<<"x = " << q0.x() << endl;cout << "y = " << q0.y() << endl;cout << "z = " << q0.z() << endl;cout << "w = " << q0.w() << endl << endl;

    exeComm();//send message
}

void DataFusion::threePointas()
{
    /*坐标测量机****************************
    点A*/
    Eigen::Matrix3d B;
    Eigen::Vector3d L;
    //Eigen::Matrix<double, 3, 1> X;//z= aox+ a1y+ a2三方程联立求解，矩阵求解
    B << a01, a11, -1,
         a02, a12, -1,
         a04, a14, -1;
    L << -a21, -a22, -a24;
      X = ((B.transpose() * B)).lu().solve(B.transpose() * L);    //lu分解(矩阵A分解为 L,U 下上三角矩阵)求逆，对于大型矩阵加速。
    //X = ((B.transpose() * B)).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B.transpose() * L);  //SVD结果稳定特别是对于矩阵接近病态，但速度慢。
    //X = (B.transpose() * B).inverse() * B.transpose() * L;    //直接解法      
    std::cout << "点A\n" << showpos << X << std::endl;

    /*点B*/
    Eigen::Matrix3d B1;
    Eigen::Vector3d L1;
    //Eigen::Matrix<double, 3, 1> X1;//z= aox+ a1y+ a2三方程联立求解，矩阵求解
    B1 << a02, a12, -1,
         a03, a13, -1,
         a04, a14, -1;
    L1 << -a22 ,- a23 ,- a24;
    X1 = ((B1.transpose() * B1)).lu().solve(B1.transpose() * L1);    
    std::cout << "点B\n" << showpos << X1 << std::endl;

    /*点c*/
    Eigen::Matrix3d B2;
    Eigen::Vector3d L2;
    //Eigen::Matrix<double, 3, 1> X2;//z= aox+ a1y+ a2三方程联立求解，矩阵求解
    B2 << a01, a11, -1,
          a03, a13, -1,
          a04, a14, -1;
    L2 << -a21, -a23, -a24;
    X2 = ((B2.transpose() * B2)).lu().solve(B2.transpose() * L2);     
    std::cout << "点c\n" << showpos << X2 << std::endl;

    /*白光干涉仪****************************
    点A*/
    Eigen::Matrix3d B_b;
    Eigen::Vector3d L_b;
    //Eigen::Matrix<double, 3, 1> X_b;//z= aox+ a1y+ a2三方程联立求解，矩阵求解
    B_b << a01_b, a11_b, -1,
           a02_b, a12_b, -1,
           a04_b, a14_b, -1;
    L_b << -a21_b, -a22_b, -a24_b;
    X_b = ((B_b.transpose() * B_b)).lu().solve(B_b.transpose() * L_b);    
    std::cout << "点A\n" << showpos << X_b << std::endl;

    /*点B*/
    Eigen::Matrix3d B1_b;
    Eigen::Vector3d L1_b;
   // Eigen::Matrix<double, 3, 1> X1_b;//z= aox+ a1y+ a2三方程联立求解，矩阵求解
    B1_b << a02_b, a12_b, -1,
            a03_b, a13_b, -1,
            a04_b, a14_b, -1;
    L1_b << -a22_b, -a23_b, -a24_b;
    X1_b = ((B1_b.transpose() * B1_b)).lu().solve(B1_b.transpose() * L1_b);   
    std::cout << "点B\n" << showpos << X1_b << std::endl;

    /*点c*/
    Eigen::Matrix3d B2_b;
    Eigen::Vector3d L2_b;
    //Eigen::Matrix<double, 3, 1> X2_b;//z= aox+ a1y+ a2三方程联立求解，矩阵求解
    B2_b << a01_b, a11_b, -1,
        a03_b, a13_b, -1,
        a04_b, a14_b, -1;
    L2_b << -a21_b, -a23_b, -a24_b;
    X2_b = ((B2_b.transpose() * B2_b)).lu().solve(B2_b.transpose() * L2_b);   
    std::cout << "点c\n" << showpos << X2_b << std::endl;

    QString str1= QString::number(X (0, 0)) + " " + QString::number(X (1, 0)) + " " + QString::number(X (2, 0))  + "\r\n";
    QString str2 = QString::number(X1(0, 0)) + " " + QString::number(X1(1, 0)) + " " + QString::number(X1(2, 0)) + "\r\n";
    QString str3 = QString::number(X2(0, 0)) + " " + QString::number(X2(1, 0)) + " " + QString::number(X2(2, 0)) + "\r\n";

    QString str1_b = QString::number(X_b(0, 0)) + " " + QString::number(X_b(1, 0)) + " " + QString::number(X_b(2, 0)) + "\r\n";
    QString str2_b = QString::number(X1_b(0, 0)) + " " + QString::number(X1_b(1, 0)) + " " + QString::number(X1_b(2, 0)) + "\r\n";
    QString str3_b = QString::number(X2_b(0, 0)) + " " + QString::number(X2_b(1, 0)) + " " + QString::number(X2_b(2, 0)) + "\r\n";

    QString str = "坐标测量机;\r\n" + str1 + str2 + str3;
    QString str_b = "白光干涉仪;\r\n" + str1_b + str2_b + str3_b;
    ui.textEdit->insertPlainText(str + str_b);
}//三基准点

void DataFusion::exeComm()////大项目软件交互通信
{
    double arr[13] = { Rd1(0,0), Rd1(0,1), Rd1(0,2),
                         Rd1(1,0), Rd1(1,1), Rd1(1,2),
                         Rd1(2,0), Rd1(2,1), Rd1(2,2),
                         Qr(0),    Qr(1),    Qr(2),   s };
                         /*double arr[13] = { 0,1, 2,
                                             3,41, 52,
                                             60,71, 82,
                                             90,10, 11,   12 };*/
    //double* arr = new double[13];
    //arr[0] = 100; arr[1] = 100; arr[2] = 100; arr[3] = 100; arr[4] = 100;
    const ULONG_PTR CUSTOM_TYPE_SEND_WEBVIEWER = 10007;
    HWND hwnd = NULL;
    QString qstrReceiveName = QString::fromLocal8Bit("QtGuiApplication1");//调用window.h种的api来寻找标题为“ReceiveMessage”的窗口
    LPWSTR path = (LPWSTR)qstrReceiveName.utf16();
    hwnd = ::FindWindowW(NULL, path);
    //判断找到的句柄是否为窗口，如果是就通过window的消息COPYDATASTRUCT结构体来实现信息与数据的封装
    if (::IsWindow(hwnd))
    {
        COPYDATASTRUCT  copydata;
        //memset(&copydata, 0, sizeof(COPYDATASTRUCT));
        copydata.dwData = CUSTOM_TYPE_SEND_WEBVIEWER;  // 用户定义数据
        copydata.lpData = arr;  // 指向数据的指针
        copydata.cbData = sizeof(arr);    //数据大小
       //winApi发送信息（通过操作系统的消息传递机制）
        ::SendMessage(hwnd, WM_COPYDATA, reinterpret_cast<WPARAM>((HWND)(this->winId())), reinterpret_cast<LPARAM>(&copydata));
        //ProMessageHandleC::sendMessageByCameraInfo((HWND)(this->winId()), c_strTitle, pszData, sizeof(stuDeviceInfo));
    }
}////大项目软件交互通信

void DataFusion::loadCloud(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud )//txt转pcd
{
    std::ifstream fs;
    fs.open(filename.c_str(), std::ios::binary);
    if (!fs.is_open() || fs.fail())
    {
        PCL_ERROR("Could not open file '%s'! Error : %s\n", filename.c_str(), strerror(errno));
        fs.close();
        return ;
    }

    std::string line;
    std::vector<std::string> st;

    while (!fs.eof())
    {
        std::getline(fs, line);
        // Ignore empty lines
        if (line.empty())
            continue;

        // Tokenize the line
        boost::trim(line);
        boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);

        if (st.size() != 3)
            continue;

        cloud->push_back(pcl::PointXYZ(float(atof(st[0].c_str())), float(atof(st[1].c_str())), float(atof(st[2].c_str()))));
    }
    fs.close();
    //设置点云属性
    cloud->width = cloud->size(); cloud->height = 1; cloud->is_dense = true;
    
    //// Load the first file
    //pcl::PointCloud<pcl::PointXYZ> cloud;
    //if (!loadCloud("bunny.txt", cloud))
    //    return (-1);

    //// Convert to PCD and save
    //pcl::PCDWriter w;
    //w.writeBinaryCompressed("txt2pcd.pcd", cloud);
}

void DataFusion::leastSquaresFit_Plane1() //最小二乘平面拟合 //AT*A*X=ATb-->X=(AT*A)逆*b//矩阵解法
{
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
        //-------------加载点云-------------------
        QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", "./pointsDate/", "Open PCD files(*.txt *.pcd)");
        if (!fileName.isEmpty()) {
            if (fileName.back() == 't') {          //打开txt---
                loadCloud(fileName.toStdString(), cloud_in);//调用读取函数---   
            }
            else if (fileName.back() == 'd') {    //打开txt---
                pcl::PCLPointCloud2 cloud2;
                Eigen::Vector4f origin;
                Eigen::Quaternionf orientation;
                int pcd_version;
                int data_type;
                unsigned int data_idx;
                int offset = 0;
                pcl::PCDReader rd;
                rd.readHeader(fileName.toStdString(), cloud2, origin, orientation, pcd_version, data_type, data_idx);//(out) data_type数据类型（0=ASCII，1=二进制，2=二进制压缩）
                //pcd文件用记事本打开，正常的是ASCII，乱码的是二进制
                if (data_type == 0)//打开ASCII
                {
                    pcl::io::loadPCDFile(fileName.toStdString(), *cloud_in);
                }
                else if (data_type == 2 || data_type == 1)//打开二进制或二进制压缩!!
                {
                    pcl::PCDReader reader;
                    reader.read<pcl::PointXYZ>(fileName.toStdString(), *cloud_in);
                }
            }
        }
       
        //去除nan点
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);

        //pcl::VoxelGrid<pcl::PointXYZ> sor;//提速滤波下采样，加快拟合速度，可选（需包含头文件）
        //sor.setInputCloud(cloud_in);
        //sor.setLeafSize(0.01f, 0.01f, 0.01f);
        //sor.filter(*cloud_in);
        
        //最小二乘平面拟合
        std::size_t cloud_size = cloud_in->points.size();
        std::cout << "方程个数：" << cloud_size << std::endl;
        try//异常处理，关键字try开始，catch子句结束。try语句块中代码异常通常被catch子句处理。
        {
            if (cloud_size < 3)//小于三个点无法计算
            {
                throw std::runtime_error("点云数据出错，请检查数据");
                QMessageBox msgBox;
                msgBox.setText("Error in point cloud data");
                msgBox.exec();
            }
        }
        catch (std::runtime_error err)
        {
            std::cerr << err.what() << std::endl;
            return ;
        }
        Eigen::MatrixXd B;
        B.resize(cloud_size, 3);
        Eigen::MatrixXd L;
        L.resize(cloud_size, 1);

        for (size_t row = 0; row < cloud_size; ++row)
        {
            B(row, 0) = cloud_in->points[row].x;
            B(row, 1) = cloud_in->points[row].y;
            B(row, 2) = 1;
            L(row, 0) = cloud_in->points[row].z;
        }
        Eigen::Matrix<double, 3, 1> aa_mat;//z= aox+ a1y+ a2
        //AT*A*X=ATb-->X=(AT*A)逆*b//矩阵解法
        aa_mat = ((B.transpose() * B)).lu().solve(B.transpose() * L); //lu分解(矩阵A分解为 L,U 下上三角矩阵)求逆，对于大型矩阵加速。
        //aa_mat = ((B.transpose() * B)).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B.transpose() * L);//SVD结果稳定特别是对于矩阵接近病态，但速度慢。
        //aa_mat = (B.transpose() * B).inverse() * B.transpose() * L;//直接解法      
        std::cout << "点云平面拟合结果\n" << showpos <<aa_mat << std::endl;
              
        ////保持编辑器在光标最后一行
        //QTextCursor cursor = ui.textEdit->textCursor();
        //cursor.movePosition(QTextCursor::End);
        //ui.textEdit->setTextCursor(cursor);       
        QString a0,a1,a2;
        a0 = QString::number(aa_mat(0, 0));      
        a1 = QString::number(aa_mat(1, 0));
        a2 = QString::number(aa_mat(2 ,0));
        if (aa_mat(1, 0) > 0)
            a1 = "+" + a1;
        if (aa_mat(2, 0) > 0)
            a2 = "+" + a2;
        int comboBox_currentIndex = ui.comboBox->currentIndex();
        QString str0;
        if (comboBox_currentIndex == 0) {
             str0 = QString::fromUtf8("坐标测量机-顶面;\r\n");
             a01 = aa_mat(0, 0);
             a11 = aa_mat(1, 0);
             a21 = aa_mat(2, 0);
        }
        else if (comboBox_currentIndex == 1) {
             str0 = QString::fromUtf8("白光干涉仪-顶面;\r\n");
             a01_b = aa_mat(0, 0);
             a11_b = aa_mat(1, 0);
             a21_b = aa_mat(2, 0);
        }       
        QString str = str0+"Z = " + a0 + "X " + a1 + "Y " + a2;
        ui.textEdit->insertPlainText(str += '\n');

        //string decomposition;
        //Eigen::Matrix<double, 5, 3> b_mat;//系数矩阵
        //b_mat(0, 0) = -1;
        //Eigen::Matrix<double, 5, 1> l_mat;//列向量        
        //Eigen::Matrix<double, 3, 1> a_mat;
        //if (decomposition == "lu")
        //{          
        //    a_mat = ((b_mat.transpose() * b_mat)).lu().solve(b_mat.transpose() * l_mat);//lu分解(矩阵A分解为 L,U 下上三角矩阵)求逆，对于大型矩阵加速。
        //}
        //else if (decomposition == "svd")
        //{            
        //    a_mat = ((b_mat.transpose() * b_mat)).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_mat.transpose() * l_mat);//SVD结果稳定特别是对于矩阵接近病态，但速度慢。
        //}
        //else
        //{            
        //    a_mat = (b_mat.transpose() * b_mat).inverse() * b_mat.transpose() * l_mat;//直接解法
        //}
        //std::cout << decomposition << "平面方程系数：a0,a1,a2:\n" << a_mat << std::endl;

}

void DataFusion::leastSquaresFit_Plane2()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", ".", "Open PCD files(*.txt)");
    if (!fileName.isEmpty()) {
        std::string file_name = fileName.toStdString();
        loadCloud(file_name, cloud_in);//---------调用读取函数------------
    //    pcl::PCLPointCloud2 cloud2;
    //    Eigen::Vector4f origin;
    //    Eigen::Quaternionf orientation;
    //    int pcd_version;
    //    int data_type;
    //    unsigned int data_idx;
    //    int offset = 0;
    //    pcl::PCDReader rd;
    //    rd.readHeader(file_name, cloud2, origin, orientation, pcd_version, data_type, data_idx);//(out) data_type数据类型（0=ASCII，1=二进制，2=二进制压缩）
    //    //pcd文件用记事本打开，正常的是ASCII，乱码的是二进制
    //    if (data_type == 0)//打开ASCII
    //    {
    //        pcl::io::loadPCDFile(fileName.toStdString(), *cloud_in);
    //    }
    //    else if (data_type == 2 || data_type == 1)//打开二进制或二进制压缩!!
    //    {
    //        pcl::PCDReader reader;
    //        reader.read<pcl::PointXYZ>(fileName.toStdString(), *cloud_in);
    //    }
    }

    //去除nan点
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);

    //pcl::VoxelGrid<pcl::PointXYZ> sor;//提速滤波下采样，加快拟合速度，可选（需包含头文件）
    //sor.setInputCloud(cloud_in);
    //sor.setLeafSize(0.01f, 0.01f, 0.01f);
    //sor.filter(*cloud_in);

    //最小二乘平面拟合
    std::size_t cloud_size = cloud_in->points.size();
    std::cout << "方程个数：" << cloud_size << std::endl;
    try//异常处理，关键字try开始，catch子句结束。try语句块中代码异常通常被catch子句处理。
    {
        if (cloud_size < 3)//小于三个点无法计算
        {
            throw std::runtime_error("点云数据出错，请检查数据");
            QMessageBox msgBox;
            msgBox.setText("Error in point cloud data");
            msgBox.exec();
        }
    }
    catch (std::runtime_error err)
    {
        std::cerr << err.what() << std::endl;
        return;
    }
    Eigen::MatrixXd B;
    B.resize(cloud_size, 3);
    Eigen::MatrixXd L;
    L.resize(cloud_size, 1);

    for (size_t row = 0; row < cloud_size; ++row)
    {
        B(row, 0) = cloud_in->points[row].x;
        B(row, 1) = cloud_in->points[row].y;
        B(row, 2) = 1;
        L(row, 0) = cloud_in->points[row].z;
    }
    Eigen::Matrix<double, 3, 1> aa_mat;//z= aox+ a1y+ a2
    aa_mat = ((B.transpose() * B)).lu().solve(B.transpose() * L); //lu分解(矩阵A分解为 L,U 下上三角矩阵)求逆，对于大型矩阵加速。
    //aa_mat = ((B.transpose() * B)).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B.transpose() * L);//SVD结果稳定特别是对于矩阵接近病态，但速度慢。
    //aa_mat = (B.transpose() * B).inverse() * B.transpose() * L;//直接解法      
    std::cout << "点云平面拟合结果\n" << showpos << aa_mat << std::endl;
    
    QString a0, a1, a2;
    a0 = QString::number(aa_mat(0, 0));
    a1 = QString::number(aa_mat(1, 0));
    a2 = QString::number(aa_mat(2, 0));
    if (aa_mat(1, 0) > 0)
        a1 = "+" + a1;
    if (aa_mat(2, 0) > 0)
        a2 = "+" + a2;
    int comboBox_currentIndex = ui.comboBox->currentIndex();
    QString str0;
    if (comboBox_currentIndex == 0) {
        str0 = QString::fromUtf8("坐标测量机-侧面1;\r\n");
        a02 = aa_mat(0, 0);
        a12 = aa_mat(1, 0);
        a22 = aa_mat(2, 0);
    }
    else if (comboBox_currentIndex == 1) {
        str0 = QString::fromUtf8("白光干涉仪-侧面1;\r\n");
        a02_b = aa_mat(0, 0);
        a12_b = aa_mat(1, 0);
        a22_b = aa_mat(2, 0);

    }
    QString str = str0 + "Z = " + a0 + "X " + a1 + "Y " + a2;
    ui.textEdit->insertPlainText(str += '\n');
}

void DataFusion::leastSquaresFit_Plane3()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    //-------------加载点云-------------------
    QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", ".", "Open PCD files(*.txt)");
    if (!fileName.isEmpty()) {
        std::string file_name = fileName.toStdString();
        loadCloud(file_name, cloud_in);//---------调用读取函数------------
        //pcl::PCLPointCloud2 cloud2;
        //Eigen::Vector4f origin;
        //Eigen::Quaternionf orientation;
        //int pcd_version;
        //int data_type;
        //unsigned int data_idx;
        //int offset = 0;
        //pcl::PCDReader rd;
        //rd.readHeader(file_name, cloud2, origin, orientation, pcd_version, data_type, data_idx);//(out) data_type数据类型（0=ASCII，1=二进制，2=二进制压缩）
        ////pcd文件用记事本打开，正常的是ASCII，乱码的是二进制
        //if (data_type == 0)//打开ASCII
        //{
        //    pcl::io::loadPCDFile(fileName.toStdString(), *cloud_in);
        //}
        //else if (data_type == 2 || data_type == 1)//打开二进制或二进制压缩!!
        //{
        //    pcl::PCDReader reader;
        //    reader.read<pcl::PointXYZ>(fileName.toStdString(), *cloud_in);
        //}
    }

    //去除nan点
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);

    //pcl::VoxelGrid<pcl::PointXYZ> sor;//提速滤波下采样，加快拟合速度，可选（需包含头文件）
    //sor.setInputCloud(cloud_in);
    //sor.setLeafSize(0.01f, 0.01f, 0.01f);
    //sor.filter(*cloud_in);

    //最小二乘平面拟合
    std::size_t cloud_size = cloud_in->points.size();
    std::cout << "方程个数：" << cloud_size << std::endl;
    try//异常处理，关键字try开始，catch子句结束。try语句块中代码异常通常被catch子句处理。
    {
        if (cloud_size < 3)//小于三个点无法计算
        {
            throw std::runtime_error("点云数据出错，请检查数据");
            QMessageBox msgBox;
            msgBox.setText("Error in point cloud data");
            msgBox.exec();
        }
    }
    catch (std::runtime_error err)
    {
        std::cerr << err.what() << std::endl;
        return;
    }
    Eigen::MatrixXd B;
    B.resize(cloud_size, 3);
    Eigen::MatrixXd L;
    L.resize(cloud_size, 1);

    for (size_t row = 0; row < cloud_size; ++row)
    {
        B(row, 0) = cloud_in->points[row].x;
        B(row, 1) = cloud_in->points[row].y;
        B(row, 2) = 1;
        L(row, 0) = cloud_in->points[row].z;
    }
    Eigen::Matrix<double, 3, 1> aa_mat;//z= aox+ a1y+ a2
    aa_mat = ((B.transpose() * B)).lu().solve(B.transpose() * L); //lu分解(矩阵A分解为 L,U 下上三角矩阵)求逆，对于大型矩阵加速。
    //aa_mat = ((B.transpose() * B)).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B.transpose() * L);//SVD结果稳定特别是对于矩阵接近病态，但速度慢。
    //aa_mat = (B.transpose() * B).inverse() * B.transpose() * L;//直接解法      
    std::cout << "点云平面拟合结果\n" << showpos << aa_mat << std::endl;
    
    QString a0, a1, a2;
    a0 = QString::number(aa_mat(0, 0));
    a1 = QString::number(aa_mat(1, 0));
    a2 = QString::number(aa_mat(2, 0));
    if (aa_mat(1, 0) > 0)
        a1 = "+" + a1;
    if (aa_mat(2, 0) > 0)
        a2 = "+" + a2;
    int comboBox_currentIndex = ui.comboBox->currentIndex();
    QString str0;
    if (comboBox_currentIndex == 0) {
        str0 = QString::fromUtf8("坐标测量机-侧面2;\r\n");
        a03 = aa_mat(0, 0);
        a13 = aa_mat(1, 0);
        a23 = aa_mat(2, 0);
    }
    else if (comboBox_currentIndex == 1) {
        str0 = QString::fromUtf8("白光干涉仪-侧面2;\r\n");
        a03_b = aa_mat(0, 0);
        a13_b = aa_mat(1, 0);
        a23_b = aa_mat(2, 0);
    }
    QString str = str0 + "Z = " + a0 + "X " + a1 + "Y " + a2;
    ui.textEdit->insertPlainText(str += '\n');
}

void DataFusion::leastSquaresFit_Plane4()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    //-------------加载点云-------------------
    QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", ".", "Open PCD files(*.txt)");
    if (!fileName.isEmpty()) {
        std::string file_name = fileName.toStdString();
        loadCloud(file_name, cloud_in);//---------调用读取函数------------
        //pcl::PCLPointCloud2 cloud2;
        //Eigen::Vector4f origin;
        //Eigen::Quaternionf orientation;
        //int pcd_version;
        //int data_type;
        //unsigned int data_idx;
        //int offset = 0;
        //pcl::PCDReader rd;
        //rd.readHeader(file_name, cloud2, origin, orientation, pcd_version, data_type, data_idx);//(out) data_type数据类型（0=ASCII，1=二进制，2=二进制压缩）
        ////pcd文件用记事本打开，正常的是ASCII，乱码的是二进制
        //if (data_type == 0)//打开ASCII
        //{
        //    pcl::io::loadPCDFile(fileName.toStdString(), *cloud_in);
        //}
        //else if (data_type == 2 || data_type == 1)//打开二进制或二进制压缩!!
        //{
        //    pcl::PCDReader reader;
        //    reader.read<pcl::PointXYZ>(fileName.toStdString(), *cloud_in);
        //}
    }

    //去除nan点
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);

    //pcl::VoxelGrid<pcl::PointXYZ> sor;//提速滤波下采样，加快拟合速度，可选（需包含头文件）
    //sor.setInputCloud(cloud_in);
    //sor.setLeafSize(0.01f, 0.01f, 0.01f);
    //sor.filter(*cloud_in);

    //最小二乘平面拟合
    std::size_t cloud_size = cloud_in->points.size();
    std::cout << "方程个数：" << cloud_size << std::endl;
    try//异常处理，关键字try开始，catch子句结束。try语句块中代码异常通常被catch子句处理。
    {
        if (cloud_size < 3)//小于三个点无法计算
        {
            throw std::runtime_error("点云数据出错，请检查数据");
            QMessageBox msgBox;
            msgBox.setText("Error in point cloud data");
            msgBox.exec();
        }
    }
    catch (std::runtime_error err)
    {
        std::cerr << err.what() << std::endl;
        return;
    }
    Eigen::MatrixXd B;
    B.resize(cloud_size, 3);
    Eigen::MatrixXd L;
    L.resize(cloud_size, 1);

    for (size_t row = 0; row < cloud_size; ++row)
    {
        B(row, 0) = cloud_in->points[row].x;
        B(row, 1) = cloud_in->points[row].y;
        B(row, 2) = 1;
        L(row, 0) = cloud_in->points[row].z;
    }
    Eigen::Matrix<double, 3, 1> aa_mat;//z= aox+ a1y+ a2
    aa_mat = ((B.transpose() * B)).lu().solve(B.transpose() * L); //lu分解(矩阵A分解为 L,U 下上三角矩阵)求逆，对于大型矩阵加速。
    //aa_mat = ((B.transpose() * B)).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B.transpose() * L);//SVD结果稳定特别是对于矩阵接近病态，但速度慢。
    //aa_mat = (B.transpose() * B).inverse() * B.transpose() * L;//直接解法      
    std::cout << "点云平面拟合结果\n" << showpos << aa_mat << std::endl;
    
    QString a0, a1, a2;
    a0 = QString::number(aa_mat(0, 0));
    a1 = QString::number(aa_mat(1, 0));
    a2 = QString::number(aa_mat(2, 0));
    if (aa_mat(1, 0) > 0)
        a1 = "+" + a1;
    if (aa_mat(2, 0) > 0)
        a2 = "+" + a2;
    int comboBox_currentIndex = ui.comboBox->currentIndex();
    QString str0;
    if (comboBox_currentIndex == 0) {
        str0 = QString::fromUtf8("坐标测量机-侧面3;\r\n");
        a04 = aa_mat(0, 0);
        a14 = aa_mat(1, 0);
        a24 = aa_mat(2, 0);
    }
    else if (comboBox_currentIndex == 1) {
        str0 = QString::fromUtf8("白光干涉仪-侧面3;\r\n");
        a04_b = aa_mat(0, 0);
        a14_b = aa_mat(1, 0);
        a24_b = aa_mat(2, 0);
    }
    QString str = str0 + "Z = " + a0 + "X " + a1 + "Y " + a2;
    ui.textEdit->insertPlainText(str += '\n');
}

void DataFusion::RANSAC_plan()
{
    /*
    随机采样一致性拟合平面
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr sac_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("sac_plane_test.pcd", *sac_cloud) == -1)
    {
        PCL_ERROR("点云读取失败 \n"); return ;       
    }
    //------------------------------------------RANSAC框架--------------------------------------------------------   
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(sac_cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);//定义RANSAC算法模型
    ransac.setDistanceThreshold(0.01);//设定距离阈值
    ransac.setMaxIterations(500);     //设置最大迭代次数
    ransac.setProbability(0.99);      //设置从离群值中选择至少一个样本的期望概率
    ransac.computeModel();            //拟合平面
    vector<int> inliers0;              //用于存放内点索引的vector
    ransac.getInliers(inliers0);       //获取内点索引
    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);  //获取拟合平面参数，coeff分别按顺序保存a,b,c,d

    cout << "平面模型系数coeff(a,b,c,d): " << coeff[0] << " \t" << coeff[1] << "\t " << coeff[2] << "\t " << coeff[3] << endl;
    /*
        //-------------------平面法向量定向，与（1，1，1）同向，并输出平面与原点的距离D---------------------------
        double a, b, c, d, A, B, C, D;//a,b,c为拟合平面的单位法向量，A,B,C为重定向后的法向量
        a = coeff[0], b = coeff[1], c = coeff[2], d = coeff[3];
        if (a + b + c > 0) {
            A = a;
            B = b;
            C = c;
            D = abs(d);
        }
        else {
            A = -a;
            B = -b;
            C = -c;
            D = abs(d);
        }
        cout << "" << A << ",\t" << "" << B << ",\t" << "" << C << ",\t" << "" << D << ",\t" << endl;
        */
    //--------------------------------根据内点索引提取拟合的平面点云-----------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr after_sac_plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud<pcl::PointXYZ>(*sac_cloud, inliers0, *after_sac_plane); // pcl::io::savePCDFileASCII("1.11.pcd", *final);
   
    /*
    或者
    */
    //--------------------------------生成一个点云--------------------------------
    //pcl::PointCloud<pcl::PointXYZ>::Ptr create_cloud(new pcl::PointCloud<pcl::PointXYZ>);			//智能指针初始化点云
    //create_cloud->width = 15;
    //create_cloud->height = 1;
    //create_cloud->points.resize(create_cloud->width * create_cloud->height);
    //for (size_t i = 0; i < create_cloud->points.size(); ++i)
    //{
    //    create_cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    //    create_cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    //    create_cloud->points[i].z = 1.0;
    //}
    //create_cloud->points[0].z = 2.0;																//手动设置离群点
    //create_cloud->points[3].z = -2.0;
    //create_cloud->points[6].z = 4.0;
    //std::cerr << "Point create_cloud data:" << create_cloud->points.size() << "points" << std::endl;			//输出当前点云中的点的信息
    //for (size_t i = 0; i < create_cloud->points.size(); ++i)
    //    std::cerr << "Clouds data:" << create_cloud->points[i].x << " " << create_cloud->points[i].y << " " << create_cloud->points[i].z << std::endl;
    
    //------------------------------------------SACMODEL_PLANE--------------------------------------------------------   
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);//智能指针保存模型信息
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);				 //智能指针保存内点//inliers表示误差能容忍的点 记录的是点云的序号  
    pcl::SACSegmentation<pcl::PointXYZ> seg;							 //创建分割器
    seg.setOptimizeCoefficients(true);  //必须要有!!
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);																//设定距离阈值，小于该阈值则为局内点，大于该阈值则为局外点
    //seg.setMaxIterations(500);
    //seg.setAxis(Eigen::Vector3f(0, 0, 1));
    //seg.setEpsAngle(45.0f * (M_PI / 180.0f));
    seg.setInputCloud(sac_cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {  PCL_ERROR("Cloud not estimate a planar model for the given dataset.");return;  }
    //std::cerr << "ax坐标轴" << seg.getAxis() << std::endl;
    std::cerr << "Model coefficients: "
        << coefficients->values[0] << " "
        << coefficients->values[1] << " "
        << coefficients->values[2] << " "
        << coefficients->values[3] << std::endl;
    for (size_t i = 0; i < inliers->indices.size(); ++i)
        std::cerr << inliers->indices[i] << " " 
        << sac_cloud->points[inliers->indices[i]].x << " " 
        << sac_cloud->points[inliers->indices[i]].y << " " 
        << sac_cloud->points[inliers->indices[i]].z << " " << std::endl;       
    //--------------------------------根据内点索引提取拟合的平面点云-----------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>);      
    pcl::copyPointCloud(*sac_cloud, *inliers, *inlierPoints);//只取inliners中索引对应的点拷贝到inlierPoints中
    // pcl::io::savePCDFileASCII("1.11.pcd", *inlierPoints);
}

void DataFusion::threeCoordinate()//三坐标数据---
{
    //三坐标---
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2(new pcl::PointCloud<pcl::PointXYZ>);                       //三坐标-------------
    QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", ".", "Open PCD files(*.txt *.pcd)");
    if (!fileName.isEmpty()) {
        if (fileName.back() == 't') {          //打开txt---
            loadCloud(fileName.toStdString(), cloud_in2);//调用读取函数---   
        }
        else if (fileName.back() == 'd') {    //打开txt---
            pcl::PCLPointCloud2 cloud2;
            Eigen::Vector4f origin;
            Eigen::Quaternionf orientation;
            int pcd_version;
            int data_type;
            unsigned int data_idx;
            int offset = 0;
            pcl::PCDReader rd;
            rd.readHeader(fileName.toStdString(), cloud2, origin, orientation, pcd_version, data_type, data_idx);//(out) data_type数据类型（0=ASCII，1=二进制，2=二进制压缩）
            //pcd文件用记事本打开，正常的是ASCII，乱码的是二进制
            if (data_type == 0)//打开ASCII
            {
                pcl::io::loadPCDFile(fileName.toStdString(), *cloud_in2);
            }
            else if (data_type == 2 || data_type == 1)//打开二进制或二进制压缩!!
            {
                pcl::PCDReader reader;
                reader.read<pcl::PointXYZ>(fileName.toStdString(), *cloud_in2);
            }
        }
    }
    std::vector<int> indices2;  //去除nan点
    pcl::removeNaNFromPointCloud(*cloud_in2, *cloud_in2, indices2);
}

void DataFusion::whiteLight()//干涉仪数据---
{
    //白光---
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1(new pcl::PointCloud<pcl::PointXYZ>);                       //白光--------------
    //-------------加载点云-------------------
    QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", ".", "Open PCD files(*.txt *.pcd)");
    if (!fileName.isEmpty()) {
        if (fileName.back() == 't') {          //打开txt---
            loadCloud(fileName.toStdString(), cloud_in1);//调用读取函数---   
        }
        else if (fileName.back() == 'd') {    //打开txt---
            pcl::PCLPointCloud2 cloud2;
            Eigen::Vector4f origin;
            Eigen::Quaternionf orientation;
            int pcd_version;
            int data_type;
            unsigned int data_idx;
            int offset = 0;
            pcl::PCDReader rd;
            rd.readHeader(fileName.toStdString(), cloud2, origin, orientation, pcd_version, data_type, data_idx);//(out) data_type数据类型（0=ASCII，1=二进制，2=二进制压缩）
            //pcd文件用记事本打开，正常的是ASCII，乱码的是二进制
            if (data_type == 0)//打开ASCII
            {
                pcl::io::loadPCDFile(fileName.toStdString(), *cloud_in1);
            }
            else if (data_type == 2 || data_type == 1)//打开二进制或二进制压缩!!
            {
                pcl::PCDReader reader;
                reader.read<pcl::PointXYZ>(fileName.toStdString(), *cloud_in1);
            }
        }
    }
    std::vector<int> indices;//去除nan点
    pcl::removeNaNFromPointCloud(*cloud_in1, *cloud_in1, indices);
}

void DataFusion::KDtreeSerch()
{    
    ////白光---
    ////pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1(new pcl::PointCloud<pcl::PointXYZ>);                       //白光---
    ////-------------加载点云-------------------
    //QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", ".", "Open PCD files(*.txt)");
    //if (fileName.isEmpty())return;
    //else if (!fileName.isEmpty()) {
    //    std::string file_name = fileName.toStdString();
    //    //txt转pcd
    //    loadCloud(file_name, cloud_in1);//---------调用读取函数------------       
    //} 
    ////三坐标---
    ////pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2(new pcl::PointCloud<pcl::PointXYZ>);                       //三坐标---
    //QString fileName2 = QFileDialog::getOpenFileName(this, "Open PointCloud", ".", "Open PCD files(*.txt)");
    //if (fileName.isEmpty())return;
    //else if (!fileName2.isEmpty()) {
    //    std::string file_name2 = fileName2.toStdString();
    //    loadCloud(file_name2, cloud_in2);//txt转pcd      
    //}

    //std::vector<int> indices;//去除nan点
    //pcl::removeNaNFromPointCloud(*cloud_in1, *cloud_in1, indices);
    //std::vector<int> indices2;  
    //pcl::removeNaNFromPointCloud(*cloud_in2, *cloud_in2, indices2);
    
                                                                    ////插入和删除点云
                                                                    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);                                                                    
                                                                    //pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
                                                                    //cloud->erase(index);//删除第一个
                                                                    //index = cloud->begin() + 5;
                                                                    //cloud->erase(cloud->begin());//删除第5个
                                                                    //pcl::PointXYZ point = { 1, 1, 1 };
                                                                    ////在索引号为5的位置1上插入一点，原来的点后移一位
                                                                    //cloud->insert(cloud->begin() + 5, point);
                                                                    //cloud->push_back(point);//从点云最后面插入一点
                                                                    //std::cout << cloud->points[5].x;//输出1
       
    //--------------------cloud_in1---密集点的白光干涉仪数据--------------------
    //--------------------cloud_in2---稀疏点三坐标测量机数据--------------------
    // 2.创建kdtree对象，并将随机创建的云设置为输入点云。然后指定一个随机坐标作为“搜索点”。//================ copyPointCloud(*cloud, *cloud2);拷贝点云===========       
    std::size_t cloud_in_size = cloud_in1->points.size();  //白光点云数量                                                         
    std::size_t cloud_in2_size = cloud_in2->points.size();//三坐标点云数

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);//新建cloud_out存储索引的点----
    cloud_out->width = cloud_in2->width;
    cloud_out->height = cloud_in2->height;
    cloud_out->points.resize(cloud_in2_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out2(new pcl::PointCloud<pcl::PointXYZ>);//新建cloud_out2存储融合后的点----
    cloud_out2->width = cloud_in2->width;
    cloud_out2->height = cloud_in2->height;
    cloud_out2->points.resize(cloud_in2_size);

    for (int i = 0; i < cloud_in2_size; ++i)
    {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud_in1); // 输入点云
        pcl::PointXYZ searchPoint;   // 搜索点
        searchPoint.x = cloud_in2->points[i].x;
        searchPoint.y = cloud_in2->at(i).y;
        searchPoint.z = cloud_in2->at(i).z;

        // 3、创建一个整数K和两个向量，用于从搜索中存储搜索点的K近邻。
        int K = 1;                                   // 需要查找的近邻点个数
        std::vector<int> pointIdxKNNSearch(K);        // 保存每个近邻点的索引
        std::vector<float> pointKNNSquaredDistance(K);// 保存每个近邻点与查找点之间的欧式距离平方
        //std::cout << "The nearest neighbor search at (" << searchPoint.x<< " " << searchPoint.y<< " " << searchPoint.z<< std::endl;

        // 4、打印出随机“搜索点”的所有10个最近邻居的位置，这些位置已经存储在先前创建的向量中。
        //if (kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0) // > 0表示能够找到近邻点， = 0表示找不到近邻点
        kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance);//===========kd-tree最近点搜索======================

        int idx;//存储选中点的索引，就是点的位置
        idx = pointIdxKNNSearch[0];
        //cout << "the choiced point at:" << idx << endl;
        //   //std::cout << "    " << (*cloud_in1)[pointIdxKNNSearch[i]].x<< " " << (*cloud_in1)[pointIdxKNNSearch[i]].y<< " " << (*cloud_in1)[pointIdxKNNSearch[i]].z<< " (squared distance: " << pointKNNSquaredDistance[i] << ")" << std::endl;

        (*cloud_out)[i].x = (*cloud_in1)[pointIdxKNNSearch[0]].x;
        (*cloud_out)[i].y = (*cloud_in1)[pointIdxKNNSearch[0]].y;
        (*cloud_out)[i].z = (*cloud_in1)[pointIdxKNNSearch[0]].z;

        (*cloud_out2)[i].x = 0.01 * (*cloud_in1)[pointIdxKNNSearch[0]].x + 0.99 * cloud_in2->points[i].x;//加权平均融合
        (*cloud_out2)[i].y = 0.01 * (*cloud_in1)[pointIdxKNNSearch[0]].y + 0.99 * cloud_in2->points[i].y;
        (*cloud_out2)[i].z = 0.5  * ((*cloud_in1)[pointIdxKNNSearch[0]].z + 1.0 * cloud_in2->points[i].z );

        cloud_in1->push_back(cloud_out2->points[i]);//将融合后的点添加到白光点云中！！！
    }
    for (int j = 0; j < cloud_in2_size; ++j) {
        int i = 0;
        for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_in1->begin(); it != cloud_in1->end(); ++it){//(int i = 0; i < cloud_in_size; ++i) {

            if (cloud_in1->points[i].x == cloud_out->points[i].x && cloud_in1->points[i].y == cloud_out->points[i].y && cloud_in1->points[i].z == cloud_out->points[i].z) {
                cloud_in1->erase(it);//删除白光点云中索引出的要融合的点
                ++i;
            }
        }           
    }
    //-----------cloud_in1输出融合后的密集白光点云-----------
    //-----------cloud_out2存储融合部分的点云---------------- 
    ////pcl::io::saveASCIIFile<pcl::PointXYZ>("output.txt", *cloud);// 保存txt点云(不知到为啥不能用）
    pcl::io::savePCDFileASCII("cloud_out2.pcd", *cloud_out2); // 保存txt点云
    pcl::io::savePCDFileASCII("cloud_in1.pcd", *cloud_in1);
    //pcl::io::savePCDFile("cloud_out2.pcd", *cloud_out2);// 保存pcd点云
    //pcl::io::savePCDFile("cloud_in1.pcd", *cloud_in1);

    ////融合后点云可视化
    //vector<Point> p = findMaxAndMin(cloud_in1); //输入点云的最值点
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1_n(new pcl::PointCloud<pcl::PointXYZ>); //缓存归一化之后的点云
    //cloud_in1_n = normialize(cloud_in1, p); //归一化---------------------同时显示融合部分归一化将变形，不可
    //MyVisualization(cloud_in1_n);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1_view(new pcl::PointCloud<pcl::PointXYZ>);//------同时减去某个点，使之贴近坐标轴----------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out2_view(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_in1_view->width = cloud_in1->width;
    cloud_in1_view->height = cloud_in1->height;
    cloud_in1_view->points.resize(cloud_in1->size());

    cloud_out2_view->width = cloud_out2->width;
    cloud_out2_view->height = cloud_out2->height;
    cloud_out2_view->points.resize(cloud_out2->size());
    int X00 = cloud_in1->points[0].x, Y00 = cloud_in1->points[0].y, Z00 = cloud_in1->points[0].z;
    for (int i = 0; i < cloud_in1->size(); ++i) {
        cloud_in1_view->points[i].x = cloud_in1->points[i].x - X00;
        cloud_in1_view->points[i].y = cloud_in1->points[i].y - Y00;
        cloud_in1_view->points[i].z = cloud_in1->points[i].z - Z00;
    }
    for (int i = 0; i < cloud_out2->size(); ++i) {
        cloud_out2_view->points[i].x = cloud_out2->points[i].x - X00;
        cloud_out2_view->points[i].y = cloud_out2->points[i].y - Y00;
        cloud_out2_view->points[i].z = cloud_out2->points[i].z - Z00;
    }

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud_in1_view, "z"); // 按照z字段进行渲染
    viewer->updatePointCloud<pcl::PointXYZ>(cloud_in1_view, fildColor, "cloud");                          //viewer->updatePointCloud(cloud, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");//设置点云再视窗中的显示方式，渲染属性，大小

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> fusion_color(cloud_out2_view, 0, 255, 255);//创建一个自定义的颜色处理器PointCloudHandlerCustom对象，设置颜色
    viewer->addPointCloud<pcl::PointXYZ>(cloud_out2_view, fusion_color, "fusion_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "fusion_cloud");

    // viewer->addCoordinateSystem(0.2);//添加坐标轴
    viewer->resetCamera();
    ui.qvtkWidget->update();
}

void DataFusion::KDSerch_leastSquaresFit_Fusioin()
{
    //--------------------cloud_in1---密集点的白光干涉仪数据--------------------
    //--------------------cloud_in2---稀疏点三坐标测量机数据--------------------
    // 2.创建kdtree对象，并将随机创建的云设置为输入点云。然后指定一个随机坐标作为“搜索点”。//================ copyPointCloud(*cloud, *cloud2);拷贝点云===========       
    std::size_t cloud_in_size = cloud_in1->points.size();  //白光点云数量                                                         
    std::size_t cloud_in2_size = cloud_in2->points.size();//三坐标点云数

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);//新建cloud_out存储索引的点----
    cloud_out->width = cloud_in2->width;
    cloud_out->height = cloud_in2->height;
    cloud_out->points.resize(cloud_in2_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out2(new pcl::PointCloud<pcl::PointXYZ>);//新建cloud_out2存储融合后的点----
    cloud_out2->width = cloud_in2->width;
    cloud_out2->height = cloud_in2->height;
    cloud_out2->points.resize(cloud_in2_size);

    for (int i = 0; i < cloud_in2_size; ++i)
    {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud_in1); // 输入点云
        pcl::PointXYZ searchPoint;   // 搜索点
        searchPoint.x = cloud_in2->points[i].x;
        searchPoint.y = cloud_in2->at(i).y;
        searchPoint.z = cloud_in2->at(i).z;

        // 3、创建一个整数K和两个向量，用于从搜索中存储搜索点的K近邻。
        int K = 1;                                   // 需要查找的近邻点个数
        std::vector<int> pointIdxKNNSearch(K);        // 保存每个近邻点的索引
        std::vector<float> pointKNNSquaredDistance(K);// 保存每个近邻点与查找点之间的欧式距离平方
        //std::cout << "The nearest neighbor search at (" << searchPoint.x<< " " << searchPoint.y<< " " << searchPoint.z<< std::endl;

        // 4、打印出随机“搜索点”的所有10个最近邻居的位置，这些位置已经存储在先前创建的向量中。
        //if (kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0) // > 0表示能够找到近邻点， = 0表示找不到近邻点
        kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance);//===========kd-tree最近点搜索======================

        int idx;//存储选中点的索引，就是点的位置
        idx = pointIdxKNNSearch[0];
        //cout << "the choiced point at:" << idx << endl;
        //   //std::cout << "    " << (*cloud_in1)[pointIdxKNNSearch[i]].x<< " " << (*cloud_in1)[pointIdxKNNSearch[i]].y<< " " << (*cloud_in1)[pointIdxKNNSearch[i]].z<< " (squared distance: " << pointKNNSquaredDistance[i] << ")" << std::endl;

        (*cloud_out)[i].x = (*cloud_in1)[pointIdxKNNSearch[0]].x;
        (*cloud_out)[i].y = (*cloud_in1)[pointIdxKNNSearch[0]].y;
        (*cloud_out)[i].z = (*cloud_in1)[pointIdxKNNSearch[0]].z;

        // 设置XYZ各纬度的标准差
        double sigma1 = 300 * pow(10, -9), sigma2 = 1700 * pow(10, -9), sigma3 = 100 * pow(10, -9);//三坐标zyz、白光xy、白光z       
        (*cloud_out2)[i].x = (pow(sigma3, 2) * cloud_in2->points[i].x + pow(sigma1, 2) * (*cloud_in1)[pointIdxKNNSearch[0]].x) / (pow(sigma1, 2) + pow(sigma3, 2));//加权最小二乘融合 
        (*cloud_out2)[i].y = (pow(sigma3, 2) * cloud_in2->points[i].y + pow(sigma1, 2) * (*cloud_in1)[pointIdxKNNSearch[0]].y) / (pow(sigma1, 2) + pow(sigma3, 2));
        (*cloud_out2)[i].z = (pow(sigma3, 2) * cloud_in2->points[i].z + pow(sigma2, 2) * (*cloud_in1)[pointIdxKNNSearch[0]].z) / (pow(sigma2, 2) + pow(sigma3, 2));

        cloud_in1->push_back(cloud_out2->points[i]);//将融合后的点添加到白光点云中!!!
    }
    for (int j = 0; j < cloud_in2_size; ++j) {
        int i = 0;
        for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_in1->begin(); it != cloud_in1->end(); ++it) {//(int i = 0; i < cloud_in_size; ++i) {

            if (cloud_in1->points[i].x == cloud_out->points[i].x && cloud_in1->points[i].y == cloud_out->points[i].y && cloud_in1->points[i].z == cloud_out->points[i].z) {
                cloud_in1->erase(it);//删除白光点云中索引出的要融合的点
                ++i;
            }
        }
    }
    //-----------cloud_in1输出融合后的密集白光点云-----------
    //-----------cloud_out2存储融合部分的点云----------------
    ////pcl::io::saveASCIIFile<pcl::PointXYZ>("output.txt", *cloud);// 保存txt点云(不知到为啥不能用）
    pcl::io::savePCDFileASCII("fusiondCloud.pcd", *cloud_out2); 
    pcl::io::savePCDFileASCII("cloud_in1.pcd", *cloud_in1);

    //pcl::io::savePCDFile("cloud_out2.pcd", *cloud_out2);// 保存pcd点云
    //pcl::io::savePCDFile("cloud_in1.pcd", *cloud_in1);


    ////融合后点云可视化
    //vector<Point> p = findMaxAndMin(cloud_in1); //输入点云的最值点
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1_n(new pcl::PointCloud<pcl::PointXYZ>); //缓存归一化之后的点云
    //cloud_in1_n = normialize(cloud_in1, p); //归一化---------------------同时显示融合部分归一化将变形，不可
    //MyVisualization(cloud_in1_n);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1_view(new pcl::PointCloud<pcl::PointXYZ>);//------同时减去某个点，使之贴近坐标轴----------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out2_view(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_in1_view->width = cloud_in1->width;
    cloud_in1_view->height = cloud_in1->height;
    cloud_in1_view->points.resize(cloud_in1->size());

    cloud_out2_view->width = cloud_out2->width;
    cloud_out2_view->height = cloud_out2->height;
    cloud_out2_view->points.resize(cloud_out2->size());
    int X00 = cloud_in1->points[0].x, Y00 = cloud_in1->points[0].y, Z00 = cloud_in1->points[0].z;
    for (int i = 0; i < cloud_in1->size(); ++i) {
        cloud_in1_view->points[i].x = cloud_in1->points[i].x - X00;
        cloud_in1_view->points[i].y = cloud_in1->points[i].y - Y00;
        cloud_in1_view->points[i].z = cloud_in1->points[i].z - Z00; 
    }
    for (int i = 0; i < cloud_out2->size(); ++i) {
        cloud_out2_view->points[i].x = cloud_out2->points[i].x - X00;
        cloud_out2_view->points[i].y = cloud_out2->points[i].y - Y00;
        cloud_out2_view->points[i].z = cloud_out2->points[i].z - Z00;
    }

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud_in1_view, "z"); // 按照z字段进行渲染
    viewer->updatePointCloud<pcl::PointXYZ>(cloud_in1_view, fildColor, "cloud");                          //viewer->updatePointCloud(cloud, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");//设置点云再视窗中的显示方式，渲染属性，大小

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> fusion_color(cloud_out2_view, 0, 255, 255);//创建一个自定义的颜色处理器PointCloudHandlerCustom对象，设置颜色
    viewer->addPointCloud<pcl::PointXYZ>(cloud_out2_view, fusion_color, "fusion_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "fusion_cloud");

   // viewer->addCoordinateSystem(0.2);//添加坐标轴
    viewer->resetCamera();
    ui.qvtkWidget->update();

}

void DataFusion::MyVisualization(pcl::PointCloud < pcl::PointXYZ>::Ptr cloud_in )
{
    viewer->removeAllPointClouds();//刷新图层//若以后改动代码不显示点云，从此处找原因，addPointCloud、updatePointCloud

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud_in, "z"); // 按照z字段进行渲染
    viewer->addPointCloud<pcl::PointXYZ>(cloud_in, fildColor, "cloud");                          //viewer->updatePointCloud(cloud, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");//设置点云再视窗中的显示方式，渲染属性，大小
    viewer->addCoordinateSystem(0.2);//添加坐标轴
    //viewer->initCameraParameters();//通过设置照相机参数，使得从默认的角度和方向观察点云？？？
   
    viewer->resetCamera();  
    ui.qvtkWidget->update();
}

void DataFusion::shouCloud()//------------------------------普通显示点云------------------------------
{
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);//不可省！否则点云重复堆叠
    //-------------加载点云-------------------
    QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", ".", "Open PCD files(*.txt *.pcd)");
    if (!fileName.isEmpty()) {
        if (fileName.back() == 't') {          //打开txt---
            loadCloud(fileName.toStdString(), cloud);//调用读取函数---   
        }
        else if (fileName.back() == 'd') {    //打开txt---
            pcl::PCLPointCloud2 cloud2;
            Eigen::Vector4f origin;
            Eigen::Quaternionf orientation;
            int pcd_version;
            int data_type;
            unsigned int data_idx;
            int offset = 0;
            pcl::PCDReader rd;
            rd.readHeader(fileName.toStdString(), cloud2, origin, orientation, pcd_version, data_type, data_idx);//(out) data_type数据类型（0=ASCII，1=二进制，2=二进制压缩）
            //pcd文件用记事本打开，正常的是ASCII，乱码的是二进制
            if (data_type == 0)//打开ASCII
            {
                pcl::io::loadPCDFile(fileName.toStdString(), *cloud);
            }
            else if (data_type == 2 || data_type == 1)//打开二进制或二进制压缩!!
            {
                pcl::PCDReader reader;
                reader.read<pcl::PointXYZ>(fileName.toStdString(), *cloud);
            }
        }
    }
    MyVisualization(cloud);
}

/*---------------------------归一化显示points------------------------------
1.主要是定义一个函数找到pcd文件的最大点最小点
2.然后在另外一个函数中理由最值点对点云进行归一化,变至(-1,1)这个区间
*/
vector<Point> findMaxAndMin(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
//--------重载，有filename的是按filename为命名来保存归一化放大后的点云--------
pcl::PointCloud<pcl::PointXYZ>::Ptr normialize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, vector<Point> MaxAndMin);
pcl::PointCloud<pcl::PointXYZ>::Ptr normialize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, vector<Point> MaxAndMin, string filename);
void DataFusion::shouCloud_normalized()//归一化显示点云
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);   
    //-------------加载点云-------------------
    QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", ".", "Open PCD files(*.txt *.pcd)");
    if (!fileName.isEmpty()) {
        if (fileName.back() == 't') {          //打开txt---
            loadCloud(fileName.toStdString(), cloud1);//调用读取函数---   
        }
        else if (fileName.back() == 'd') {    //打开txt---
            pcl::PCLPointCloud2 cloud2;
            Eigen::Vector4f origin;
            Eigen::Quaternionf orientation;
            int pcd_version;
            int data_type;
            unsigned int data_idx;
            int offset = 0;
            pcl::PCDReader rd;
            rd.readHeader(fileName.toStdString(), cloud2, origin, orientation, pcd_version, data_type, data_idx);//(out) data_type数据类型（0=ASCII，1=二进制，2=二进制压缩）
            //pcd文件用记事本打开，正常的是ASCII，乱码的是二进制
            if (data_type == 0)//打开ASCII
            {
                pcl::io::loadPCDFile(fileName.toStdString(), *cloud1);
            }
            else if (data_type == 2 || data_type == 1)//打开二进制或二进制压缩!!
            {
                pcl::PCDReader reader;
                reader.read<pcl::PointXYZ>(fileName.toStdString(), *cloud1);
            }
        }
    }
    cout << cloud1->size() << endl;
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*cloud1, *cloud1, mapping);//去除点云无效点!!!（不可省）
    cout << cloud1->size() << endl;
    //for (int i = 0; i < cloud1->size(); ++i) { cout << cloud1->points[i].z << endl; }

    //    normialize(cloud,findMaxAndMin(cloud1));
    vector<Point> p = findMaxAndMin(cloud1); //输入点云的最值点
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_n(new pcl::PointCloud<pcl::PointXYZ>); //缓存归一化之后的点云
    //cloud_n = normialize(cloud1, p, fileName.toStdString() ); //归一化//重载，有filename的是按filename为命名来保存归一化放大后的点云
    cloud_n = normialize(cloud1, p); //归一化
    cout << "\nafter normalized:" << endl;
    vector<Point> p_ = findMaxAndMin(cloud_n); //归一化点云之后的最值点
    MyVisualization(cloud_n);

   // //可视化
   //// pcl::visualization::PCLVisualizer viewer("3D Viewer");
   // //    int v1(0);
   // //
   // //    viewer.createViewPort(0.5,0,1,1,v1);
   // //    viewer.setBackgroundColor (1, 1, 1);
   // //    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_cloud_color_handler (cloud, 0, 0, 0);
   // //    viewer.addPointCloud (cloud, point_cloud_color_handler, "cloud",v1);
   // int v2(0);
   // viewer->createViewPort(0, 0, 0.5, 1, v2);
   // viewer->setBackgroundColor(1, 0.5, 0.5);
   // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cloud_n, 0, 0, 0);
   // viewer->addPointCloud(cloud_n, color2, "cloud_n", v2);
   // viewer->addText("cloud_normalized", 0, 0, "cloud_n", v2);
   // viewer->resetCamera();
   // ui.qvtkWidget->update();     
}

vector<Point> findMaxAndMin(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
    vector<Point> vector_back;
    Point max, min;
    max.x = cloud_in->points[0].x;
    max.y = cloud_in->points[0].y;
    max.z = cloud_in->points[0].z;

    min.x = cloud_in->points[0].x;
    min.y = cloud_in->points[0].y;
    min.z = cloud_in->points[0].z;

    for (int i = 0; i < cloud_in->size(); i++)
    {
        if (cloud_in->points[i].x >= max.x) { max.x = cloud_in->points[i].x; }
        if (cloud_in->points[i].x <= min.x) { min.x = cloud_in->points[i].x; }

        if (cloud_in->points[i].y >= max.y) { max.y = cloud_in->points[i].y; }
        if (cloud_in->points[i].y <= min.y) { min.y = cloud_in->points[i].y; }

        if (cloud_in->points[i].z >= max.z) { max.z = cloud_in->points[i].z; }
        if (cloud_in->points[i].z <= min.z) { min.z = cloud_in->points[i].z; }
    }

    vector_back.push_back(max);
    vector_back.push_back(min);

    cout << "max: " << max.x << "," << max.y << "," << max.z << endl;
    cout << "min: " << min.x << "," << min.y << "," << min.z << endl;

    return vector_back;

}

//归一化//重载，有filename的是按filename为命名来保存归一化放大后的点云
pcl::PointCloud<pcl::PointXYZ>::Ptr normialize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, vector<Point> MaxAndMin)//normialize归一化
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_back(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_back->width = cloud_in->width;
    cloud_back->height = cloud_in->height;
    cloud_back->is_dense = cloud_in->is_dense;
    cloud_back->resize(cloud_in->size());
    Point max_bound, min_bound;
    max_bound = MaxAndMin[0];
    min_bound = MaxAndMin[1];
    double middlex, middley, middlez;
    middlex = (max_bound.x + min_bound.x) / 2; //中间坐标（最大+最小坐标）/2
    middley = (max_bound.y + min_bound.y) / 2;
    middlez = (max_bound.z + min_bound.z) / 2;

    cout << middlex << "     " << middley << "     " << middlez << endl;

    for (int i = 0; i < cloud_back->size(); i++)
    {
        cloud_back->points[i].x = cloud_in->points[i].x - middlex;//所有坐标-中间值
        cloud_back->points[i].y = cloud_in->points[i].y - middley;
        cloud_back->points[i].z = cloud_in->points[i].z - middlez;
    }

    //scale based on x range
    double scalecoe = (max_bound.x - min_bound.x) / 2;
    cout << scalecoe << endl;
    for (int i = 0; i < cloud_back->size(); i++) {
        cloud_back->points[i].x = cloud_back->points[i].x / scalecoe;//均除以区间长度，归一化
        cloud_back->points[i].y = cloud_back->points[i].y / scalecoe;
        cloud_back->points[i].z = cloud_back->points[i].z / scalecoe;
    }
    //   //pcl::io::savePCDFileASCII("normalized.pcd",*cloud_back);
   //ostringstream oss;
   ////string aaa= "normalized_" + filename;
   //oss << "normalized_" << filename;
   ////pcl::io::savePCDFileASCII(oss.str(), *cloud_back);       //保存点云:normalized_文件名  
   //cout << "Normalized!" << endl;
   //cout << oss.str() << " Saved!" << endl;
    return cloud_back;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr normialize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, vector<Point> MaxAndMin, string filename)//normialize归一化
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_back(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_back->width = cloud_in->width;
    cloud_back->height = cloud_in->height;
    cloud_back->is_dense = cloud_in->is_dense;
    cloud_back->resize(cloud_in->size());
    Point max_bound, min_bound;
    max_bound = MaxAndMin[0];
    min_bound = MaxAndMin[1];
    double middlex, middley, middlez;
    middlex = (max_bound.x + min_bound.x) / 2; //中间坐标（最大+最小坐标）/2
    middley = (max_bound.y + min_bound.y) / 2;
    middlez = (max_bound.z + min_bound.z) / 2;

    cout << middlex << "     "<<middley << "     "<<middlez << endl;

    for (int i = 0; i < cloud_back->size(); i++)
    {
        cloud_back->points[i].x = cloud_in->points[i].x - middlex;//所有坐标-中间值
        cloud_back->points[i].y = cloud_in->points[i].y - middley;
        cloud_back->points[i].z = cloud_in->points[i].z - middlez;
    }

    //scale based on x range
    double scalecoe = (max_bound.x - min_bound.x) / 2;
    cout << scalecoe << endl;
    for (int i = 0; i < cloud_back->size(); i++) {
        cloud_back->points[i].x = cloud_back->points[i].x / scalecoe;//均除以区间长度，归一化
        cloud_back->points[i].y = cloud_back->points[i].y / scalecoe;
        cloud_back->points[i].z = cloud_back->points[i].z / scalecoe;
    }
        //pcl::io::savePCDFileASCII("normalized.pcd",*cloud_back);
    ostringstream oss;
    //string aaa= "normalized_" + filename;
    oss << "normalized_" << filename;
    //pcl::io::savePCDFileASCII(oss.str(), *cloud_back);       //保存点云:normalized_文件名  
    cout << "Normalized!" << endl;
    cout << oss.str() << " Saved!" << endl;
    return cloud_back;
}

