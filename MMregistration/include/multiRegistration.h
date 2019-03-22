#pragma once

#include <pcl/io/pcd_io.h>    //PLY相关头文件
#include <pcl/point_types.h>  //

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/ia_kfpcs.h>
#include <pcl/registration/impl/ia_kfpcs.hpp>

#include <pcl/features/fpfh.h>
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/features/normal_3d.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>

#include <pcl/console/time.h>   

#include <iostream>
#include <string>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudT;
using namespace std;

class multiRegistration
{

public:
	multiRegistration();
	~multiRegistration();

	/***********距离数据转2d（3d）点云数据**************/
	void 
	SplitString(const std::string& s, std::vector<string>& v, const std::string& c);

	int
	lidarDistanceToPointCloud_2(const std::string& filename, const pcl::PointCloud<PointT>::Ptr& cloud);

	int
	lidarDistanceToPointCloud_3(const std::string& filename, const pcl::PointCloud<PointT>::Ptr& cloud);

	int
	FpfhSACAlignment(std::vector<pcl::PointCloud<PointT>::Ptr>& pointCloudPtrVect, Eigen::Matrix4f& sac_trans);

	void 
	multiStationRegistration(pcl::PointCloud<PointT>::Ptr& ScanSetTarget, pcl::PointCloud<PointT>::Ptr& ScanSetSource, double& ICPscore);

	void 
	print4x4Matrix(const Eigen::Matrix4f & matrix);

	int 
	Save4x4MatrixVect(std::vector<Eigen::Matrix4f> &transformation_matrix_Vect, std::vector<double>& ICPscore, std::string &MatrixName, std::string &fliePath);

	int
	getAbsTransationMatrix(const std::vector<Eigen::Matrix4f> &transformation_matrix_Vect, std::vector<Eigen::Matrix4f> &AbsTransationMatrix);


public:
	pcl::PointCloud<PointT>::Ptr cloud_icp;  // ICP 输出点云
	Eigen::Matrix4f transformation_matrix= Eigen::Matrix4f::Identity();
	//全局变量参数
	float uniform_samplingRadiusSearch;  // 降采样搜索半径R1
	float NormalEstimationRadiusSearch = 0.05;  // 法线计算搜索半径R2 > 2R1;
	int iterations = 30;  // 默认的ICP迭代次数

private:

};

