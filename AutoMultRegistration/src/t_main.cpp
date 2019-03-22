#include "dbtype.h"
#include "dbkdtree.h"
#include "angleRegister2d.h"
#include "multiRegistration.h"

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#include<iostream>

int register2d(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_targ,
	float& icpScore, Eigen::Matrix4f& transformation_matrix, float& detaAngle)
{
	std::vector<db::Point2f> src_points;
	std::vector<db::Point2f> targ_points;
	Eigen::Matrix<float, 2, 3> RT = Eigen::Matrix<float, 2, 3>::Zero();
	//MatrixXf RT(2,3)
	for (size_t i = 0; i < cloud_in->size();)
	{
		db::Point2f Point2f;
		Point2f.x = cloud_in->points[i].x;
		Point2f.y = cloud_in->points[i].y;
		src_points.push_back(Point2f);
		i+=1;
	}
	for (size_t i = 0; i < cloud_targ->size(); )
	{
		db::Point2f Point2f;
		Point2f.x = cloud_targ->points[i].x;
		Point2f.y = cloud_targ->points[i].y;
		targ_points.push_back(Point2f);
		i+=1;
	}

	
	AngleRegister AR;
	if (AR.registration_2dLidar(src_points, targ_points, RT, detaAngle) == -1)
	{
		return -1;
	};

	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();      // 单位阵
	transform_1(0, 0) = RT(0, 0);
	transform_1(0, 1) = RT(0, 1);
	transform_1(0, 3) = RT(0, 2);

	transform_1(1, 0) = RT(1, 0);
	transform_1(1, 1) = RT(1, 1);
	transform_1(1, 3) = RT(1, 2);

	transform_1(2, 2) = 1;
	transform_1(3, 3) = 1;

	//cout << transform_1 << endl;
	// 执行变换
	//pcl::PointCloud<pcl::PointXYZ>::Ptr pPointCloudOut(new pcl::PointCloud<pcl::PointXYZ>());
	//pcl::transformPointCloud(*cloud_in, *pPointCloudOut, transform_1);
	//transformation_matrix = transform_1;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaxCorrespondenceDistance(0.5); //设置对应点对之间的最大距离（此值对配准结果影响较大）
	icp.setTransformationEpsilon(1e-10); //设置两次变化矩阵之间的差值（一般设置为1e-10即可）；
	icp.setEuclideanFitnessEpsilon(0.00001); //设置收敛条件是均方误差和小于阈值， 停止迭代
	icp.setMaximumIterations(100);     //设置最大迭代次数iterations=true
	icp.setInputSource(cloud_in);   //设置输入的点云
	icp.setInputTarget(cloud_targ);    //目标点云
	icp.align(*cloud_icp, transform_1);          //匹配后源点云
//	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();  //初始化
	if (icp.hasConverged())                 //icp.hasConverged ()=1（true）输出变换矩阵的适合性评估
	{
		//std::cout << "\n直线匹配 has converged, score is " << icp.getFitnessScore() << std::endl;
		icpScore = icp.getFitnessScore();
		transformation_matrix = icp.getFinalTransformation(); //.cast<double>();
	}
	

	if(transformation_matrix(1,0)>=0)
	{
		detaAngle=acos(transformation_matrix(0,0));
	}
	else
	{
		detaAngle=3.1415962-acos(transformation_matrix(0,0));
	}
	return 0;
}

int main(int argc, char **argv)
{
	if (argc != 6)      //pass three arguments to main,if not, print an error message
		throw runtime_error("wrong number of arguments");

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);  // 原始点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_targ(new pcl::PointCloud<pcl::PointXYZ>);  // 目标点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_targ_temp(new pcl::PointCloud<pcl::PointXYZ>);  // temp
	pcl::PointCloud<pcl::PointXYZ>::Ptr pPointCloudOut(new pcl::PointCloud<pcl::PointXYZ>());

	multiRegistration ml;
	if (ml.lidarDistanceToPointCloud_3(argv[1], cloud_in) == -1)  //"C:\\Users\\18148\\Desktop\\room_test\\003.txt"
	{
		throw runtime_error("reading data failure !!!");
	}
	if (ml.lidarDistanceToPointCloud_3(argv[2], cloud_targ_temp) == -1)
	{
		throw runtime_error("reading data failure !!!");
	}

	float delta_x = atof(argv[3]);
	float delta_y = atof(argv[4]);
	float theta = atof(argv[5]);
	Eigen::Matrix4f transformation_matrix_targ = Eigen::Matrix4f::Identity();  //初始化
	transformation_matrix_targ(0, 0) = cos(theta);
	transformation_matrix_targ(0, 1) = -sin(theta);
	transformation_matrix_targ(0, 3) = delta_x;

	transformation_matrix_targ(1, 0) = sin(theta);
	transformation_matrix_targ(1, 1) = cos(theta);
	transformation_matrix_targ(1, 3) = delta_y;

	transformation_matrix_targ(2, 2) = 1;
	transformation_matrix_targ(3, 3) = 1;

	pcl::transformPointCloud(*cloud_targ_temp, *cloud_targ, transformation_matrix_targ);    //执行变换

	float detaAngle = 0.0;
	float icpScore = 10.0;
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();  //初始化

	int ret = register2d(cloud_in, cloud_targ, icpScore, transformation_matrix, detaAngle);
	if (ret == -1)
	{
		transformation_matrix = Eigen::Matrix4f::Identity();
	}

	float scoreTemp= icpScore;
	multiRegistration MR;
	if (icpScore>0.15)    //0.15 
	{
		double icpScore;
		MR.multiStationRegistration(cloud_targ, cloud_in, icpScore);
		if (icpScore<scoreTemp)
		{
			//std::cout << MR.transformation_matrix << std::endl;
			transformation_matrix = MR.transformation_matrix;	
			scoreTemp = icpScore;
		}	
	}
	//MR.print4x4Matrix(transformation_matrix);
	//pcl::transformPointCloud(*cloud_in, *pPointCloudOut, transformation_matrix);    //执行变换
	//结果输出控制台
	//std::cout << "detaX:" << transformation_matrix(0, 3) << std::endl;
	//std::cout << "detaY:" << transformation_matrix(1, 3) << std::endl;
	//std::cout << "detaAngle:" << detaAngle << std::endl;
	//std::cout << "ICPScore:" << scoreTemp << std::endl;
	std::cout << transformation_matrix(0, 3)<<","<< transformation_matrix(1, 3)<<","<< detaAngle << "," << scoreTemp << std::endl;
	return 0;
}
