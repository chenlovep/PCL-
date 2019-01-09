#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp> //pcl中的点云类型
#include <pcl/filters/statistical_outlier_removal.h>//过滤离散点
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include "cv.h"
#include <iostream>
#include <sstream>

using namespace pcl;

template <class PointT>
bool readPointCloud(const std::string &fileName, pcl::PointCloud<PointT> &outPointCloud)
{
	string _format = fileName.substr(fileName.length() - 3, 3);
	if (_format == "ply")
	{
		if (pcl::io::loadPLYFile<PointT>(fileName, outPointCloud) < 0)
		{
			pcl::console::print_error("Error loading pointCloud file!\n");
			return false;
		}
	}
	else if (_format == "pcd")
	{
		if (pcl::io::loadPCDFile(fileName, outPointCloud) < 0)
		{
			pcl::console::print_error("Error loading pointCloud file!\n");
			return false;
		}
	}
	else
	{
		cout << "file format error!" << endl;
		return false;
	}
	return true;
}

//打印4*4变换矩阵
//print 4*4 matrix
inline void print4x4Matrix(const Eigen::Matrix4d & matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
	return;
}

template <class PointT>
bool pointCloudICP(pcl::PointCloud<PointT> PointSource, pcl::PointCloud<PointT> PointTarget, pcl::PointCloud<PointT> &outPointCloud)
{
	return false;
}

template <class PointT>
bool addPointCloud(pcl::visualization::PCLVisualizer &viewer, typename pcl::PointCloud<PointT>::Ptr pointCloud, const std::string pcloudID, cv::Scalar &color, int size, bool holdon)  //为什么加了typename就可以了？？？？？
{
	if (!holdon)
	{
		viewer.removeAllPointClouds();
	}
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(pointCloud, (int)color[0], (int)color[1], (int)color[2]);
	viewer.addPointCloud(pointCloud, cloud_in_color_h, pcloudID);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, pcloudID);
	//viewer.updatePointCloud(pointCloud, pcloudID);//加上这句后什么都没有
	//viewer.spin();
	return true;
}

//利用统计分析过滤离群点
template <class PointT>
void filterOutlier(typename PointCloud<PointT>::Ptr &srcPointCloud)
{
	//基本原理，对点云所有点计算与最近点的K个点的距离的均值，对所有的点的距离均值求平均值和标准差，确定全局均值和标准差并以
	//mean + std_mul_ * stddev 作为距离阈值distance_threshold，其中 std_mul_为标准差的放大倍数。将大于distance_threshold判定为离散点。
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(srcPointCloud);
	sor.setMeanK(20);//Set the number of nearest neighbors to use for mean distance estimation.
	sor.setStddevMulThresh(1.0);
	sor.filter(*srcPointCloud);
	return;
}

//计算点云的分辨率，对于存在较多离散点的场景，应先对离散点剔除。
template <class PointT>
double computeCloudResolution(typename PointCloud<PointT>::ConstPtr srcPointCloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<PointT> tree;
	tree.setInputCloud(srcPointCloud);
	for (size_t i = 0; i < srcPointCloud->size(); ++i)
	{
		if (!pcl_isfinite((*srcPointCloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		//函数说明：1. 为所要查询的点的索引； 2.为K领域的个数，=1时表示查询点自己；3.为搜索完的领域点对应的索引； 4.为领域点到查询点的欧式距离
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}


//对点云进行下采样
template <class PointT>
bool downSamplePointCloud(typename pcl::PointCloud<PointT>::Ptr &srcPointCloud, const double relSamplingDistance,
	typename pcl::PointCloud<PointT>::Ptr &outCloud, const int method)
{
	//用uniformSampling对点云进行下采样
	if (method == 1)
	{
		pcl::UniformSampling<PointT> uniform_sampling;
		uniform_sampling.setInputCloud(srcPointCloud);
		uniform_sampling.setRadiusSearch(relSamplingDistance);
		uniform_sampling.filter(*outCloud);
		return true;
	}
	//用voxelGraid进行下采样
	if (method == 2)
	{
		pcl::VoxelGrid<PointT> sor;
		sor.setInputCloud(srcPointCloud);
		sor.setLeafSize(relSamplingDistance, relSamplingDistance, relSamplingDistance);
		sor.filter(*outCloud);
		return true;
	}
	//通过实验发现，方法一，是对原样本点进行采样，而方法二则是通过体素计算重心法进行采样，采样效率上，方法2更快，大概是方法一的6倍
	return true;
}
