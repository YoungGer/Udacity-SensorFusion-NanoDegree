/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting
#ifndef RANSAC2D_H_
#define RANSAC2D_H_

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData();

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D();

pcl::visualization::PCLVisualizer::Ptr initScene();

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol);

template<typename PointT>  
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

#endif