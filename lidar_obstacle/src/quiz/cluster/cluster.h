/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting
#ifndef CLUSTER_H
#define CLUSTER_H

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area

pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom);

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points);

void render2DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth);

void ClusterHelper(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int curr_index, std::vector<bool>& processed, std::vector<int>& cluster);

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);

#endif