/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting
#include "ransac2d.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
  while (maxIterations--) {
    
    // Randomly sample subset and fit line
    std::unordered_set<int> curr_inliers; 
    while (curr_inliers.size() != 2) {
      curr_inliers.insert(rand() % cloud->points.size());
    }

    auto itr = curr_inliers.begin();
    float x1 = cloud->points[*itr].x;
    float y1 = cloud->points[*itr].y;
    itr++;
    float x2 = cloud->points[*itr].x;
    float y2 = cloud->points[*itr].y;

    float a = y1 - y2;
    float b = x2 - x1;
    float c = x1 * y2 - x2 * y1;

    // Measure distance between every point and fitted line
    for (int i = 0; i < cloud->points.size(); ++i)
    {
      if (curr_inliers.count(i) > 0)
        continue;

      float nx = cloud->points[i].x;
      float ny = cloud->points[i].y;
      float dist = fabs(a * nx + b * ny + c) / sqrt(a * a + b * b);

      if (dist < distanceTol)
        curr_inliers.insert(i);
    }

    if (curr_inliers.size() > inliersResult.size())
      inliersResult = curr_inliers;

  }
	
	return inliersResult;

}

template<typename PointT>  
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
  std::unordered_set<int> inliersResult;

  srand(time(NULL));
  
  // TODO: Fill in this function
  while (maxIterations--) {
    
    // Randomly sample subset and fit line
    std::unordered_set<int> curr_inliers; 
    while (curr_inliers.size() != 3) {
      curr_inliers.insert(rand() % cloud->points.size());
    }

    auto itr = curr_inliers.begin();
    float x1 = cloud->points[*itr].x;
    float y1 = cloud->points[*itr].y;
    float z1 = cloud->points[*itr].z;
    itr++;
    float x2 = cloud->points[*itr].x;
    float y2 = cloud->points[*itr].y;
    float z2 = cloud->points[*itr].z;
    itr++;
    float x3 = cloud->points[*itr].x;
    float y3 = cloud->points[*itr].y;
    float z3 = cloud->points[*itr].z;

    // plane params
    float a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
    float b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
    float c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
    float d = - (a*x1 + b*y1 + c*z1);

    // Measure distance between every point and fitted line
    for (int i = 0; i < cloud->points.size(); ++i)
    {
      if (curr_inliers.count(i) > 0)
        continue;

      float nx = cloud->points[i].x;
      float ny = cloud->points[i].y;
      float nz = cloud->points[i].z;
      float dist = fabs(a * nx + b * ny + c * nz + d) / sqrt(a * a + b * b + c * c);

      if (dist < distanceTol)
        curr_inliers.insert(i);
    }

    if (curr_inliers.size() > inliersResult.size())
      inliersResult = curr_inliers;

  }
  
  return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data 2d for line fit
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	// std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

  // Create data 3d for plane fit
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
  std::unordered_set<int> inliers = RansacPlane<pcl::PointXYZ>(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
