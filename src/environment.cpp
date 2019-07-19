/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);

    // get point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = lidar->scan();


    // TODO:: Create point processor
    //ProcessPointClouds<pcl::PointXYZ> point_processor;
    ProcessPointClouds<pcl::PointXYZ>* point_processor = new ProcessPointClouds<pcl::PointXYZ>();

    // segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> seg_result_pair = point_processor->SegmentPlane(input_cloud, 100, 0.2);
    
    // cluster
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_cloud = point_processor->Clustering(seg_result_pair.first, 1.0, 3, 30);

    // RENDER---------------------------
    // render rays
    // renderRays(viewer, lidar->position, input_cloud);

    // render point clouds
    // renderPointCloud(viewer, input_cloud, "input_cloud");

    // render obstacle and plane
    //renderPointCloud(viewer, seg_result_pair.first, "obstacle", Color(1,0,0));
    renderPointCloud(viewer, seg_result_pair.second, "plane", Color(1,1,0));

    // render cluster
    int cluster_id = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster: clusters_cloud)
    {
        // render cluster point cloud
        renderPointCloud(viewer, cluster, "obstacle_cloud "+std::to_string(cluster_id), colors[cluster_id]);

        // render box
        Box box = point_processor->BoundingBox(cluster);
        renderBox(viewer, box, cluster_id, colors[cluster_id], 1);

        cluster_id++;
    }
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // processor
    ProcessPointClouds<pcl::PointXYZI>* point_processor = new ProcessPointClouds<pcl::PointXYZI>();

    // load cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud = point_processor->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    // render cloud
    renderPointCloud(viewer, input_cloud, "input_cloud");
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    
    // simpleHighway
    // simpleHighway(viewer);

    // load pcd data
    cityBlock(viewer);    

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}