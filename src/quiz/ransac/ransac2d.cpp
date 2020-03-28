/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include "math.h"
#include <stdlib.h>
#include <iostream>


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
	std::unordered_set<int> bestResult;
	srand(time(NULL));
	// TODO: Fill in this function
	// For max iterations 
	for (int it=0; it<maxIterations; it++){
		int num_points = cloud->points.size();
		// Randomly sample subset and fit line
		int idx0 = 0;
		int idx1 = 0;
		int idx2 = 0;
		while ((idx0 == idx1) || (idx0 == idx2) || (idx1 == idx2)){
			idx0 = rand() % num_points;
			idx1 = rand() % num_points;
			idx2 = rand() % num_points;
		}

		pcl::PointXYZ point0 = cloud->points[idx0];
		pcl::PointXYZ point1 = cloud->points[idx1];
		pcl::PointXYZ point2 = cloud->points[idx2];
		auto a = (point1.y - point0.y) * (point2.z - point0.z) - (point1.z - point0.z) * (point2.y - point0.y);
		auto b = (point1.z - point0.z) * (point2.x - point0.x) - (point1.x - point0.x) * (point2.z - point0.z);
		auto c = (point1.x - point0.x) * (point2.y - point0.y) - (point1.y - point0.y) * (point2.x - point0.x);
		auto d = - (a * point0.x + b * point0.y+ c * point0.z) ;
		auto denom = sqrt(a*a + b*b + c*c);

		std::unordered_set<int> tmpResult;
		for (int index=0; index<num_points; index++){
			// Measure distance between every point and fitted line
			auto p = cloud->points[index];
			auto dist = (fabs(a * p.x + b * p.y + c * p.z + d) / denom);
			// If distance is smaller than threshold count it as inlier
			if (dist < distanceTol){
				tmpResult.insert(index);
			}
		}

		if (tmpResult.size() > bestResult.size()){
			bestResult = tmpResult;
		}
	}
	// Return indicies of inliers from fitted line with most inliers
	return bestResult;
}

// std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
// {
// 	std::unordered_set<int> bestResult;
// 	srand(time(NULL));
// 	// TODO: Fill in this function
// 	// For max iterations 
// 	for (int it=0; it<maxIterations; it++){
// 		int num_points = cloud->points.size();
// 		// Randomly sample subset and fit line
// 		int idx0 = 0;
// 		int idx1 = 0;
// 		while (idx0 == idx1){
// 			idx0 = rand() % num_points;
// 			idx1 = rand() % num_points;
// 		}

// 		pcl::PointXYZ point0 = cloud->points[idx0];
// 		pcl::PointXYZ point1 = cloud->points[idx1];
// 		auto a = point0.y - point1.y;
// 		auto b = point1.x - point0.x;
// 		auto c = point0.x * point1.y - point1.x * point0.y;
// 		auto denom = sqrt(a*a + b*b);

// 		std::unordered_set<int> tmpResult;
// 		for (int index=0; index<num_points; index++){
// 			// Measure distance between every point and fitted line
// 			auto p = cloud->points[index];
// 			auto d = (fabs(a * p.x + b * p.y + c) / denom);
// 			// If distance is smaller than threshold count it as inlier
// 			if (d < distanceTol){
// 				tmpResult.insert(index);
// 			}
// 		}

// 		if (tmpResult.size() > bestResult.size()){
// 			bestResult = tmpResult;
// 		}
// 	}
// 	// Return indicies of inliers from fitted line with most inliers
// 	return bestResult;
// }

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

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
