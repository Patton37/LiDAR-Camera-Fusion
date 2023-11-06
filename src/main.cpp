#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <opencv2/opencv.hpp>

#include "clustering.h"
int main() {
	
	
	// double dist = 0.0; 
    cv::Point3f pointA, pointB, pointC, D,E,F;
	pointA = cv::Point3f(5.0, 5.0, 5.0);
	pointB = cv::Point3f(1.0, 1.0, 1.0);
	pointC = cv::Point3f(20.0, 20.0, 20.0); 
	D = cv::Point3f(0.0, 0.0, 0.0);
	E = cv::Point3f(7.0, 7.0, 7.0);
	F = cv::Point3f(25.0, 25.0, 25.0);

    // dist = calculateDistance(pointA,pointB);
	// std::cout << "Distance: "  << dist << std::endl;

	int clusters; 
	std::vector<cv::Point3f> points; 
	std::vector<cv::Point3f> centroids; 

	points.push_back(pointA); 
	points.push_back(pointB); 
	points.push_back(pointC);

	centroids.push_back(D); 
	centroids.push_back(E); 
	centroids.push_back(F);  
	
	// clusters = assignToClusters(points, centroids);
	// std::cout << "Number of Clusters: " << clusters << std::endl;

	std::vector<int> assignments(points.size());

	updateCentroids(centroids, points, assignments)
	return 0; 
}
