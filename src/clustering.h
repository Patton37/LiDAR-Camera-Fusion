/*
<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
<  LiDAR CR/IR GUI 11/1/2023 
<   
<       AUTHOR: Drew Patton, Noah Wilson  
<       DATE CREATED: 11/1/2023                                                        
<       GROUP: Avanced Technology
<       PROJECT: LiDAR for L2 Applications CR/IR 
<       CUSTOMER: Dan Heafs 
<       DESCRIPTION: 
<		This is the header file of cluster.cpp
<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
*/
#ifndef CLUSTERING_H
#define CLUSTERING_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <opencv2/opencv.hpp>

double calculateDistance(const cv::Point3f& point1, const cv::Point3f& point2);
int assignToClusters(const std::vector<cv::Point3f>& points, const std::vector<cv::Point3f>& centroids);
void updateCentroids(std::vector<cv::Point3f>& centroids, const std::vector<cv::Point3f>& points, const std::vector<int>& assignments); 
void cluster();

#endif // CLUSTERING_H

