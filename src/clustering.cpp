/*
<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
<  LiDAR CR/IR GUI 11/1/2023 
<   
<       AUTHOR: Drew Patton
<       DATE CREATED: 11/1/2023                                                        
<       GROUP: Avanced Technology
<       PROJECT: LiDAR for L2 Applications CR/IR 
<       CUSTOMER: Dan Heafs 
<       DESCRIPTION: 
<		This program contains a single function that reads pointcloud data from a .csv <		file and finds clusters in it. This code is based on Noah Wilson's code written <		in python.  
<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
*/

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <opencv2/opencv.hpp>

/*
FUNCTION: assignToCluster
Author: Drew Patton 
Description: 
    This function calculates distacnce between 3D points. 
*/
double calculateDistance(const cv::Point3f& point1, const cv::Point3f& point2) {
    cv::Point3f diff = point1 - point2;
    return cv::norm(diff);
}

/*
FUNCTION: assignToCluster
Author: Drew Patton 
Description: 
    This function takes a vector of cv::Point3f points (3D points) for the centroids as well as another one for the points themselves and assigns them a cluster 
    based on distance to the nearest centroid. The function returns the number of clusters.
*/
int assignToClusters(const std::vector<cv::Point3f>& points, const std::vector<cv::Point3f>& centroids) {
    int numClusters = centroids.size();
    int numPoints = points.size();
    std::vector<int> assignments(numPoints);

    for (int i = 0; i < numPoints; i++) {
        double minDistance = calculateDistance(points[i], centroids[0]);
        int clusterIndex = 0;

        for (int j = 1; j < numClusters; j++) {
            double distance = calculateDistance(points[i], centroids[j]);
            if (distance < minDistance) {
                minDistance = distance;
                clusterIndex = j;
            }
        }

        assignments[i] = clusterIndex;
    }

    return numClusters;
}


/*
FUNCTION: Update Centroids 
Author: Drew Patton 
Description: 
    This function updates centroids based on the new points as well as their assignments. 
*/
void updateCentroids(std::vector<cv::Point3f>& centroids, const std::vector<cv::Point3f>& points, const std::vector<int>& assignments) {
    int numClusters = centroids.size();
    int numPoints = points.size();
    std::vector<cv::Point3f> newCentroids(numClusters, cv::Point3f(0.0, 0.0, 0.0));
    std::vector<int> clusterSizes(numClusters, 0);

    for (int i = 0; i < numPoints; i++) {
        int clusterIndex = assignments[i];
        newCentroids[clusterIndex] += points[i];
        clusterSizes[clusterIndex]++;
    }

    for (int i = 0; i < numClusters; i++) {
        if (clusterSizes[i] > 0) {
            newCentroids[i] /= static_cast<float>(clusterSizes[i]);
        }
    }

    centroids = newCentroids;
}

/*
FUNCTION: Update Centroids 
Author: Drew Patton 
Description: 
    This function serves as the program head for the clustering algorithm. If clustering is to be implemented from another program, 
    then this function should be invoked. An input file is defined as the source of the points. Additionally, the maximum number of clusters can be set here. 
*/
void cluster() {
    int numClusters = 3; //The maximum number of clusters 


    //INPUT FILE:
    std::ifstream input("capture.csv");
    if (!input) {
        std::cerr << "Failed to open the input file." << std::endl;
        return;
    }

    //Create a vector of 3D points
    std::vector<cv::Point3f> points; 
    std::string line;

    while (std::getline(input, line)) {
        std::istringstream iss(line);
        float x, y, z;
        if (iss >> x >> y >> z) {
            points.push_back(cv::Point3f(x, y, z));
        }
    }
    
    //Create a vector of 3D points for the centroids: 
    std::vector<cv::Point3f> centroids(numClusters); 
    centroids[0] = points[0]; //Initialize the first centroid with the first point (MIGHT CHANGE)

    for (int i = 1; i < numClusters; i++) {
        std::vector<float> maxDistances;
        for (const auto& point : points) {
            double minDistance = std::numeric_limits<double>::max();
            for (int j = 0; j < i; j++) {
                double distance = calculateDistance(point, centroids[j]);
                minDistance = std::min(minDistance, distance);
            }
            maxDistances.push_back(static_cast<float>(minDistance));
        }
        // Select the point with the highest minimum distance as the next centroid
        int nextCentroidIndex = std::distance(maxDistances.begin(), std::max_element(maxDistances.begin(), maxDistances.end()));
        centroids[i] = points[nextCentroidIndex];
    }

    // K-means clustering
    int maxIterations = 100;
    for (int iter = 0; iter < maxIterations; iter++) {
        std::vector<int> assignments(points.size());
        int numClustersAssigned = assignToClusters(points, centroids);

        if (numClustersAssigned != numClusters) {
            std::cerr << "Clustering resulted in fewer clusters than requested. Retrying with different initial centroids." << std::endl;
            break;
        }

        std::vector<cv::Point3f> prevCentroids = centroids;
        updateCentroids(centroids, points, assignments);

        // Check for convergence (when centroids do not change significantly)
        bool converged = true;
        for (int i = 0; i < numClusters; i++) {
            if (calculateDistance(centroids[i], prevCentroids[i]) > 1e-6) {
                converged = false;
                break;
            }
        }

        if (converged) {
            break;
        }
    }

    // Print the number of clusters and centroids
    std::cout << "Number of clusters: " << numClusters << std::endl;
    std::cout << "Centroids:" << std::endl;
    for (int i = 0; i < numClusters; i++) {
        std::cout << "Cluster " << i + 1 << ": (" << centroids[i].x << ", " << centroids[i].y << ", " << centroids[i].z << ")" << std::endl;
    }

    
    //OPTIONAL PLOTTING 
}



