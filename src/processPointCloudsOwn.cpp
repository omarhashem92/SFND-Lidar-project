#include "processPointCloudsOwn.h"
#include "processPointClouds.cpp"

template<typename PointT>
std::vector<float> ProcessPointCloudsOwn<PointT>::Ransac::findLineCoefficents(float x1, float y1, float z1,
 float x2, float y2, float z2, float x3, float y3, float z3)
 {
     //plane equation Ax + By + Cz + D = 0
	float a = (y2-y1) * (z3-z1) - (z2-z1) * (y3-y1);
	float b = (z2-z1) * (x3-x1) - (x2-x1) * (z3-z1);
	float c = (x2-x1) * (y3-y1) - (y2-y1) * (x3-x1);
	float d = -(a * x1 + b * y1 + c * z1);
	std::vector<float> lineCoefficents = {a, b, c, d};
	return lineCoefficents;
 }

 //plane coefficents are a, b, c, d
//point to compare the distance with the line is defined by: (x, y, z)
template<typename PointT>
float ProcessPointCloudsOwn<PointT>::Ransac::getPointDistance(float a, float b, float c, float d, float x, float y, float z)
{
	//apply the equation: d = |Ax + By + Cz + D| / sqrt(A^2 + B^2 + C^2)
	float dist = fabs(a * x + b * y + c * z + d) / sqrt(a * a + b * b + c * c);
	return dist;
}

template<typename PointT>
std::unordered_set<int> ProcessPointCloudsOwn<PointT>::Ransac::ransac(typename pcl::PointCloud<PointT>::Ptr cloud,
 int maxIterations,float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// For max iterations 
	float point1Idx, point2Idx, point3Idx, maxCountPointsOnLine = 0;
	float bestPoint1Idx = 0, bestPoint2Idx = 0, bestPoint3Idx = 0;
	for(int j = 0; j < maxIterations; ++j)
	{
		// Randomly sample subset and fit line
		point1Idx = rand() % cloud->points.size();
		point3Idx = point2Idx = point1Idx;
		while(point1Idx == point2Idx)
			point2Idx = rand() % cloud->points.size(); 
		
		while(point3Idx == point2Idx || point3Idx == point1Idx)
			point3Idx = rand() % cloud->points.size(); 

		PointT point1 = cloud->points[point1Idx];
		PointT point2 = cloud->points[point2Idx];
		PointT point3 = cloud->points[point3Idx];
		std::vector<float> lineCoeffs = findLineCoefficents(point1.x, point1.y, point1.z,
		 point2.x, point2.y, point2.z , point3.x, point3.y, point3.z);
		float a = lineCoeffs[0];
		float b = lineCoeffs[1];
		float c = lineCoeffs[2];
		float d = lineCoeffs[3];

		float countPointsOnLine = 0;
		
		for(int i = 0; i < cloud->points.size(); ++i)
		{
			PointT aPoint = cloud->points[i];
			float x = aPoint.x, y = aPoint.y, z = aPoint.z;
			float dist = getPointDistance(a, b, c, d, x, y, z);
			if(dist < distanceTol)
			{
				countPointsOnLine++;
			}
		}
		if(countPointsOnLine > maxCountPointsOnLine)
		{
			maxCountPointsOnLine = countPointsOnLine;
			bestPoint1Idx = point1Idx;
			bestPoint2Idx = point2Idx;
			bestPoint3Idx = point3Idx;
		}
	}
	
	// Return indicies of inliers from fitted line with most inliers
	PointT bestPoint1 = cloud->points[bestPoint1Idx];
	PointT bestPoint2 = cloud->points[bestPoint2Idx];
	PointT bestPoint3 = cloud->points[bestPoint3Idx];
	std::vector<float> bestLineCoeffs = findLineCoefficents(bestPoint1.x, bestPoint1.y, bestPoint1.z,
	  bestPoint2.x, bestPoint2.y, bestPoint2.z, bestPoint3.x, bestPoint3.y, bestPoint3.z);
	float bestA = bestLineCoeffs[0];
	float bestB = bestLineCoeffs[1];
	float bestC = bestLineCoeffs[2];
	float bestD = bestLineCoeffs[3];
	for(int i = 0; i < cloud->points.size(); ++i)
	{
		PointT aPoint = cloud->points[i];
		float x = aPoint.x, y = aPoint.y, z = aPoint.z;
		float dist = getPointDistance(bestA, bestB, bestC, bestD, x, y, z);
		if(dist < distanceTol)
		{
			inliersResult.insert(i);
		}
	}
	return inliersResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointCloudsOwn<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, 
    float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    std::unordered_set<int> inliers = Ransac::ransac(cloud, maxIterations, distanceThreshold);
    if (inliers.size () == 0)
    {
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    static unsigned long long countNumTimesOfExecution = 0;
    static unsigned long long accumelativeTimeTaken = 0;
    countNumTimesOfExecution++;
    accumelativeTimeTaken += elapsedTime.count();
    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
    return std::make_pair(cloudOutliers, cloudInliers); 
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointCloudsOwn<PointT>::Cluster::euclideanCluster(
    const std::vector<std::vector<float>>& points, float distanceTol, int minSize, int maxSize)
{
	std::vector<std::vector<int>> clusters;
	std::vector<bool> isProcessed(points.size(), false);
	for(int i = 0; i < points.size(); ++i)
	{
		auto point = points[i];
		if(!isProcessed[i])
		{
			std::vector<int> cluster;
			proximity(i, cluster, isProcessed, points, distanceTol);
            if(cluster.size() >= minSize && cluster.size() <= maxSize)
			    clusters.push_back(cluster);
		}
	}
	return clusters;
}

template<typename PointT>
void ProcessPointCloudsOwn<PointT>::Cluster::proximity(int idx, std::vector<int>& cluster, std::vector<bool>& isProcessed,
 const std::vector<std::vector<float>>& points, float distanceTol)
{
	isProcessed[idx] = true;
	cluster.push_back(idx);
	auto nearbyPointsIndicies = tree->search(points[idx], distanceTol);
	for(auto nearbyPointIdx : nearbyPointsIndicies)
	{
		if(!isProcessed[nearbyPointIdx])
		{
			proximity(nearbyPointIdx, cluster, isProcessed, points, distanceTol);
		}
	}
}

template<typename PointT>
void ProcessPointCloudsOwn<PointT>::Cluster::insertPointsInKdTree(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    for (int i=0; i<cloud->points.size(); i++) 
    {
        std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
    	this->tree->insert(point,i);
    }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointCloudsOwn<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;//the return of the function that will contain (n) elements, where (n) is the number of clusters. Each element contains the cloud-points which are related to the cluster

    // Creating the KdTree object for the search method of the extraction
    /*typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);//create a Kdtree object to find the nearest neighbours in log(n)
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;//This vector will contain (n) elements, where (n) is the number of clusters. Each element contains the cloud-indicies which are related to that cluster
    pcl::EuclideanClusterExtraction<PointT> ec;//use euclidean distance method for finding distance between points
    ec.setClusterTolerance (clusterTolerance);//The distance between points to be considered in the same cluster
    ec.setMinClusterSize (minSize);//minimum cluster size to avoid taking noise as clusters
    ec.setMaxClusterSize (maxSize);//maximum cluster size to avoid taking 2 clusters as 1 big cluster
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);//fill the cluster indices vector*/
    Cluster clusterProcessor;
    clusterProcessor.insertPointsInKdTree(cloud);
    
    std::vector<std::vector<float>> points;
    for(auto point : cloud->points)
    {
        std::vector<float> aPoint = {point.x, point.y, point.z/*, point.intensity */};
        points.push_back(aPoint);
    }
    std::vector<std::vector<int>> clustersIndices = clusterProcessor.euclideanCluster(points, clusterTolerance, minSize,
     maxSize);
    for(std::vector<int> cluster : clustersIndices)
  	{
  		typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
  		for(int a_point_idx : cluster)
  			clusterCloud->points.push_back(cloud->points[a_point_idx]);
        clusters.push_back(clusterCloud);
  	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    
    static unsigned long long countNumTimesOfExecution = 0;
    static unsigned long long accumelativeTimeTaken = 0;
    countNumTimesOfExecution++;
    accumelativeTimeTaken += elapsedTime.count();
    
    return clusters;
}
