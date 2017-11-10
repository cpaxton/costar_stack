#include "object_on_table_segmenter/plane_segmenter.h"

CloudXYZRGBA::Ptr PlaneSegmenter::cropBox(const CloudXYZRGBA &input_cloud, const Eigen::Affine3f &box_pose, const Eigen::Vector3f &box_size) const
{
	CloudXYZRGBA::Ptr cropped_cloud(new CloudXYZRGBA());
	pcl::CropBox<pcl::PointXYZRGBA> crop_box;
	bool is_organized = input_cloud.isOrganized();

	crop_box.setKeepOrganized(is_organized);
	crop_box.setInputCloud(input_cloud.makeShared());
	crop_box.setMax(box_size.homogeneous());
	crop_box.setMin((-box_size).homogeneous());
	crop_box.setTransform(box_pose.inverse());
	crop_box.filter(*cropped_cloud);

	if (visualize_)
	{
		viewer_->setWindowName("CropBox Result");
		viewer_->removeAllPointClouds();
		viewer_->addPointCloud(cropped_cloud, "cropbox");
		viewer_->spin();
		viewer_->removeAllPointClouds();
	}

	return cropped_cloud;
}

void PlaneSegmenter::organizedPlaneSegmentation(
	const CloudXYZRGBA &input_cloud,
	std::vector< pcl::PlanarRegion<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGBA> > > &regions,
	pcl::PointCloud<pcl::Label>::Ptr &labels,
	std::vector<pcl::PointIndices> &label_indices) const
{
	// Get Normal Cloud
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(10.0f);
	CloudXYZRGBA::Ptr input_cloud_ptr = input_cloud.makeShared();
	ne.setInputCloud(input_cloud_ptr);
	ne.compute(*normals);

	// Segment planes
	pcl::OrganizedMultiPlaneSegmentation< pcl::PointXYZRGBA, pcl::Normal, pcl::Label > mps;

	mps.setMinInliers (minimal_inliers_);
	mps.setAngularThreshold (0.017453 * angular_threshold_); // in radians
	mps.setDistanceThreshold (distance_threshold_); // in meters
	mps.setInputNormals (normals);
	mps.setInputCloud (input_cloud_ptr);
	mps.setProjectPoints (true); // project the boundary points to the plane
	std::vector<pcl::ModelCoefficients> model_coefficients;
	std::vector<pcl::PointIndices> inlier_indices;
	std::vector<pcl::PointIndices> boundary_indices;

	mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
}

CloudXYZRGBA::Ptr PlaneSegmenter::generatePlaneConvexHull(const CloudXYZRGBA &input_cloud) const
{
	std::vector< pcl::PlanarRegion<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGBA> > > regions;
	pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>());
	std::vector<pcl::PointIndices> label_indices;
	this->organizedPlaneSegmentation(input_cloud,regions,labels,label_indices);

	// Retrieve the convex hull.
	CloudXYZRGBA::Ptr convex_hull(new CloudXYZRGBA());
	unsigned int best_region = 0;
	unsigned int best_region_idx = 0;

	if (regions.size() > 0) {
		std::cout << "Number of possible plane regions: " << regions.size() << std::endl;

		for (size_t i = 0; i < regions.size(); i++){
			std::cout << "Region" << i <<" size: " << regions[i].getCount() << std::endl;
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr contour (new pcl::PointCloud<pcl::PointXYZRGBA>);
			contour->points = regions[i].getContour();

			if (regions[i].getCount() > best_region)
			{
				best_region = regions[i].getCount();
				best_region_idx = i;
			}

			if (visualize_)
			{
				std::stringstream ss;
				ss << "Region" << i;
				viewer_->setWindowName(ss.str());
				viewer_->removeAllPointClouds();
				viewer_->addPointCloud(contour, ss.str());
				viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, ss.str());
				viewer_->spin();
				viewer_->removeAllPointClouds();
			}
		} 

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr boundary(new pcl::PointCloud<pcl::PointXYZRGBA>);
		boundary->points = regions[best_region_idx].getContour();
		
		pcl::ConvexHull<pcl::PointXYZRGBA> hull;
		hull.setInputCloud(boundary);
		// Make sure that the resulting hull is bidimensional.
		hull.setDimension(2);
		hull.reconstruct(*convex_hull);
	}
	return convex_hull;
}

std::vector<CloudXYZRGBA::Ptr> PlaneSegmenter::clusterPointCloud(const CloudXYZRGBA &input_cloud, const unsigned int &minimal_cluster_size)
{
	CloudXYZRGBA::Ptr input_cloud_ptr = input_cloud.makeShared();
	std::vector< pcl::PlanarRegion<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGBA> > > regions;
	pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>());
	std::vector<pcl::PointIndices> label_indices;
	this->organizedPlaneSegmentation(input_cloud,regions,labels,label_indices);

	std::vector<bool> plane_labels;
	plane_labels.resize(label_indices.size(), false);

	pcl::EuclideanClusterComparator<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator(
		new pcl::EuclideanClusterComparator<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>());
	euclidean_cluster_comparator->setInputCloud (input_cloud_ptr);
	euclidean_cluster_comparator->setLabels (labels);
	euclidean_cluster_comparator->setExcludeLabels (plane_labels);
	euclidean_cluster_comparator->setDistanceThreshold(0.03f, false);

	pcl::PointCloud<pcl::Label> euclidean_labels;
	std::vector<pcl::PointIndices> euclidean_label_indices;
	pcl::OrganizedConnectedComponentSegmentation<pcl::PointXYZRGBA,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator);
	euclidean_segmentation.setInputCloud (input_cloud_ptr);
	euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);

	std::vector<CloudXYZRGBA::Ptr> result_cluster;
	for (std::size_t i = 0; i < euclidean_label_indices.size (); i++)
	{
		if (euclidean_label_indices[i].indices.size() > minimal_cluster_size)
		{
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>());
			pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

			pcl::PointIndices::Ptr object_cloud_indices (new pcl::PointIndices());
			*object_cloud_indices = euclidean_label_indices[i];
			extract.setInputCloud(input_cloud_ptr);
			extract.setIndices(object_cloud_indices);
			extract.setKeepOrganized(true);
			extract.filter(*cloud_cluster);
			result_cluster.push_back(cloud_cluster);

			std::cerr << "\tCluster size: "<< euclidean_label_indices[i].indices.size() << std::endl;
		}
	}
	return result_cluster;
}


CloudXYZRGBA::Ptr PlaneSegmenter::segmentAbovePlane(const CloudXYZRGBA &input_cloud, const CloudXYZRGBA &plane_convex_hull,
	const double &min_above_plane, const double &max_above_plane) const
{
	CloudXYZRGBA::Ptr input_cloud_ptr = input_cloud.makeShared();
	if (plane_convex_hull.empty())
	{
		std::cerr << "Error, the plane convex hull is still empty.\nReturning unsegmented input point cloud\n";
		return input_cloud_ptr;
	}
	std::cout << "\nSegment object above plane \n";
	bool is_organized = input_cloud.isOrganized();

	// Prism object.
	pcl::ExtractPolygonalPrismData<pcl::PointXYZRGBA> prism;
	prism.setInputCloud(input_cloud_ptr);
	prism.setInputPlanarHull(plane_convex_hull.makeShared());

	// from 1 cm above plane to 50 cm above plane
	std::cerr << "Segmentation range: " << min_above_plane << " to " << max_above_plane << std::endl;
	prism.setHeightLimits(min_above_plane, max_above_plane);
	pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices());

	prism.segment(*objectIndices);
	// Get and show all points retrieved by the hull.
	CloudXYZRGBA::Ptr objects(new CloudXYZRGBA());
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	extract.setInputCloud(input_cloud_ptr);
	extract.setIndices(objectIndices);
	
	extract.setKeepOrganized(is_organized);

	extract.filter(*objects);
	return objects;
}

CloudXYZRGBA::Ptr PlaneSegmenter::segmentAbovePlane(const CloudXYZRGBA &input_cloud, const double &min_above_plane, const double &max_above_plane) const
{
	return this->segmentAbovePlane(input_cloud, *plane_convex_hull_, min_above_plane, max_above_plane);
}

CloudXYZRGBA::Ptr PlaneSegmenter::segmentAbovePlane(const CloudXYZRGBA &input_cloud, const CloudXYZRGBA &plane_convex_hull) const
{
	return this->segmentAbovePlane(input_cloud, plane_convex_hull, min_above_plane_, max_above_plane_);
}

CloudXYZRGBA::Ptr PlaneSegmenter::segmentAbovePlane(const CloudXYZRGBA &input_cloud) const
{
	return this->segmentAbovePlane(input_cloud, *plane_convex_hull_, min_above_plane_, max_above_plane_);
}

CloudXYZRGBA::Ptr PlaneSegmenter::generatePlaneConvexHullFromPoseXYplane(const Eigen::Affine3f &pose, const double &size) const
{
	CloudXYZRGBA::Ptr plane_points(new CloudXYZRGBA());
	for (int i = -1; i < 2; i+=2)
	{
		for (int j = -1; j < 2; j+=2)
		{
			pcl::PointXYZRGBA tmp;
			tmp.x = i * size/2;
			tmp.y = j * size/2;
			tmp.z = 0;
			plane_points->push_back(tmp);
		}
	}
	CloudXYZRGBA::Ptr plane_hull(new CloudXYZRGBA());
	pcl::transformPointCloud (*plane_points, *plane_hull, pose);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convex_hull(new pcl::PointCloud<pcl::PointXYZRGBA>());

	pcl::ConvexHull<pcl::PointXYZRGBA> hull;
	hull.setInputCloud(plane_hull);
	// Make sure that the resulting hull is bidimensional.
	hull.setDimension(2);
	hull.reconstruct(*convex_hull);
	return convex_hull;
}

void PlaneSegmenter::setPlaneSegmentationParameters(const double &distance_threshold, const double &angular_threshold, const unsigned int &minimal_inliers)
{
	this->distance_threshold_ = distance_threshold;
	this->angular_threshold_ = angular_threshold;
	this->minimal_inliers_ = minimal_inliers;
}

void PlaneSegmenter::setPlaneConvexHull(const CloudXYZRGBA &plane_convex_hull)
{
	if (plane_convex_hull.empty())
	{
		std::cerr << "Error, the input convex hull is empty.\n";
		this->ready = false;
		return;
	}
	else
	{
		this->ready = true;
		*plane_convex_hull_ = plane_convex_hull;
		if (visualize_)
		{
			viewer_->setWindowName("Selected Plane Convex Hull");
			viewer_->removeAllPointClouds();
			viewer_->addPointCloud(plane_convex_hull_, "plane_hull");
			viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "plane_hull");
			viewer_->spin();
			viewer_->removeAllPointClouds();
		}
	}
}

void PlaneSegmenter::setPlaneSegmentationHeight(const double &min_above_plane, const double &max_above_plane)
{
	this->min_above_plane_ = min_above_plane;
	this->max_above_plane_ = max_above_plane;
}

void PlaneSegmenter::setVisualizationFlag(const bool visualize)
{
	this->visualize_ = visualize;

	if (visualize && !viewer_)
	{
		viewer_ = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer ("Visualizer"));
		viewer_->initCameraParameters();
		viewer_->addCoordinateSystem(0.1);
		viewer_->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
		viewer_->setSize(1280, 960);
	}
}
