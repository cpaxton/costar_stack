#include <iostream>
#include <boost/filesystem.hpp>

#include "sp_segmenter/semantic_segmentation.h"

int main(int argc, char** argv)
{
    // Setting up the point cloud segmenter.
    SemanticSegmentation segmenter;

// -------------------------------------------------------------------------
// Setting up segmenter's parameters
    std::cerr << "Setting up the segmenter parameters.\n";
    
    boost::filesystem::path full_path( boost::filesystem::current_path() );
    std::cout << "Current path is : " << full_path << std::endl;

    segmenter.setDirectorySHOT("./data/UW_shot_dict");
    segmenter.setUseMultiClassSVM(true);
    segmenter.setUseBinarySVM(false);
    segmenter.setDirectorySVM("./data/link_node_svm");
    segmenter.setPointCloudDownsampleValue(0.003);
    segmenter.setHierFeaRatio(3);
    segmenter.setUseVisualization(true);
    segmenter.setUseCropBox(true);
    segmenter.setCropBoxSize(0.35,0.35,0.35);
    Eigen::Quaternionf base_rotation(0.316470, 0.647486, 0.629978, -0.289372);
    Eigen::Affine3f table_pose_relative_to_camera = Eigen::Translation3f(0.049957, 0.085419, 0.986988) * base_rotation;
    segmenter.setCropBoxPose(table_pose_relative_to_camera);

#ifdef USE_OBJRECRANSAC
    segmenter.setUseComputePose(true);
    segmenter.setUseCuda(true);
    segmenter.setModeObjRecRANSAC(STANDARD_RECOGNIZE);
    segmenter.setUseObjectPersistence(true);
    segmenter.setUsePreferredOrientation(false);
    segmenter.setPreferredOrientation(base_rotation);
    segmenter.setMinConfidenceObjRecRANSAC(0.15);
    ModelObjRecRANSACParameter node_param(0.05,0.004,0.1,0.1),
        link_param(0.122,0.004,0.1,0.1);
    segmenter.addModelSymmetricProperty("link_uniform", 180, 180, 90, 90, "z");
    segmenter.addModelSymmetricProperty("node_uniform", 90, 90, 90, 90, "z");
    // The order of adding mesh should be consistent to the name of svm.
    // For example, in this sample code, the svm file is link_node_svm, so link should be added before node)
    segmenter.addModel("./data/mesh", "link_uniform", link_param);
    segmenter.addModel("./data/mesh", "node_uniform", node_param);
#endif
    bool use_table = true;
    if (use_table)
    {
        segmenter.setUseTableSegmentation(true);
        segmenter.setCropAboveTableBoundary(0.01, 0.5);
        segmenter.setTableSegmentationParameters(0.02,2.0,7500);

        bool load_table_from_file = false;

        if (load_table_from_file)
            segmenter.loadTableFromFile("./sample_pcd_data/table.pcd");
        else
        {
            std::cerr << "Generating table point cloud segmentation.\n";
            // generate new table
            pcl::PCDReader reader;
            pcl::PointCloud<PointT>::Ptr raw_unsegmented_table_points(new pcl::PointCloud<PointT>);
            if( reader.read ("./sample_pcd_data/raw_unsegmented_table.pcd", *raw_unsegmented_table_points) == 0){
                segmenter.getTableSurfaceFromPointCloud(raw_unsegmented_table_points);
            }
            else
                std::cerr << "Generating table from raw input point cloud failed: File not found.\n";
        }
    }

// -------------------------------------------------------------------------
// Initialize the segmenter with the parameters that has been set
    std::cerr << "Initialize segmenter based on input parameters.\n";
    segmenter.initializeSemanticSegmentation();

// Do point cloud segmentation and calculate its pose
    pcl::PCDReader reader;
    pcl::PointCloud<PointT>::Ptr raw_unsegmented_points_that_contains_objects(new pcl::PointCloud<PointT>);
    std::cerr << "Reading unsegmented point cloud that contains object.\n";
    if( reader.read ("./sample_pcd_data/raw_sample_point_cloud.pcd", *raw_unsegmented_points_that_contains_objects) == 0)
    {
        pcl::PointCloud<PointLT>::Ptr segmented_point_cloud_labels;
        if (segmenter.segmentPointCloud(raw_unsegmented_points_that_contains_objects, segmented_point_cloud_labels))
        {
            std::cerr << "Object point cloud segmentation is successful.\n";
#ifdef USE_OBJRECRANSAC
            std::vector<ObjectTransformInformation> pose_estimation = segmenter.calculateObjTransform(segmented_point_cloud_labels);
            if (pose_estimation.size() > 0)
            {
                std::cerr << "Object transform calculated successfully.\n";
                for (std::vector<ObjectTransformInformation>::const_iterator it = pose_estimation.begin(); it!=pose_estimation.end(); ++it)
                {
                    std::cerr << *it;
                }
            }
#endif
        }
    }



    return 0;
} 
