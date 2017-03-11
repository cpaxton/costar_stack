from sp_segmenter.SemanticSegmentationPy import *

segmenter = SemanticSegmentation()
print "Setting up the segmenter parameters."
use_additional_parameter = True
use_table = True
have_objrecransac = True
compute_pose = True and have_objrecransac

ObjRecRANSAC_mode_dict = {"STANDARD_BEST":0, "STANDARD_RECOGNIZE":1, "GREEDY_RECOGNIZE":2}
# Setting up segmenter's parameters

segmenter.setDirectorySHOT("../data/UW_shot_dict")
segmenter.setDirectoryFPFH("../data/UW_sift_dict")
segmenter.setDirectorySIFT("../data/UW_fpfh_dict")

segmenter.setUseSHOT(True)
segmenter.setUseFPFH(False)
segmenter.setUseSIFT(False)

segmenter.setUseMultiClassSVM(True);
segmenter.setUseBinarySVM(False);
segmenter.setDirectorySVM("../data/link_node_svm");
segmenter.setPointCloudDownsampleValue(0.003);
segmenter.setHierFeaRatio(3);
segmenter.setUseVisualization(False);
print "All parameter required for point cloud segmentation has been loaded."

if compute_pose:
    segmenter.setUseComputePose(True);
    segmenter.setUseCuda(False);
    segmenter.setUseCombinedObjRecRANSAC(False)
    segmenter.setModeObjRecRANSAC(ObjRecRANSAC_mode_dict["STANDARD_RECOGNIZE"]);
    segmenter.setMinConfidenceObjRecRANSAC(0.15);
    node_param = ModelObjRecRANSACParameter(0.05,0.004,0.1,0.1);
    link_param  = ModelObjRecRANSACParameter(0.05,0.004,0.1,0.1);
    # The order of adding mesh should be consistent to the name of svm.
    # For example, in this sample code, the svm file is link_node_svm, so link should be added before node)
    segmenter.addModel("../data/mesh", "link_uniform", link_param);
    segmenter.addModel("../data/mesh", "node_uniform", node_param);
    print "All parameters required for computing pose has been loaded"

if use_additional_parameter:
    print "Setting up crop box"
    segmenter.setUseCropBox(False);
    base_orientation = EigenQuaternion(0.316470, 0.647486, 0.629978, -0.289372);
    base_origin = EigenTranslation(0.049957, 0.085419, 0.986988);
    # table_pose_to_camera = EigenPose()

    # Setting the eigen pose in python is sometimes buggy (for no reason?)
    # print "Make eigen pose"
    # table_pose_to_camera = makeEigenPose(base_orientation,base_origin);
    # print "Setting cropbox size"
    # segmenter.setCropBoxSize(0.35,0.35,0.35);
    # print "Setting cropbox pose"
    # segmenter.setCropBoxPose(table_pose_to_camera);
    # print "Crop box has been set successfully."
    if use_table:
        segmenter.setUseTableSegmentation(True);
        segmenter.setCropAboveTableBoundary(0.01, 0.5);
        load_table_from_file = False;

        if (load_table_from_file):
            segmenter.loadTableFromFile("../sample_pcd_data/table.pcd");
        else:
            print "Generating table point cloud segmentation.";
            # generate new table
            segmenter.setTableSegmentationParameters(0.02,2.0,7500);
            raw_unsegmented_table_points = loadPointCloudFromFile("../sample_pcd_data/raw_unsegmented_table.pcd")
            segmenter.getTableSurfaceFromPointCloud(raw_unsegmented_table_points,False,".");

    if compute_pose:
        segmenter.setUseObjectPersistence(True);
        segmenter.setUsePreferredOrientation(True);
        segmenter.setPreferredOrientation(base_orientation);
        segmenter.addModelSymmetricProperty("node_uniform", 90, 90, 90, 90, "z");
        segmenter.addModelSymmetricProperty("link_uniform", 180, 180, 90, 90, "z");

    print "All additional parameters has been loaded."

# -------------------------------------------------------------------------
# Initialize the segmenter with the parameters that has been set
print "Initialize segmenter based on input parameters.\n";
segmenter.initializeSemanticSegmentation();

# Do point cloud segmentation and calculate its pose
raw_unsegmented_points_that_contains_objects = loadPointCloudFromFile("../sample_pcd_data/raw_sample_point_cloud.pcd")
segmented_point_cloud_labels = PointCloudXYZL()
if segmenter.segmentPointCloud(raw_unsegmented_points_that_contains_objects, segmented_point_cloud_labels):
    if compute_pose:
        print "Object point cloud segmentation is successful.\n";
        print "List of detected object transform: ";
        printAllPoses(segmenter.calculateObjTransform(segmented_point_cloud_labels));

