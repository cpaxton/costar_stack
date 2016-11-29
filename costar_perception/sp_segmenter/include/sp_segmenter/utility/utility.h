#pragma once
#include "typedef.h"
#include "mcqd.h"
#include "liblinear/linear.h"

#define MAX_SEGMENT 100

typedef std::vector<feature_node> sparseVec;

struct SparseDataOneClass{
    std::vector< sparseVec > fea_vec;
    int label;
};

struct poseT{
    std::string model_name;
    Eigen::Vector3f shift;
    Eigen::Quaternion<float> rotation;
    double confidence;
};

struct ModelT{
    pcl::PointCloud<myPointXYZ>::Ptr model_cloud;
    pcl::PolygonMesh::Ptr model_mesh; 
    pcl::PointCloud<myPointXYZ>::Ptr model_center;
    std::string model_label;
};

struct PR_ELEM
{
    float recall;
    float precision;
    float f_score;
    bool valid;
};

typedef std::vector< std::vector<int> > IDXSET;

double get_wall_time();

int saveMat( const std::string& filename, const cv::Mat& M);

int readMat( const std::string& filename, cv::Mat& M);

void GenRandSeq(std::vector<size_t> &rand_idx, size_t len);

bool exists_test (const std::string& name);

bool exists_dir (const std::string& name);

bool play_dice(float prob);

void CheckNormal(cv::Mat normal);

cv::Mat getFullImage(const pcl::PointCloud<PointT>::Ptr full_cloud);

std::vector< std::vector<int> > connectedNodes(bool adj[][MAX_SEGMENT], int num);

void showCorrs(const std::vector<keyT> &key1, const std::vector<keyT> &key2, pcl::CorrespondencesPtr corrs, 
	const pcl::PointCloud<PointT>::Ptr surface1, const pcl::PointCloud<PointT>::Ptr surface2, pcl::visualization::PCLVisualizer::Ptr &viewer);


// From my previous stuff
void RGBToHSI(int rgb[], float hsi[]);

void RGBToLab(int rgb[], float lab[]);

double computeCloudResolution (const pcl::PointCloud<PointT>::ConstPtr &cloud);

void ComputeCentroid(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<myPointXYZ>::Ptr center_cloud);

void computeNormals(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<NormalT>::Ptr &cloud_normals, float normal_ss);

void computeKeyNormals(const pcl::PointCloud<myPointXYZ>::Ptr keypoints, pcl::PointCloud<NormalT>::Ptr &keypoints_normals, pcl::PointCloud<myPointXYZ>::Ptr surface, float normal_ss);

void ComputeKeyRF(std::vector<keyT> &keypoints, const pcl::PointCloud<myPointXYZ>::Ptr surface, const pcl::PointCloud<NormalT>::Ptr surface_normals, float rf_rad);

void RefineCloud(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr refined_cloud, float radius);

void ExtractHue(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hue);

cv::Mat fpfh_cloud(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<PointT>::Ptr keys, const pcl::PointCloud<NormalT>::Ptr cloud_normals, float radius, bool normalzied = true);

//cv::Mat shot_cloud(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normals, float radius);

cv::Mat shot_cloud_ss(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normals, const pcl::PointCloud<pcl::ReferenceFrame>::Ptr lrf, pcl::PointCloud<PointT>::Ptr &down_cloud, float radius, float ss);

cv::Mat cshot_cloud_uni(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normals, std::vector<int> &rand_idx, float radius, int snum);

cv::Mat cshot_cloud_ss(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normals, const pcl::PointCloud<pcl::ReferenceFrame>::Ptr lrf, pcl::PointCloud<PointT>::Ptr &down_cloud, float radius, float ss);

//cv::Mat usc_cloud_ss(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normals, const pcl::PointCloud<pcl::ReferenceFrame>::Ptr lrf, pcl::PointCloud<PointT>::Ptr &down_cloud, float radius, float ss);

// return the filenames of all files that have the specified extension
// in the specified directory and all subdirectories
void find_files(const boost::filesystem::path &root, const std::string& ext, std::vector<std::string>& ret);

void find_dirs(const boost::filesystem::path &root, std::vector<std::string>& ret);

void ComputeSIFT(const Cloud_Image &frame, std::vector<keyT> &key_vec, std::vector<keyDescrT> &key_descr, bool show_keys = false);

pcl::PointIndices::Ptr  ConnectedComponent3D(const pcl::PointCloud<PointT>::Ptr cloud, float radius = 0.01, int min_segment = 100);

pcl::PointCloud<myPointXYZ>::Ptr GenNormalPt(const pcl::PointCloud<myPointXYZ>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normals, float len);

float distPt(const myPointXYZ &pt1, const myPointXYZ &pt2);

void transformKeys(std::vector<keyT> &keys, const Eigen::Matrix4f &trans);

pcl::PointCloud<myPointXYZ>::Ptr uniformSampleCloud(const pcl::PointCloud<myPointXYZ>::Ptr cloud, int num);

std::vector< std::vector<int> > maximalClique(const Eigen::MatrixXi &adj_mat);

void saveUndirectedGraph(const Eigen::MatrixXi &adj, std::string filename);

Eigen::MatrixXi readUndirectedGraph(std::string filename);

poseVec readRots(std::string filename);

bool checkTrans(const Eigen::Matrix4f &tran);

pcl::PointCloud<PointT>::Ptr createPartialView(const pcl::PointCloud<PointT>::Ptr cloud, float sampleR = 0.01);

void genViews(const pcl::PointCloud<PointT>::Ptr full_cloud, const poseVec &rot_set, 
        CloudSet &partial_set, poseVec &partial_ground_tran, pcl::visualization::PCLVisualizer::Ptr viewer, float s1=0.001, float s2=0.003);

void writePoseTxT(std::string filename, const Eigen::Matrix4f &mat);

cv::Mat read2d(std::string filename);

cv::Mat read3d(std::string filename);

void FormFeaSparseMat(std::vector<SparseDataOneClass> &data_set, problem &prob, int total, int fea_dim);

void sparseCvMat(cv::Mat data, std::vector< std::vector<feature_node> > &fea_vec);

void saveCvMatSparse(std::string filename, const cv::Mat &data);

void saveCvMatSparse(std::string filename, const std::vector< sparseVec> &data, int dim);

void GenSVMParamter(parameter &param, float CC);

void saveCvMatSparse(std::string filename, const cv::Mat &data);

void mergeProbs(const std::vector<problem> &problem_set, problem &final_prob);

size_t readSoluSparse_piecewise(std::string filename, std::vector< std::vector<feature_node> > &fea_vec, std::vector< std::pair<int, int> > inds);

cv::Mat readCvMatSparse(std::string filename);

void getNonNormalPCDFiles(std::string path, std::vector<std::string> &files);

cv::Mat sparseToMat(const std::vector<feature_node> &fea, int dim);

pcl::PointCloud<PointT>::Ptr NormalizeXY(const pcl::PointCloud<PointT>::Ptr cloud);

void NormalizeCloud(pcl::PointCloud<PointT>::Ptr cloud);

cv::Mat getHSIHist(const pcl::PointCloud<PointT>::Ptr cloud, cv::flann::Index &dict, int len, int K, float ss = -1);

cv::Mat TriEncoder(cv::Mat data, cv::Mat dict);

cv::Mat KNNEncoder(cv::Mat data, cv::flann::Index &tree, int len, int K); //for L2 normalized data

void MaxOP(cv::Mat dom, cv::Mat sub);

cv::Mat gen3DNormCube(int len);

pcl::PointCloud<pcl::ReferenceFrame>::Ptr computeRF(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normals, pcl::PointCloud<PointT>::Ptr &down_cloud, float radius = 0.035, float ss = -1);

MulInfoT convertPCD(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normal);

bool PreCloud(MulInfoT &data, float ss, bool light_flag = true);

int readCSV(std::string filename, std::string label, std::vector<poseT> &poses);
int writeCSV(std::string filename, std::string label, const std::vector<poseT> &poses);

pcl::PointCloud<PointLT>::Ptr SPCloud(const pcl::PointCloud<PointT>::Ptr cloud, std::vector<pcl::PointCloud<PointT>::Ptr> &segs, IDXSET &seg_to_cloud, std::multimap<uint32_t, uint32_t> &graph, 
                                    float voxel_resol = 0.008f, float seed_resol = 0.1f, float color_w = 0.2f, float spatial_w = 0.4f, float normal_w = 1.0f);

pcl::PointCloud<PointT>::Ptr SPCloud(const pcl::PointCloud<PointT>::Ptr cloud, std::vector<pcl::PointIndices::Ptr> &segs_idx, std::multimap<uint32_t, uint32_t> &graph, 
                                    float voxel_resol = 0.008f, float seed_resol = 0.1f, float color_w = 0.2f, float spatial_w = 0.4f, float normal_w = 1.0f);

void ReadCloudNormal(const std::string path, std::vector<std::string> &pcd_files, std::vector<std::string> &normal_files);