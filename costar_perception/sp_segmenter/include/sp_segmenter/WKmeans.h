#pragma once
#include "sp_segmenter/utility/utility.h"

struct protoT{
    sparseVec elems;
    size_t count;
};

void HierKmeans(const std::vector<protoT> &proto_set, cv::Mat &centers, int K);
 
class WKmeans{
public:
    WKmeans();
    ~WKmeans();

    void setThreadNum(int thread_num_){ omp_set_num_threads(thread_num_);thread_num = thread_num_;}
    void AddData(cv::Mat data, cv::Mat weights);
    void W_Cluster(cv::Mat &centers, int K, int numIter = 10);

private:
    std::vector< std::vector<int> > cSetIdx;
    int thread_num;                                 //omp_get_num_threads()
    float eps;      //stopping eps
    
    int fea_len;
    std::vector< cv::Mat > raw_data_vec;
    std::vector< float > data_w;
    
private:
    float M_step(cv::Mat centers);           //assigning label for each proto based on the nearest center
    void E_step(cv::Mat &centers, int K);            //averaging each group and get new center

    cv::Mat Initialize(int K);                      //initialize the centers
};

class sparseK {
public:
    sparseK();
    ~sparseK();

    void AddData(cv::Mat data);
    void LoadProtos(std::vector<protoT> &proto_set_){proto_set = proto_set_;fea_len = proto_set[0].elems[proto_set[0].elems.size()-1].index;}
    std::vector<float> W_Cluster(cv::Mat &centers, int K, std::vector< std::vector<protoT> > &protoByCluster);
    void setThreadNum(int thread_num_){ omp_set_num_threads(thread_num_);thread_num = thread_num_;}

    std::vector<protoT> proto_set;
private:
	
    bool M_step(std::vector< sparseVec > &centers, int K, std::vector<float> &energy_set);
    void E_step(std::vector< sparseVec > &centers, int K);

    void Initialize(std::vector< sparseVec > &centers, int K);

    float distFunc2(const sparseVec &vec1, const sparseVec &vec2);

    sparseVec convertToSparse(cv::Mat data);
    cv::Mat convertTDense(sparseVec &data);
    
    int fea_len;			//-1
    std::vector< std::vector<int> > cSetIdx;
    int thread_num;		//omp_get_num_threads()
    size_t total_count;		//0
    bool weight_flag;		//flagging the weighted kmeans
};