#include "sp_segmenter/WKmeans.h"

WKmeans::WKmeans()
{
    eps = 1e-6;
    thread_num = omp_get_max_threads();
}

WKmeans::~WKmeans(){}

void WKmeans::AddData(cv::Mat data, cv::Mat weights)
{
    if( data.rows != weights.rows )
    {
        std::cerr << "data.rows != weights.rows" << std::endl;
        exit(0);
    }
    if( raw_data_vec.empty() == true )
        fea_len = data.cols;
    if( data.cols != fea_len )
    {
        std::cerr << "data.cols != feature len" << std::endl;
        exit(0);
    }
    float *ptr = (float *)weights.data;
    for( int i = 0 ; i < data.rows ; i++, ptr++ )
    {
        raw_data_vec.push_back(data.row(i));
        data_w.push_back(*ptr);
    }
}

cv::Mat WKmeans::Initialize(int K)  	//initialize the centers, should be deallocated outside the function after clustering
{
    std::vector<size_t> rand_idx;
    GenRandSeq(rand_idx, raw_data_vec.size());
    
    cv::Mat centers = cv::Mat::zeros(K, fea_len, CV_32FC1);
    for( int i = 0 ; i < K ; i++ )
        raw_data_vec[rand_idx[i]].copyTo(centers.row(i));
    
    return centers;
}

//assigning label for each proto based on the nearest center
float WKmeans::M_step(cv::Mat centers)		
{
    int K = centers.rows;
    cv::flann::LinearIndexParams indexParams;
    
    cv::flann::Index center_tree(centers, indexParams);
    
    std::vector< std::vector<std::vector<int> > > index_set(thread_num);
    std::vector< float > dist_set(thread_num, 0);
    for( int t = 0 ; t < thread_num ; t++ )
        index_set[t].resize(K);
    
    int num = raw_data_vec.size();
    #pragma omp parallel for schedule(dynamic, 1)
    for( int i = 0 ; i < num ; i++ ){
        
        std::vector<int> idxs;
        std::vector<float> dists;
        center_tree.knnSearch(raw_data_vec[i], idxs, dists, 1, cv::flann::SearchParams ());
        
        int thread_id = omp_get_thread_num();
        index_set[thread_id][idxs[0]].push_back(i);
        dist_set[thread_id] += data_w[i]*dists[0];
    }
    int count = 0;
    for( int k = 0 ; k < K ; k++ ){
        cSetIdx[k].clear();
        
        for( int t = 0 ; t < thread_num ; t++ )
            cSetIdx[k].insert(cSetIdx[k].end(), index_set[t][k].begin(), index_set[t][k].end());
    }
	 
    float energy = 0;
    for( int t = 0 ; t < thread_num ; t++ )
        energy += dist_set[t];
    return energy;
}


//averaging each group and get new center
void WKmeans::E_step(cv::Mat &centers, int K)
{
    centers.release();
    centers = cv::Mat::zeros(K, fea_len, CV_32FC1);
    
    #pragma omp parallel for schedule(dynamic, 1)
    for( int k = 0 ; k < K;  k++ ){
        cv::Mat cur_center = centers.row(k);
        float w_sum = 0;
        for( std::vector<int>::iterator it = cSetIdx[k].begin() ; it < cSetIdx[k].end() ; it++ )
        {    
            cur_center += data_w[*it]*raw_data_vec[*it];
            w_sum += data_w[*it];
        }
        cur_center /= w_sum;
    }
}

void WKmeans::W_Cluster(cv::Mat &centers, int K, int numIter)
{
    centers = cv::Mat::zeros(K, fea_len, CV_32FC1);
    float min_energy = INF_;
    cSetIdx.resize(K);
    
    for( int t = 0 ; t < numIter ; t++ )
    {
        std::cerr << "Round-" << t << std::endl;
        cv::Mat tmp_centers = Initialize(K);
        
        int cur_counter = 0;
        float pre_energy = -INF_, cur_energy = INF_;
        while( true )
        {
            cur_energy = M_step(tmp_centers);
            //std::cerr<<"Iter-"<<cur_counter<<" Energy: "<<cur_energy<<std::endl;
            if( fabs(pre_energy - cur_energy) < eps )
                break;
            
            E_step(tmp_centers, K);
            pre_energy = cur_energy;
            cur_counter++;
        }
        std::cerr << "Energy-" << cur_energy << std::endl;
        if( cur_energy < min_energy )
        {
            min_energy = cur_energy;
            tmp_centers.copyTo(centers);
        }
    }
    
    cSetIdx.clear();
}

//Sparse Data Kmeans
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
sparseK::sparseK()
{
    total_count = 0;

    fea_len = -1;

    thread_num = omp_get_max_threads();

    weight_flag = false;
}

sparseK::~sparseK(){}

float sparseK::distFunc2(const sparseVec &vec1, const sparseVec &vec2)
{
    //suppose vec1 and vec2 are in order to their indexes
    float sum = 0;
    sparseVec::const_iterator ptr1 = vec1.begin();
    sparseVec::const_iterator ptr2 = vec2.begin();
    while( ptr1 < vec1.end() - 1 || ptr2 < vec2.end() - 1 )
    {
        if( ptr1->index < ptr2->index )
        {
            sum += ptr1->value * ptr1->value;
            ptr1++;
        }
        else if( ptr2->index < ptr1->index )
        {
            sum += ptr2->value * ptr2->value;
            ptr2++;
        }
        else
        {
            float buf = ptr1->value - ptr2->value;
            sum += buf * buf;
            ptr1++;
            ptr2++;
        }
    }

    //return sum;
    return sqrt(sum);
}

sparseVec sparseK::convertToSparse(cv::Mat data)
{
	sparseVec temp;
	float *base_ptr = (float *)data.data;
	for( int i = 0 ; i < fea_len ; i++, base_ptr++ ){
		if(*base_ptr != 0)
		{
			feature_node buf;
			buf.index = i;
			buf.value = *base_ptr;
			temp.push_back(buf);
		}
	}
	feature_node buf;
	buf.index = fea_len;
	buf.value = 0;

	temp.push_back(buf);

	return temp;
}

cv::Mat sparseK::convertTDense(sparseVec &data)
{
	cv::Mat temp = cv::Mat::zeros(1, fea_len, CV_32FC1);
        
        for( sparseVec::iterator it = data.begin(); it < data.end() - 1; it++ )
        {
            temp.at<float>(0, (*it).index) = (*it).value;
	}
	return temp;
}

void sparseK::AddData(cv::Mat data)
{
    if( proto_set.empty() == true )
        fea_len = data.cols;

    assert( data.rows == fea_len );
    sparseVec cur_data = convertToSparse(data);
    total_count++;

    protoT buf;
    buf.elems = cur_data;
    buf.count = 1;
    proto_set.push_back(buf);


    if( total_count != proto_set.size() )
    {
        std::cerr<<"BUG1!"<<std::endl;
        exit(0);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void sparseK::Initialize(std::vector< sparseVec > &centers, int K) //initialize the centers, should be deallocated outside the function after clustering
{
	cSetIdx.clear();
	centers.clear();
	cSetIdx.resize(K);
	centers.resize(K);

	int num = proto_set.size();
	if( weight_flag == true )
	{
		bool *flag = new bool[num];
		memset(flag, 0, sizeof(bool)*num);

		for( int k = 0 ; k < K ; k++ ){
			size_t max = 0;
			int max_idx = -1;
			for(int i = 0 ; i < num ; i++ ){
				if( flag[i] == false && proto_set[i].count > max)
				{
					max = proto_set[i].count;
					max_idx = i;
				}
			}
		
			centers[k] = proto_set[max_idx].elems;
			flag[max_idx] = true;
		}

		delete []flag;
	}
	else
	{
		std::vector<size_t> rand_idx;
		GenRandSeq(rand_idx, num);

		for( int k = 0 ; k < K ; k++ )
			centers[k] = proto_set[rand_idx[k]].elems;
		
	}
}

//assigning label for each proto based on the nearest center
bool sparseK::M_step(std::vector< sparseVec > &centers, int K, std::vector<float> &energy_set)		
{
	std::vector< std::vector<std::vector<int> > > index_set(thread_num);
	std::vector< std::vector<float> > dist_set(thread_num);
	for( int t = 0 ; t < thread_num ; t++ ){
		index_set[t].resize(K);
		dist_set[t].resize(K, 0);
	}

	int num = proto_set.size();
	#pragma omp parallel for
	for( int i = 0 ; i < num; i++ ){
		float min_dist = INF_, buf;
		int min_idx = -1;
		for( int k = 0 ; k < K ; k++ ){
			buf = distFunc2(centers[k], proto_set[i].elems);
			if( buf < min_dist )
			{
				min_dist = buf;
				min_idx = k;
			}
		}
		int thread_id = omp_get_thread_num();
		index_set[thread_id][min_idx].push_back(i);
		dist_set[thread_id][min_idx] += min_dist;
	}

	int count = 0;
        energy_set.clear();
	energy_set.resize(K, 0);
	bool no_change = true;
	for( int k = 0 ; k < K ; k++ ){
		//std::vector<int> pre_label = cSetIdx[k];
		cSetIdx[k].clear();
		for( int t = 0 ; t < thread_num ; t++ ){
			cSetIdx[k].insert(cSetIdx[k].end(), index_set[t][k].begin(), index_set[t][k].end());
			energy_set[k] += dist_set[t][k];
                }
	}

	return no_change;
}


//averaging each group and get new center
void sparseK::E_step(std::vector< sparseVec > &centers, int K)
{
    centers.clear();
    centers.resize(K);
    //#pragma omp parallel for firstprivate(K)
    for( int k = 0 ; k < K;  k++ ){
        int num = cSetIdx[k].size();

        cv::Mat new_center = cv::Mat::zeros(1, fea_len, CV_32FC1);
        float *ptr = (float *)new_center.data, wi, w_sum =0;
        for( int i = 0 ; i < num ; i++ ){
            wi = weight_flag == true ? proto_set[ cSetIdx[k][i] ].count : 1;

            sparseVec::iterator s_it = proto_set[ cSetIdx[k][i] ].elems.begin();
            sparseVec::iterator e_it = proto_set[ cSetIdx[k][i] ].elems.end()-1;
            for( sparseVec::iterator it = s_it ;it < e_it ; it++ )
                *(ptr + (*it).index ) += wi*(*it).value;

            w_sum += wi;
        }
        new_center /= w_sum;
        //if( cv::norm(new_center, cv::NORM_L2) > 1.0 ){
        //    std::cerr << cv::norm(new_center, cv::NORM_L2) << std::endl;
        //    std::cerr << w_sum << std::endl;
        //    std::cin.get();
        //}
        centers[k] = convertToSparse(new_center);
    }
}

std::vector<float> sparseK::W_Cluster(cv::Mat &centers, int K, std::vector< std::vector<protoT> > &protoByCluster)
{
    protoByCluster.clear();
    protoByCluster.resize(K);

    std::vector< sparseVec > tmp_centers;
    //std::cerr<<"Initializing K Centers..."<<std::endl;
    Initialize(tmp_centers, K);
    //std::cerr<<"Initializing K Centers Done..."<<std::endl;

    int iter = 0;
    float pre_energy = INF_, cur_energy;
    std::vector<float> energy_set;

    while( true )
    {
        energy_set.clear();
        bool no_change = M_step(tmp_centers, K, energy_set);
        cur_energy = 0;
        for( int k = 0 ; k < K ; k++ )
            cur_energy += energy_set[k];

        if( pre_energy - cur_energy < 1e-6 )
        {
            //std::cerr<<iter<<std::endl;
            break;
        }
        //if( no_change == true )
        //	break;

        E_step(tmp_centers, K);
        
        pre_energy = cur_energy;
        iter++;
    }

    centers = cv::Mat::zeros(K, fea_len, CV_32FC1);
    for( int k = 0 ; k < K ; k++ )
    {
        cv::Mat buf = convertTDense(tmp_centers[k]);
        buf.copyTo(centers.row(k));
        protoByCluster[k].clear();
        protoByCluster[k].resize(cSetIdx[k].size());
        std::vector<int>::iterator it;
        int i = 0;
        for( it = cSetIdx[k].begin(), i = 0 ; it < cSetIdx[k].end() ; it++, i++ )
            protoByCluster[k][i] = proto_set[(*it)];
    }

    cSetIdx.clear();
    return energy_set;
}

void HierKmeans(const std::vector<protoT> &proto_set, cv::Mat &centers, int K)
{
    int count = 0;
    if( K <= 1 )
        return;

    int min_num = (proto_set.size()+0.0) / (4.0*K);
    min_num = min_num < 100 ? 100 : min_num;

    std::vector<float> energy_pool(1, INF_-1);
    std::vector<cv::Mat> center_pool(1);
    std::vector< std::vector<protoT> > proto_pool(1);
    proto_pool[0] = proto_set;
//    proto_set.clear();
    while(true)
    {
        if( proto_pool.size() >= K )
            break;

        // Search for the cluster with biggest energy to decompose
        float max_energy = -1, max_idx = -1;
        for( int i = 0 ; i < energy_pool.size(); i++ ){
            if( energy_pool[i] > max_energy && proto_pool[i].size() >= min_num )
            {
                max_energy = energy_pool[i];
                max_idx = i;
            }
        }
        
        if( max_idx < 0 )
        {
            std::cerr<<"Stop Growing!"<<std::endl;
            break;
        }
        sparseK	temp_Kmeans;
        temp_Kmeans.setThreadNum(8);
        temp_Kmeans.LoadProtos(proto_pool[max_idx]);
        proto_pool[max_idx].clear();
        std::vector< std::vector<protoT> > cur_proto_pool;
        cv::Mat cur_centers;
        std::vector<float> buf_energy_set = temp_Kmeans.W_Cluster(cur_centers, 2, cur_proto_pool);
        if( cur_proto_pool.size() != 2 )
        {
            std::cerr<<"BUG1!!!"<<std::endl;
            exit(0);
        }
        //proto_pool[max_idx].clear();
        proto_pool[max_idx] = cur_proto_pool[0];
        cv::Mat buf0 = cv::Mat::zeros(1, cur_centers.cols, CV_32FC1);
        cur_centers.row(0).copyTo(buf0);
        center_pool[max_idx] = buf0;
        energy_pool[max_idx] = buf_energy_set[0];
        
        proto_pool.push_back(cur_proto_pool[1]);
        cv::Mat buf1 = cv::Mat::zeros(1, cur_centers.cols, CV_32FC1);
        cur_centers.row(1).copyTo(buf1);
        center_pool.push_back(buf1);
        energy_pool.push_back(buf_energy_set[1]);
        
        float sum_energy = 0;
        for( size_t e = 0 ; e < energy_pool.size() ; e++ )
            sum_energy+=energy_pool[e];

        std::cerr<<"ITER-"<<count<<", Current Energy: "<<sum_energy<<std::endl;
        count++;
        
    }

    centers = cv::Mat::zeros(center_pool.size(), center_pool[0].cols, CV_32FC1);
    for( int i = 0 ; i < center_pool.size(); i++ )
        center_pool[i].copyTo(centers.row(i));

    std::cerr<<"Final Center Number: "<<centers.rows<<std::endl;

}