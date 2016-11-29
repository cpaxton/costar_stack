#ifndef SP_SEGMENTER_SIFT_POOLING_H
#define SP_SEGMENTER_SIFT_POOLING_H
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sp_segmenter/features.h"
#include "sp_segmenter/UWDataParser.h"

std::vector<cv::Mat> SIFTPooling(const MulInfoT &data, const std::vector<cv::SiftFeatureDetector*> &sift_det_vec, cv::SiftDescriptorExtractor * sift_ext, 
                    Hier_Pooler &hie_producer, const std::vector< boost::shared_ptr<Pooler_L0> > &pooler_set, const cv::Mat &atlas, int MODEL_MAX = 100)
{
    // should prepare data with img and map2d
    cv::Mat cur_rgb = data.img;
    cv::Mat cur_map2d = data.map2d;
    cv::Mat cur_gray(cur_rgb.size(), CV_8UC1);
    cv::cvtColor(cur_rgb, cur_gray, CV_BGR2GRAY);

    std::vector<cv::KeyPoint> sift_keys = extSIFTKeys(cur_rgb, sift_det_vec);
//    for( size_t i = 0 ; i < sift_det_vec.size() ; i++ )
//    {
//        std::vector<cv::KeyPoint> tmp_keys;
//	sift_det_vec[i]->detect(cur_gray, tmp_keys);
//	sift_keys.insert(sift_keys.end(), tmp_keys.begin(), tmp_keys.end());
//	//std::cerr << sift_keys.size() << std::endl;
//    }    
    
    std::vector<MulInfoT> data_set(MODEL_MAX+1);
    std::vector< std::vector<cv::KeyPoint> > siftKey_set(MODEL_MAX+1);
    std::vector<int> active_idx;
    std::vector<bool> active_flag(MODEL_MAX+1, false);
    
    for( size_t k = 0 ; k < sift_keys.size() ; k++ )
    {
        int row = round(sift_keys[k].pt.y);
        int col = round(sift_keys[k].pt.x);
        
        int tmp_idx = cur_map2d.at<int>(row, col);
        int label = atlas.at<int>(row, col);
        if( tmp_idx >= 0 && label > 0 )
        {
            if( active_flag[label] == false )
            {
                data_set[label]= convertPCD(data.cloud, data.cloud_normals);
                
                active_idx.push_back(label);
                active_flag[label] = true;
            }
            data_set[label].down_cloud->push_back(data_set[label].cloud->at(tmp_idx));
            siftKey_set[label].push_back(sift_keys[k]);
        }
    }
//    std::cerr << "******" << active_idx.size() << std::endl;
    std::vector<cv::Mat> sift_pool_fea(MODEL_MAX+1);
    for( size_t k = 0 ; k < active_idx.size() ; k++ )
    {
//        std::cerr << " Filtered Keypoints: " << siftKey_set[active_idx[k]].size() << " " << data_set[active_idx[k]].down_cloud->size() << std::endl;
    
        cv::Mat cur_sift_descr;
        sift_ext->compute(cur_gray, siftKey_set[active_idx[k]], cur_sift_descr);
        for(int r = 0 ; r < cur_sift_descr.rows ; r++ )
        {
            cv::normalize(cur_sift_descr.row(r), cur_sift_descr.row(r));
//            std::cerr << cur_sift_descr.row(k) << std::endl;
//            std::cin.get();
        }
        std::vector<cv::Mat> main_fea = hie_producer.getHierFea(data_set[active_idx[k]], 0);
        //std::cerr << main_fea[0].rows << " " << main_fea[0].cols << " " << cur_sift_descr.rows<< std::endl;
	//std::cin.get();
	std::vector<cv::Mat> pool_fea_vec;
	for( size_t j = 1 ; j < pooler_set.size() ; j++ )
    	{
            cv::Mat cur_final1 = pooler_set[j]->PoolOneDomain(cur_sift_descr, main_fea[0], 2, false);
            cv::Mat cur_final2 = pooler_set[j]->PoolOneDomain(cur_sift_descr, main_fea[1], 2, false);
            pool_fea_vec.push_back(cur_final1);
            pool_fea_vec.push_back(cur_final2);
    	}
	cv::hconcat(pool_fea_vec, sift_pool_fea[active_idx[k]]);
        //cv::Mat cur_final1 = genericPooler.PoolOneDomain(cur_sift_descr, main_fea[0], 2, false);
        //cv::Mat cur_final2 = genericPooler.PoolOneDomain(cur_sift_descr, main_fea[1], 2, false);
        //cv::hconcat(cur_final1, cur_final2, sift_pool_fea[active_idx[k]]);
    }
    
    return sift_pool_fea;
}
#endif