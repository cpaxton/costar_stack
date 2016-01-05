#include <opencv2/core/core.hpp>

#include "../include/features.h"
#include "../include/BBDataParser.h"
#include "../include/UWDataParser.h"
#include "../include/JHUDataParser.h"

#define MAX_LAYER 7

int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/scene/");
    std::string scene_name("office");
    pcl::console::parse_argument(argc, argv, "--i", scene_name);
    
    std::vector<std::string> background_name;
    background_name.push_back(scene_name + "_background");
    background_name.push_back(scene_name + "_raw");
    background_name.push_back(scene_name + "_imposter");
    
    std::string shot_path("UW_shot_dict/");
    std::string svm_path(scene_name + "_svm/");
    boost::filesystem::create_directories(svm_path+"hard/");
/***************************************************************************************************************/
    float radius = 0.02;
    float down_ss = 0.003;
    float ratio = 0.1;
    pcl::console::parse_argument(argc, argv, "--rd", radius);
    pcl::console::parse_argument(argc, argv, "--rt", ratio);
    pcl::console::parse_argument(argc, argv, "--ss", down_ss);
    
    std::cerr << "Ratio: " << ratio << std::endl;
    std::cerr << "Downsample: " << down_ss << std::endl;
/***************************************************************************************************************/
    Hier_Pooler hie_producer(radius);
    hie_producer.LoadDict_L0(shot_path, "200", "200");
    hie_producer.setRatio(ratio);
/***************************************************************************************************************/
    std::vector< boost::shared_ptr<Pooler_L0> > lab_pooler_set(5+1);
    for( size_t i = 1 ; i < lab_pooler_set.size() ; i++ )
    {
        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0);
        cur_pooler->setHSIPoolingParams(i);
        lab_pooler_set[i] = cur_pooler;
    }
/***************************************************************************************************************/
    std::vector<model*> binary_models(3);
    for( int ll = 0 ; ll <= 2 ; ll++ )
    {
        std::stringstream ss;
        ss << ll;
        
        binary_models[ll] = load_model((svm_path+"binary_L"+ss.str()+".model").c_str());
    }
    
    if( true )
    {
        int test_dim = -1;
        std::vector< std::vector< sparseVec> > final_test(MAX_LAYER);
        for( size_t k = 0 ; k < background_name.size() ; k++ )
        {
            std::stringstream kk;
            kk << k;
            
            #pragma omp parallel for schedule(dynamic, 1)
            for( int j = 0 ; j < 200 ; j++ )
            {
                std::stringstream ss;
                ss << j;

                std::string filename(in_path + background_name[k] +"/" + background_name[k] + "_" + ss.str() + ".pcd");
                std::cerr << "Loading " << filename << std::endl;
                if( exists_test(filename) == false )//|| exists_test(filename_n) == false )
                {
                    pcl::console::print_warn("Failed to Read: %s\n", filename.c_str());
                    continue;
                }
                pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
                pcl::io::loadPCDFile(filename, *full_cloud);

                spPooler triple_pooler;
                triple_pooler.init(full_cloud, hie_producer, radius, down_ss);
                triple_pooler.build_SP_LAB(lab_pooler_set, false);
                
                for( int ll = 0 ; ll <= 2 ; ll++ )
                {
                    std::vector<cv::Mat> hard_fea = triple_pooler.gethardNegtive(binary_models[ll], ll, false);
                    for( std::vector<cv::Mat>::iterator it = hard_fea.begin(); it < hard_fea.end() ; it++ )
                    {
                        if( test_dim > 0 && it->cols != test_dim )
                        {
                            std::cerr << "Error: test_dim > 0 && cur_final.cols != test_dim   " << test_dim << " " << it->cols << std::endl;
                            exit(0);
                        }
                        else if( test_dim < 0 )
                        {
                            #pragma omp critical
                            {
                                test_dim = it->cols;
                                std::cerr << "Fea Dim: " << test_dim << std::endl;
                            }
                        }	
                        std::vector< sparseVec> this_sparse;
                        sparseCvMat(*it, this_sparse);
                        #pragma omp critical
                        {
                            final_test[ll].push_back(this_sparse[0]);
                        }
                    }
                }
                
                triple_pooler.reset();
            }
            
        }
        for( int ll = 0 ; ll < final_test.size() ; ll++ )
        {
            if( final_test[ll].empty() == true )
                continue;
            std::stringstream mm;
            mm << ll;
            saveCvMatSparse( svm_path + "hard/train_0_L" + mm.str() +".smat", final_test[ll], test_dim);
            final_test[ll].clear();
        }
    }
    return 1;
} 

