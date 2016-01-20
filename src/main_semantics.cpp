#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sp_segmenter/features.h"
#include "sp_segmenter/JHUDataParser.h"

std::map<std::string, int> model_name_map;
uchar color_label[11][3] = 
{ {0, 0, 0}, 
  {255, 0, 0},
  {0, 255, 0},
  {0, 0, 255},
  {255, 255, 0},
  {255, 0, 255},
  {0, 255, 255},
  {255, 128, 0},
  {255, 0, 128},
  {0, 128, 255},
  {128, 0, 255},
};   

std::vector<std::string> readMesh(std::string mesh_path, std::vector<ModelT> &model_set);
std::vector<poseT> readGT(std::string pose_path, std::string file_id);
pcl::PointCloud<PointLT>::Ptr genSeg_all(const pcl::PointCloud<PointT>::Ptr scene, const std::vector<ModelT> &model_set, const std::vector<poseT> &gt_poses, std::map<std::string, int> &model_map);
//std::vector<int> spGt(const pcl::PointCloud<PointT>::Ptr gt_cloud, const std::vector<pcl::PointCloud<PointT>::Ptr> segs);
void visualizeLabels(const pcl::PointCloud<PointLT>::Ptr label_cloud, pcl::visualization::PCLVisualizer::Ptr viewer, uchar colors[][3]);
std::vector<PR_ELEM> semanticPR(const pcl::PointCloud<PointLT>::Ptr gt_cloud, const pcl::PointCloud<PointLT>::Ptr label_cloud, int model_num);
pcl::PointCloud<PointLT>::Ptr densifyLabels(const pcl::PointCloud<PointLT>::Ptr label_cloud, const pcl::PointCloud<PointT>::Ptr ori_cloud);

void calcPR(const std::vector< std::vector<PR_ELEM> > &pr_vec, std::string out_file)
{
    if( pr_vec.empty() == true )
        return;
    int model_num = pr_vec[0].size() - 1;
    
    std::vector<PR_ELEM> avg_model(model_num+1);
    std::vector<int> obj_count(model_num+1, 0);
    
    for( int i = 0 ; i < model_num+1 ; i++ )
    {
        avg_model[i].precision = 0;
        avg_model[i].recall = 0;
        avg_model[i].f_score = 0;
        avg_model[i].valid = true;
    }
    
    for( int i = 0 ; i < pr_vec.size() ; i++ ){
        if( pr_vec[i].empty() == true )
            continue;
        for( int j = 1 ; j < model_num + 1 ; j++ ){
            
            if( pr_vec[i][j].valid )
            {
                avg_model[j].recall    += pr_vec[i][j].recall;
                avg_model[j].precision += pr_vec[i][j].precision;
                avg_model[j].f_score   += pr_vec[i][j].f_score;

                obj_count[j]++;
            }
        }
        
    }
    
    std::ofstream fp;
//    fp.open((out_path + prefix_set[tt] + "_pr.txt").c_str());
    fp.open(out_file.c_str());

    for( int j = 1 ; j < model_num + 1 ; j++ )
    {
        std::cerr << obj_count[j] << std::endl;
        fp << obj_count[j] << std::endl;

        float cur_recall = avg_model[j].recall / obj_count[j];
        float cur_precision = avg_model[j].precision / obj_count[j];
        float cur_f = avg_model[j].f_score / obj_count[j];

        std::cerr << std::setprecision(4) << cur_recall << "\t" << cur_precision << "\t" << cur_f << std::endl;
        fp << std::setprecision(4) << cur_recall << "\t" << cur_precision << "\t" << cur_f << std::endl;
        
    }
    fp.close();
    
}

int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/scene/");
    std::string scene_name("office");
    
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--i", scene_name);
    
    std::string out_path("../../data_pool/"+scene_name+"/");
    std::string out_cloud_path("../../data_pool/semanticLabels/");
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    
    boost::filesystem::create_directories(out_path);
    
    std::string binary_path(scene_name + "_binary/");
    std::string multi_path("multi_tmp1/");
    std::string mesh_path("/home/chi/devel_mode/ObjRecRANSAC/data/mesh/");
    std::string gt_path("../../data_pool/GT_segs/");
    std::string shot_path("UW_shot_dict/");
    std::string sift_path("UW_new_sift_dict/");
    std::string fpfh_path("UW_fpfh_dict/");
/***************************************************************************************************************/
    
    float radius = 0.02;
    float down_ss = 0.003;
    float ratio = 0.1;
    
    pcl::console::parse_argument(argc, argv, "--rt", ratio);
    pcl::console::parse_argument(argc, argv, "--ss", down_ss);
    std::cerr << "Ratio: " << ratio << std::endl;
    std::cerr << "Downsample: " << down_ss << std::endl;
    
    bool view_flag = false;
    if( pcl::console::find_switch(argc, argv, "-v") == true )
        view_flag = true;
    
/***************************************************************************************************************/
    Hier_Pooler hie_producer(radius);
    hie_producer.LoadDict_L0(shot_path, "200", "200");
    hie_producer.setRatio(ratio);
/***************************************************************************************************************/    
    std::vector<cv::SiftFeatureDetector*> sift_det_vec;
    for( float sigma = 0.7 ; sigma <= 1.61 ; sigma += 0.1 )
    {	
        cv::SiftFeatureDetector *sift_det = new cv::SiftFeatureDetector(
        	0, // nFeatures
        	4, // nOctaveLayers
        	-10000, // contrastThreshold 
        	100000, //edgeThreshold
        	sigma//sigma
        	);
        sift_det_vec.push_back(sift_det);	
    }
/***************************************************************************************************************/
    std::vector< boost::shared_ptr<Pooler_L0> > sift_pooler_set(1+1);
    for( size_t i = 1 ; i < sift_pooler_set.size() ; i++ )
    {
        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0(-1));
        sift_pooler_set[i] = cur_pooler;
    }
    sift_pooler_set[1]->LoadSeedsPool(sift_path+"dict_sift_L0_400.cvmat"); 
/***************************************************************************************************************/
    std::vector< boost::shared_ptr<Pooler_L0> > fpfh_pooler_set(1+1);
    for( size_t i = 1 ; i < fpfh_pooler_set.size() ; i++ )
    {
        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0(-1));
        fpfh_pooler_set[i] = cur_pooler;
    }
    fpfh_pooler_set[1]->LoadSeedsPool(fpfh_path+"dict_fpfh_L0_400.cvmat");
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
        
        binary_models[ll] = load_model((binary_path+"binary_L"+ss.str()+".model").c_str());
    }
    
    std::vector<model*> multi_models(5);
    for( int ll = 0 ; ll <= 4 ; ll++ )
    {
        std::stringstream ss;
        ss << ll;
        
        multi_models[ll] = load_model((multi_path+"multi_L"+ss.str()+".model").c_str());
    }
/***************************************************************************************************************/
    setObjID(model_name_map);
    std::vector<ModelT> model_set;
    std::vector<std::string> model_names = readMesh(mesh_path, model_set);
    int model_num = model_names.size();
/***************************************************************************************************************/
    
    pcl::visualization::PCLVisualizer::Ptr viewer;
    if( view_flag )
    {
        viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->initCameraParameters();
        viewer->addCoordinateSystem(0.1);
        viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
        viewer->setSize(1280, 960);
    }

    std::vector< std::string > prefix_set;
    prefix_set.push_back(scene_name);
    
    for( int tt = 0 ; tt < prefix_set.size() ; tt++ )
    {
        std::vector<std::string> scene_names;
        std::vector<std::string> gt_names;
        std::vector<std::string> out_names;
        
        for( int i = 0 ; i < 10 ; i++ )
        {
            std::stringstream ss;
            ss << i;
            
            std::string cur_path(in_path + prefix_set[tt] +"_"+ ss.str() + "/");
            std::string cur_gt(gt_path + prefix_set[tt] +"_"+ ss.str() + "/");
            std::string cur_out(out_cloud_path + prefix_set[tt] +"_"+ ss.str() + "/");
            boost::filesystem::create_directories(cur_out);
            for( int j = 0 ; j <= 99 ; j++ )
            {
                std::stringstream jj;
                jj << j;
                
                std::string filename(cur_path + prefix_set[tt] +"_"+ ss.str() + "_" + jj.str() + ".pcd");
                scene_names.push_back(filename);
                std::string gt_file(cur_gt + prefix_set[tt] +"_"+ ss.str() + "_" + jj.str() + "_gtseg.pcd");
                gt_names.push_back(gt_file);
                std::string out_file(cur_out + prefix_set[tt] +"_"+ ss.str() + "_" + jj.str() + "_seg.pcd");
                out_names.push_back(out_file);
            }
        }
        
        int file_num = scene_names.size();
        std::vector< std::vector<PR_ELEM> > pr_fore_L0(file_num);
        std::vector< std::vector<PR_ELEM> > pr_fore_L1(file_num);
        std::vector< std::vector<PR_ELEM> > pr_fore_L2(file_num);

        std::vector< std::vector<PR_ELEM> > pr_multi_L1(file_num);
        std::vector< std::vector<PR_ELEM> > pr_multi_L2(file_num);
        std::vector< std::vector<PR_ELEM> > pr_multi_L3(file_num);
        std::vector< std::vector<PR_ELEM> > pr_multi_L4(file_num);
        
//        #pragma omp parallel for schedule(dynamic, 1)
        for( int i = 0 ; i < file_num ; i++ )
        {
            std::string filename(scene_names[i]);
            std::cerr << filename << std::endl;
            std::string gt_file(gt_names[i]);

            if( exists_test(filename) == false )//|| exists_test(gt_file) == false )
            {
                pcl::console::print_warn("Failed to Read: %s\n", filename.c_str());
                continue;
            }
            pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
            pcl::io::loadPCDFile(filename, *full_cloud);
            
            pcl::PointCloud<PointLT>::Ptr all_gt_cloud(new pcl::PointCloud<PointLT>());
            pcl::io::loadPCDFile<PointLT>(gt_file, *all_gt_cloud);
            
            pcl::PointCloud<PointLT>::Ptr fore_gt_cloud(new pcl::PointCloud<PointLT>());
            pcl::copyPointCloud(*all_gt_cloud, *fore_gt_cloud);
            for( pcl::PointCloud<PointLT>::iterator it = fore_gt_cloud->begin() ; it < fore_gt_cloud->end(); it++ )
                if( it->label > 0 )
                    it->label = 1;

            spPooler triple_pooler;
            triple_pooler.init(full_cloud, hie_producer, radius, down_ss);
            triple_pooler.build_SP_LAB(lab_pooler_set, false);
//            std::cerr << "Stage 1" << std::endl;
                    
            pcl::PointCloud<PointLT>::Ptr foreground_cloud(new pcl::PointCloud<PointLT>());
            std::vector< std::vector<PR_ELEM> > cur_fore_pr(3);
            for( int ll = 0 ; ll <= 2 ; ll++ )
            {
                bool reset_flag = ll == 0 ? true : false;
                if( ll >= 1 )
                    triple_pooler.extractForeground(false);
                triple_pooler.InputSemantics(binary_models[ll], ll, reset_flag, false);
                foreground_cloud = triple_pooler.getSemanticLabels();
                cur_fore_pr[ll] = semanticPR(fore_gt_cloud, foreground_cloud, 1);
                if( view_flag )
                {
                    viewer->removeAllPointClouds();
                    viewer->addPointCloud(full_cloud, "full");
//                    std::cerr << cur_fore_pr[ll][1].recall << " " << cur_fore_pr[ll][1].precision << std::endl; 
                    visualizeLabels(foreground_cloud, viewer, color_label);
                    viewer->removeAllPointClouds();
                }
            }

            pr_fore_L0[i] = cur_fore_pr[0];
            pr_fore_L1[i] = cur_fore_pr[1];
            pr_fore_L2[i] = cur_fore_pr[2];  
            
            continue;
                        
////            std::cerr << "Stage 2" << std::endl;
//            triple_pooler.extractForeground(true);
//            triple_pooler.build_SP_FPFH(fpfh_pooler_set, radius, false);
//            triple_pooler.build_SP_SIFT(sift_pooler_set, hie_producer, sift_det_vec, false);
//
////            std::cerr << "Stage 3" << std::endl;
//            std::vector< std::vector<PR_ELEM> > cur_multi_pr(5);
//            pcl::PointCloud<PointLT>::Ptr label_cloud(new pcl::PointCloud<PointLT>());
//            for( int ll = 1 ; ll <= 4 ; ll++ )
//            {
////                bool reset_flag = true;
//                bool reset_flag = ll == 1 ? true : false;
//                triple_pooler.InputSemantics(multi_models[ll], ll, reset_flag, false);
//                label_cloud = triple_pooler.getSemanticLabels();
//                cur_multi_pr[ll] = semanticPR(all_gt_cloud, label_cloud, model_num);
//                
//                if( view_flag )
//                {
//                    std::cerr << ll << std::endl;
//                    viewer->removeAllPointClouds();
//                    viewer->addPointCloud(full_cloud, "full");
//                    visualizeLabels(label_cloud, viewer, color_label);
////                    std::stringstream ss;
////                    ss << i;
////                    viewer->saveScreenshot(scene_name+"_"+ss.str()+".png");
//                    viewer->removeAllPointClouds();
//                }
//            }
//            
//            pcl::io::savePCDFile<PointLT>(out_names[i], *label_cloud, true);
//            triple_pooler.reset();
////            std::cerr << "Stage 4" << std::endl;
//            pr_multi_L1[i] = cur_multi_pr[1];
//            pr_multi_L2[i] = cur_multi_pr[2];
//            pr_multi_L3[i] = cur_multi_pr[3];
//            pr_multi_L4[i] = cur_multi_pr[4];
        }
        
        calcPR(pr_fore_L0, out_path + "fore_L0_pr.txt");
        calcPR(pr_fore_L1, out_path + "fore_L1_pr.txt");
        calcPR(pr_fore_L2, out_path + "fore_L2_pr.txt");
        
//        calcPR(pr_multi_L1, out_path + "multi_L1_pr.txt");
//        calcPR(pr_multi_L2, out_path + "multi_L2_pr.txt");
//        calcPR(pr_multi_L3, out_path + "multi_L3_pr.txt");
//        calcPR(pr_multi_L4, out_path + "multi_L4_pr.txt");
    }
    
    for( int ll = 0 ; ll <= 2 ; ll++ )
        free_and_destroy_model(&binary_models[ll]);
    for( int ll = 1 ; ll <= 4 ; ll++ )
        free_and_destroy_model(&multi_models[ll]);
    
    return 1;
} 

void visualizeLabels(const pcl::PointCloud<PointLT>::Ptr label_cloud, pcl::visualization::PCLVisualizer::Ptr viewer, uchar colors[][3])
{
    pcl::PointCloud<PointT>::Ptr view_cloud(new pcl::PointCloud<PointT>());
    for( pcl::PointCloud<PointLT>::const_iterator it = label_cloud->begin() ; it < label_cloud->end() ; it++ )
    {
        if( pcl_isfinite(it->z) == false )
            continue;
        
        PointT pt;
        pt.x = it->x;
        pt.y = it->y;
        pt.z = it->z;
        pt.rgba = colors[it->label][0] << 16 | colors[it->label][1] << 8 | colors[it->label][2];
        view_cloud->push_back(pt);
    }
    
    viewer->addPointCloud(view_cloud, "label_cloud");
//    viewer->spinOnce(1);
    viewer->spin();
    viewer->removePointCloud("label_cloud");
}

std::vector<std::string> readMesh(std::string mesh_path, std::vector<ModelT> &model_set)
{
    boost::filesystem::path p(mesh_path);
    std::vector< std::string > ret;
    find_files(p, ".obj", ret);
    
    std::vector< std::string > valid_names;
    for(size_t i = 0 ; i < ret.size() ; i++ )
    {
        std::string model_name = ret[i].substr(0, ret[i].size()-4);
        ModelT cur_model;
        pcl::PolygonMesh::Ptr model_mesh(new pcl::PolygonMesh()); 
        pcl::io::loadPolygonFile(mesh_path + ret[i], *model_mesh); 
        pcl::PointCloud<myPointXYZ>::Ptr model_cloud(new pcl::PointCloud<myPointXYZ>()); 
        pcl::fromPCLPointCloud2(model_mesh->cloud, *model_cloud);
        cur_model.model_mesh = model_mesh;
        cur_model.model_label = model_name;
        cur_model.model_cloud = model_cloud;
            
        model_set.push_back(cur_model);
        valid_names.push_back(model_name);
    }
    return valid_names;
}

std::vector<poseT> readGT(std::string pose_path, std::string file_id)
{
    std::vector<poseT> gt_poses;
    
    boost::filesystem::path p(pose_path);
    std::vector< std::string > ret;
    find_files(p, "_"+file_id+".csv", ret);
    for(size_t i = 0 ; i < ret.size() ; i++ )
    {
        std::string model_name = ret[i].substr(0, ret[i].size()-5-file_id.size());
        readCSV(pose_path + ret[i], model_name, gt_poses);
    }
    return gt_poses;
}

pcl::PointCloud<PointLT>::Ptr genSeg_all(const pcl::PointCloud<PointT>::Ptr scene, const std::vector<ModelT> &model_set, const std::vector<poseT> &gt_poses, std::map<std::string, int> &model_map)
{
    pcl::PointCloud<myPointXYZ>::Ptr scene_xyz(new pcl::PointCloud<myPointXYZ>());
    pcl::copyPointCloud(*scene, *scene_xyz);
    
    pcl::search::KdTree<myPointXYZ> tree;
    tree.setInputCloud (scene_xyz);
    
    std::vector<int> obj_labels(scene->size(), -1);
    std::vector<float> obj_dist(scene->size(), 1000);
    float T = 0.01;
    
    for(size_t k = 0 ; k < gt_poses.size() ; k++ )
    {
        std::stringstream kk;
        kk << k;

        int model_idx = -1;
        int obj_id = -1;
        for(size_t i = 0 ; i < model_set.size(); i++ ){
            if(model_set[i].model_label == gt_poses[k].model_name)
            {
                model_idx = i;
                obj_id = model_map[model_set[i].model_label];
                //std::cerr << model_set[i].model_label << " " << obj_id << std::endl;
                break;
            }
        }
        if( obj_id <= 0 )
        {
            std::cerr << "No Matching Model!" << std::endl;
            exit(0);
        }
        
        pcl::PointCloud<myPointXYZ>::Ptr buf_cloud(new pcl::PointCloud<myPointXYZ>());
        pcl::transformPointCloud(*(model_set[model_idx].model_cloud), *buf_cloud, gt_poses[k].shift, gt_poses[k].rotation);
        
        for(pcl::PointCloud<myPointXYZ>::iterator it = buf_cloud->begin() ; it < buf_cloud->end() ; it++ )
        {
            std::vector<int> idx;
            std::vector<float> dist;
    
            tree.radiusSearch(*it, T, idx, dist, buf_cloud->size());
            for( size_t j = 0 ; j < idx.size() ; j++ )
            {
                if( obj_dist[idx[j]] > dist[j] )
                {
                    obj_labels[idx[j]] = obj_id;
                    obj_dist[idx[j]] = dist[j];
                } 
            }   
        }
    }
    
    pcl::PointCloud<PointLT>::Ptr seg_cloud(new pcl::PointCloud<PointLT>());
    for(size_t i = 0 ; i < scene->size() ; i++ )
    {
        PointLT new_pt;
        new_pt.x = scene->at(i).x;
        new_pt.y = scene->at(i).y;
        new_pt.z = scene->at(i).z;
        if(obj_labels[i] > 0 )
            new_pt.label = obj_labels[i];
        else
            new_pt.label = 0;
        seg_cloud->push_back(new_pt);
    }
    return seg_cloud;
}

std::vector<PR_ELEM> semanticPR(const pcl::PointCloud<PointLT>::Ptr gt_cloud, const pcl::PointCloud<PointLT>::Ptr label_cloud, int model_num)
{
    std::vector<PR_ELEM> model_pr(model_num + 1);
    
    if( label_cloud->empty() == true )
    {
        for(size_t i = 0 ; i < model_pr.size() ; i++ )
        {
            model_pr[i].precision = 0;
            model_pr[i].recall = 0;
            model_pr[i].f_score = 0;
            model_pr[i].valid = true;
        }
        return model_pr;
    }
    
    float T = 0.015;
    float sqrT = T*T;
    
    std::vector<int> corr_count1(model_num + 1, 0);
    std::vector<int> count_in_cloud(model_num + 1, 0);
    
    pcl::search::KdTree<PointLT> tree1;
    tree1.setInputCloud(gt_cloud);
    for( pcl::PointCloud<PointLT>::const_iterator it = label_cloud->begin() ; it < label_cloud->end() ; it++ )
    {
        if( pcl_isfinite(it->z) == false )
            continue;
        std::vector<int> indices (1);
	std::vector<float> sqr_distances (1);
        int nres = tree1.nearestKSearch(*it, 1, indices, sqr_distances);
        if ( nres >= 1 && sqr_distances[0] <= sqrT )
        {
            uint32_t gt_label = gt_cloud->at(indices[0]).label;
            if( gt_label == it->label )
                corr_count1[it->label]++;
        }
        count_in_cloud[it->label]++;
    }
    
    std::vector<int> corr_count(model_num + 1, 0);
    std::vector<int> count_in_gt(model_num + 1, 0);
    pcl::search::KdTree<PointLT> tree;
    tree.setInputCloud(label_cloud);
    for( pcl::PointCloud<PointLT>::const_iterator it = gt_cloud->begin() ; it < gt_cloud->end() ; it++ )
    {
        if( pcl_isfinite(it->z) == false )
            continue;
        std::vector<int> indices (1);
	std::vector<float> sqr_distances (1);
        int nres = tree.nearestKSearch(*it, 1, indices, sqr_distances);
        if ( nres >= 1 && sqr_distances[0] <= sqrT )
        {
            uint32_t estimated_label = label_cloud->at(indices[0]).label;
            if( estimated_label == it->label )
                corr_count[it->label]++;
//            count_in_cloud[estimated_label]++;
        }
        count_in_gt[it->label]++;
    }
    
    for( size_t i = 1 ; i < corr_count.size() ; i++ )
    {
        if( count_in_gt[i] <= 0 )
        {
            model_pr[i].valid = false;
            model_pr[i].recall = -1.0;
            model_pr[i].precision = -1.0;
            model_pr[i].f_score = -1.0;
        }
        else
        {
            float cur_recall = (corr_count[i] + 0.0) / count_in_gt[i];
            float cur_precision = count_in_cloud[i] == 0 ? 0.0 : (corr_count1[i] + 0.0) / count_in_cloud[i];
            model_pr[i].valid = true;
            model_pr[i].recall = cur_recall;
            model_pr[i].precision = cur_precision;
            model_pr[i].f_score = cur_recall + cur_precision == 0 ? 0 : 2*(cur_precision*cur_recall)/(cur_recall+cur_precision);
        }
    }
    return model_pr;
}

pcl::PointCloud<PointLT>::Ptr densifyLabels(const pcl::PointCloud<PointLT>::Ptr label_cloud, const pcl::PointCloud<PointT>::Ptr ori_cloud)
{
    pcl::search::KdTree<PointLT> tree;
    tree.setInputCloud(label_cloud);
    
    pcl::PointCloud<PointLT>::Ptr dense_cloud(new pcl::PointCloud<PointLT>());
    
    float T = 0.01;
    float sqrT = T*T;
    for( pcl::PointCloud<PointT>::const_iterator it = ori_cloud->begin() ; it < ori_cloud->end() ; it++ )
    {
        if( pcl_isfinite(it->z) == false )
            continue;
        PointLT tmp;
        tmp.x = it->x;
        tmp.y = it->y;
        tmp.z = it->z;
       
        std::vector<int> ind (1);
	std::vector<float> sqr_dist (1);
        int nres = tree.nearestKSearch(tmp, 1, ind, sqr_dist);
        if ( nres >= 1 && sqr_dist[0] <= sqrT )
        {
            tmp.label = label_cloud->at(ind[0]).label;
            dense_cloud->push_back(tmp);
        }   
    }
    return dense_cloud;
}



/*            pcl::PointCloud<PointT>::Ptr tmp_cloud(new pcl::PointCloud<PointT>());
            std::vector<int> idx_ff;
            pcl::removeNaNFromPointCloud(*full_cloud, *tmp_cloud, idx_ff);

            pcl::PassThrough<PointT> pass;
            pass.setInputCloud (tmp_cloud);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (0.1, 1.5);
            //pass.setFilterLimitsNegative (true);
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
            pass.filter (*cloud);
            
            viewer->removeAllPointClouds();
            viewer->addPointCloud(cloud, "full");
            viewer->spin();
            continue;*/