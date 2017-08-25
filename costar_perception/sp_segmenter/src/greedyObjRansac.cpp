#include "sp_segmenter/greedyObjRansac.h"

//greedyObjRansac::greedyObjRansac(double pairWidth_, double voxelSize_) : objrec(pairWidth_, voxelSize_, 1.0)
greedyObjRansac::greedyObjRansac(double pairWidth_, double voxelSize_, double relNumOfPairsInHashTable_) : objrec(pairWidth_, voxelSize_, relNumOfPairsInHashTable_),
    min_confidence(0.0)
{
//    successProbability = 0.9999999;
    successProbability = 0.99;
    voxelSize = voxelSize_;
    
    visibility = 0.1;
    relativeObjSize = 0.1;
    
    pairWidth = pairWidth_;
    
    srand(time(NULL));
}

void greedyObjRansac::setParams(double vis_, double rel_)
{
    visibility = vis_;
    relativeObjSize = rel_;
}

poseT greedyObjRansac::getBestModel(list< boost::shared_ptr<PointSetShape> >& detectedShapes)
{
    double max_confidence = -1;
    poseT new_pose;
    std::string model_name;
    for ( list< boost::shared_ptr<PointSetShape> >::iterator it = detectedShapes.begin() ; it != detectedShapes.end() ; ++it )
    {
        boost::shared_ptr<PointSetShape> shape = (*it);
        
        if( shape->getConfidence() > max_confidence )
        {
            max_confidence = shape->getConfidence();
            model_name = shape->getUserData()->getLabel();
            
            double **mat4x4 = mat_alloc(4, 4);
            shape->getHomogeneousRigidTransform(mat4x4);
            
            new_pose.model_name = std::string(shape->getUserData()->getLabel());
            Eigen::Matrix3f rot;
            rot << mat4x4[0][0], mat4x4[0][1], mat4x4[0][2], 
                   mat4x4[1][0], mat4x4[1][1], mat4x4[1][2], 
                   mat4x4[2][0], mat4x4[2][1], mat4x4[2][2];

            new_pose.shift = Eigen::Vector3f (mat4x4[0][3], mat4x4[1][3], mat4x4[2][3]);
            mat_dealloc(mat4x4, 4);

            new_pose.rotation = rot;
            new_pose.confidence = max_confidence;
        }
    }
    
    return new_pose;
}

poseT greedyObjRansac::recognizeOne(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, pcl::PointCloud<myPointXYZ>::Ptr &rest_cloud)
{
    vtkSmartPointer<vtkPolyData> vtk_scene = PolyDataFromPointCloud(scene_xyz);
    vtkPoints* scene = vtk_scene->GetPoints();
    
    //list<PointSetShape*> detectedObjects;
    list< boost::shared_ptr<PointSetShape> > detectedObjects;
    objrec.doRecognition(scene, successProbability, detectedObjects);
    
    //vtk_scene->Delete();
    
    if( detectedObjects.empty() == true )
    {
        poseT dummy_pose;
        return dummy_pose;
    }

    poseT new_pose = getBestModel(detectedObjects);
    
    pcl::PointCloud<myPointXYZ>::Ptr trans_model(new pcl::PointCloud<myPointXYZ>());
    for( int i = 0 ; i < models.size() ; i++ )
    {
        if(models[i].model_label == new_pose.model_name )
        {
            pcl::transformPointCloud(*models[i].model_cloud, *trans_model, new_pose.shift, new_pose.rotation);
            break;
        } 
    }
    
    //for ( list< boost::shared_ptr<PointSetShape> >::iterator it = detectedObjects.begin() ; it != detectedObjects.end() ; ++it )
    //    delete *it;
    
    rest_cloud = FilterCloud(scene_xyz, trans_model);
    return new_pose;
}

void greedyObjRansac::GreedyRecognize(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, std::vector<poseT> &poses)
{
    poses.clear();
    pcl::PointCloud<myPointXYZ>::Ptr cur_scene = scene_xyz;

    int iter = 0;
    min_confidence = 0.;

    while(true)
    {
        std::cerr<< "Recognizing Attempt --- " << iter << std::endl;
        std::cerr << "Scene point cloud size: " << cur_scene->size() << std::endl;
        pcl::PointCloud<myPointXYZ>::Ptr filtered_scene(new pcl::PointCloud<myPointXYZ>());
        poseT new_pose = recognizeOne(cur_scene, filtered_scene);
        
        if( filtered_scene->empty() == true )
        {
            std::cerr << "Iteration #" << iter << ": No object detected anymore from this point cloud.\n";
            break;
        }

        poses.push_back(new_pose);
        cur_scene = filtered_scene;
        iter++;

    }
    std::cerr<< "Recognizing Done!!!" << std::endl;

}

void greedyObjRansac::StandardBest(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, std::vector<poseT> &poses)
{
    vtkSmartPointer<vtkPolyData> vtk_scene = PolyDataFromPointCloud(scene_xyz);
    vtkPoints* scene = vtk_scene->GetPoints();
    //vtkPoints* scene = PolyDataFromPointCloud(scene_xyz);
    //list<PointSetShape*> detectedObjects;
    list< boost::shared_ptr<PointSetShape> > detectedObjects;
    objrec.doRecognition(scene, successProbability, detectedObjects);
    
    float max = -1000;
    boost::shared_ptr<PointSetShape> best_shape;
    
    min_confidence = 0.;

    for ( list< boost::shared_ptr<PointSetShape> >::iterator it = detectedObjects.begin() ; it != detectedObjects.end() ; ++it )
    {
//        if ( (*it)->getUserData() )
//            printf("\t%s, confidence: %lf\n", (*it)->getUserData()->getLabel(), (*it)->getConfidence());
        if ( (*it)->getConfidence() >= 0.0 && (*it)->getConfidence() > max )
        {
            max = (*it)->getConfidence();
            best_shape = *it;
        }
    }
    if( max > 0 )
    {
        double **mat4x4 = mat_alloc(4, 4);
        best_shape->getHomogeneousRigidTransform(mat4x4);

        poseT new_pose;
        new_pose.model_name = std::string(best_shape->getUserData()->getLabel());
        Eigen::Matrix3f rot;
        rot << mat4x4[0][0], mat4x4[0][1], mat4x4[0][2], 
               mat4x4[1][0], mat4x4[1][1], mat4x4[1][2], 
               mat4x4[2][0], mat4x4[2][1], mat4x4[2][2];

        new_pose.shift = Eigen::Vector3f (mat4x4[0][3], mat4x4[1][3], mat4x4[2][3]);
        mat_dealloc(mat4x4, 4);
        new_pose.rotation = rot;
        new_pose.confidence = best_shape -> getConfidence();
        poses.push_back(new_pose);
    }
}

void greedyObjRansac::StandardRecognize(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, std::vector<poseT> &poses, double minConfidence)
{
    vtkSmartPointer<vtkPolyData> vtk_scene = PolyDataFromPointCloud(scene_xyz);
    vtkPoints* scene = vtk_scene->GetPoints();
    //vtkPoints* scene = PolyDataFromPointCloud(scene_xyz);
    
    //list<PointSetShape*> detectedObjects;
    list< boost::shared_ptr<PointSetShape> > detectedObjects;
    objrec.doRecognition(scene, successProbability, detectedObjects);
    min_confidence = minConfidence;

    for ( list< boost::shared_ptr<PointSetShape> >::iterator it = detectedObjects.begin() ; it != detectedObjects.end() ; ++it )
    {
        boost::shared_ptr<PointSetShape> shape = (*it);

        if (shape->getConfidence() < minConfidence){
            // printf("Skipping shape, confidence too low\n");
            std::cerr << "Skipping shape: " << shape->getUserData()->getLabel() << " confidence: " << shape->getConfidence() <<" is too low\n";
            continue;
        }
    	else std::cerr << shape->getUserData()->getLabel() << " confidence: " << shape->getConfidence() << std::endl;
        double **mat4x4 = mat_alloc(4, 4);
        shape->getHomogeneousRigidTransform(mat4x4);
        
        poseT new_pose;
        new_pose.model_name = std::string(shape->getUserData()->getLabel());
        Eigen::Matrix3f rot;
        rot << mat4x4[0][0], mat4x4[0][1], mat4x4[0][2], 
               mat4x4[1][0], mat4x4[1][1], mat4x4[1][2], 
               mat4x4[2][0], mat4x4[2][1], mat4x4[2][2];

        new_pose.shift = Eigen::Vector3f (mat4x4[0][3], mat4x4[1][3], mat4x4[2][3]);
        new_pose.rotation = rot;
        new_pose.confidence = shape->getConfidence();
        /*
        new_pose.tran << mat4x4[0][0], mat4x4[0][1], mat4x4[0][2], mat4x4[0][3],
                 mat4x4[1][0], mat4x4[1][1], mat4x4[1][2], mat4x4[1][3],
                 mat4x4[2][0], mat4x4[2][1], mat4x4[2][2], mat4x4[2][3],
                 mat4x4[3][0], mat4x4[3][1], mat4x4[3][2], mat4x4[3][3];
        new_pose.model_name = std::string (shape->getUserData()->getLabel());
        */
        mat_dealloc(mat4x4, 4);
        poses.push_back(new_pose);
    }   
    
    //vtk_scene->GetPoints()->Delete();
    //for ( list< boost::shared_ptr<PointSetShape> >::iterator it = detectedObjects.begin() ; it != detectedObjects.end() ; ++it )
    //    delete *it;
}

/// @todo make this data be loaded somewhere totally different, then just pass in the relevant objects.
void greedyObjRansac::AddModel(std::string name, std::string label)
{
    // Construct Object Ransac Model
    //objrec = ObjRecRANSAC (pairWidth, voxelSize, 0.5/*leave this one like this*/);
    
    UserData* userData;
    userData = new UserData();
    userData->setLabel(label.c_str()); // Just set an 'Amicelli' label
    
    vtkPolyDataReader* reader = vtkPolyDataReader::New();
    reader->SetFileName((name+".vtk").c_str());
    reader->Update();
    // Add the model to the model library
    objrec.addModel(reader->GetOutput(), userData);
    objrec.setVisibility(visibility);
    objrec.setRelativeObjectSize(relativeObjSize);
    objrec.setNumberOfThreads(8);
    //delete userData;
    
    models.push_back(LoadMesh(name,label));
}

void greedyObjRansac::visualize(pcl::visualization::PCLVisualizer::Ptr viewer, const std::vector<poseT> &poses, uchar color[3])
{
    std::vector<pcl::PointCloud<myPointXYZ>::Ptr> rec;
    for( std::vector<ModelT>::iterator it = models.begin() ; it < models.end() ; it++ )
    {
        pcl::PointCloud<myPointXYZ>::Ptr cur_cloud(new pcl::PointCloud<myPointXYZ>()); 
        pcl::fromPCLPointCloud2(it->model_mesh->cloud, *cur_cloud);
        rec.push_back(cur_cloud);
    }
    
    int count = 0;
    for(std::vector<poseT>::const_iterator it = poses.begin() ; it < poses.end() ; it++, count++ )
    {
        for( int i = 0 ; i < models.size() ; i++ )
        {
            if(models[i].model_label == it->model_name )
            {
                pcl::PointCloud<myPointXYZ>::Ptr cur_cloud(new pcl::PointCloud<myPointXYZ>()); 
                pcl::transformPointCloud(*rec[i], *cur_cloud, it->shift, it->rotation);

                std::stringstream ss;
                ss << count;

                viewer->addPolygonMesh<myPointXYZ>(cur_cloud, models[i].model_mesh->polygons, it->model_name+"_"+ss.str());
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color[0]/255.0, color[1]/255.0, color[2]/255.0, it->model_name+"_"+ss.str());
                //if( it->model_name == "1" )
                //    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.55, 0.05, it->model_name+"_"+ss.str());
                //else if( it->model_name == "2" )
                //    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.05, 0.55, 1.0, it->model_name+"_"+ss.str());
                break;
            }
        }
        
    }
}

void greedyObjRansac::visualize_m(pcl::visualization::PCLVisualizer::Ptr viewer, const std::vector<poseT> &poses, std::map<std::string, std::size_t> &model_name_map, uchar model_color[11][3])
{
    std::vector<pcl::PointCloud<myPointXYZ>::Ptr> rec;
    for( std::vector<ModelT>::iterator it = models.begin() ; it < models.end() ; it++ )
    {
        pcl::PointCloud<myPointXYZ>::Ptr cur_cloud(new pcl::PointCloud<myPointXYZ>()); 
        pcl::fromPCLPointCloud2(it->model_mesh->cloud, *cur_cloud);
        rec.push_back(cur_cloud);
    }
    
    int count = 0;
    for(std::vector<poseT>::const_iterator it = poses.begin() ; it < poses.end() ; it++, count++ )
    {
        for( int i = 0 ; i < models.size() ; i++ )
        {
            if(models[i].model_label == it->model_name )
            {
                pcl::PointCloud<myPointXYZ>::Ptr cur_cloud(new pcl::PointCloud<myPointXYZ>()); 
                pcl::transformPointCloud(*rec[i], *cur_cloud, it->shift, it->rotation);

                std::stringstream ss;
                ss << count;

                viewer->addPolygonMesh<myPointXYZ>(cur_cloud, models[i].model_mesh->polygons, it->model_name+"_"+ss.str());
                std::size_t id = model_name_map[it->model_name];
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, model_color[id][0]/255.0, model_color[id][1]/255.0, model_color[id][2]/255.0, it->model_name+"_"+ss.str());
                //if( it->model_name == "1" )
                //    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.55, 0.05, it->model_name+"_"+ss.str());
                //else if( it->model_name == "2" )
                //    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.05, 0.55, 1.0, it->model_name+"_"+ss.str());
                break;
            }
        }
        
    }
}

void greedyObjRansac::clearMesh(pcl::visualization::PCLVisualizer::Ptr viewer, const std::vector<poseT> &poses)
{
    int count = 0;
    for(std::vector<poseT>::const_iterator it = poses.begin() ; it < poses.end() ; it++, count++ )
    {
        for( int i = 0 ; i < models.size() ; i++ )
        {
            if(models[i].model_label == it->model_name )
            {
                std::stringstream ss;
                ss << count;

                viewer->removePolygonMesh(it->model_name+"_"+ss.str());
            }
        }
        
    }
}

void greedyObjRansac::ICP(std::vector<poseT> &poses, const pcl::PointCloud<myPointXYZ>::Ptr scene)
{
    if( poses.empty() == true || scene->empty() == true )
        return;
    std::vector<pcl::PointCloud<myPointXYZ>::Ptr> rec;
    for( std::vector<ModelT>::iterator it = models.begin() ; it < models.end() ; it++ )
    {
        pcl::PointCloud<myPointXYZ>::Ptr cur_cloud(new pcl::PointCloud<myPointXYZ>()); 
        pcl::fromPCLPointCloud2(it->model_mesh->cloud, *cur_cloud);
        rec.push_back(cur_cloud);
    }
    
    pcl::IterativeClosestPoint<myPointXYZ, myPointXYZ> icp;
    icp.setInputTarget(scene);
    
    for(std::vector<poseT>::iterator it = poses.begin() ; it < poses.end() ; it++ )
    {
        for( int i = 0 ; i < models.size() ; i++ )
        {
            if(models[i].model_label == it->model_name )
            {
//                pcl::PointCloud<myPointXYZ>::Ptr cur_cloud(new pcl::PointCloud<myPointXYZ>()); 
//                pcl::transformPointCloud(*rec[i], *cur_cloud, it->shift, it->rotation);
                Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
                guess(0, 3) = it->shift(0);
                guess(1, 3) = it->shift(1);
                guess(2, 3) = it->shift(2);
                guess.block<3,3>(0,0) = it->rotation.toRotationMatrix();
                
                pcl::PointCloud<myPointXYZ>::Ptr Final(new pcl::PointCloud<myPointXYZ>());

#if PCL_VERSION_COMPARE(<, 1, 7, 0)
                icp.setInputCloud(rec[i]);
#else
                icp.setInputSource(rec[i]);
#endif
                icp.align(*Final, guess);
                
                Eigen::Matrix4f tran = icp.getFinalTransformation();
                it->shift(0) = tran(0, 3);
                it->shift(1) = tran(1, 3);
                it->shift(2) = tran(2, 3);
                
                Eigen::Matrix3f rot = tran.block<3,3>(0,0);
                it->rotation = Eigen::Quaternion<float> (rot);
                
                break;
            }
        }
        
    }
    
}

void greedyObjRansac::genHypotheses(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, list<AcceptedHypothesis> &acc_hypotheses)
{
    vtkSmartPointer<vtkPolyData> vtk_scene = PolyDataFromPointCloud(scene_xyz);
    // vtkPoints* scene = vtk_scene->GetPoints();
    
    //vtkPoints* scene = PolyDataFromPointCloud(scene_xyz);
    
    list<AcceptedHypothesis> cur_hypotheses;
    //int flag = objrec.getHypotheses(scene, successProbability, cur_hypotheses);
    
    for ( list<AcceptedHypothesis>::iterator it = cur_hypotheses.begin() ; it != cur_hypotheses.end() ; ++it )
        acc_hypotheses.push_back(*it);
    //acc_shapes.insert(acc_shapes.end(), cur_shapes.begin(), cur_shapes.end());
    //cur_hypo_set.clear();
}

void greedyObjRansac::mergeHypotheses(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, list<AcceptedHypothesis> &acc_hypotheses, std::vector<poseT> &poses)
{
    vtkSmartPointer<vtkPolyData> vtk_scene = PolyDataFromPointCloud(scene_xyz);
    // vtkPoints* scene = vtk_scene->GetPoints();
    
    //vtkPoints* scene = PolyDataFromPointCloud(scene_xyz);
    
    list<PointSetShape*> detectedObjects;
    //objrec.FilterHypotheses(scene, acc_hypotheses, detectedObjects);
    
    for ( list<PointSetShape*>::iterator it = detectedObjects.begin() ; it != detectedObjects.end() ; ++it )
    {
        PointSetShape* shape = (*it);
        std::string instance_name(shape->getUserData()->getLabel());
        //if ( shape->getUserData() )
        //    printf("\t%s, confidence: %lf\n", instance_name.c_str(), shape->getConfidence());
        
        if( (instance_name == "link" && shape->getConfidence() > 0.1) ||
            (instance_name == "node" && shape->getConfidence() > 0.1) )
        {
            double **mat4x4 = mat_alloc(4, 4);
            shape->getHomogeneousRigidTransform(mat4x4);

            poseT new_pose;
            new_pose.model_name = instance_name;
            Eigen::Matrix3f rot;
            rot << mat4x4[0][0], mat4x4[0][1], mat4x4[0][2], 
                   mat4x4[1][0], mat4x4[1][1], mat4x4[1][2], 
                   mat4x4[2][0], mat4x4[2][1], mat4x4[2][2];

            new_pose.shift = Eigen::Vector3f (mat4x4[0][3], mat4x4[1][3], mat4x4[2][3]);
            new_pose.rotation = rot;
            /*
            new_pose.tran << mat4x4[0][0], mat4x4[0][1], mat4x4[0][2], mat4x4[0][3],
                     mat4x4[1][0], mat4x4[1][1], mat4x4[1][2], mat4x4[1][3],
                     mat4x4[2][0], mat4x4[2][1], mat4x4[2][2], mat4x4[2][3],
                     mat4x4[3][0], mat4x4[3][1], mat4x4[3][2], mat4x4[3][3];
            new_pose.model_name = instance_name;
            */
            mat_dealloc(mat4x4, 4);
            poses.push_back(new_pose);
        }
    }   
    
    //vtk_scene->GetPoints()->Delete();
    for ( list<PointSetShape*>::iterator it = detectedObjects.begin() ; it != detectedObjects.end() ; ++it )
        delete *it;
}

pcl::PointCloud<myPointXYZ>::Ptr greedyObjRansac::FillModelCloud(const std::vector<poseT> &poses)
{
    pcl::PointCloud<myPointXYZ>::Ptr model_cloud(new pcl::PointCloud<myPointXYZ>());
    for(std::vector<poseT>::const_iterator it = poses.begin() ; it < poses.end() ; it++ )
    {
        for( int i = 0 ; i < models.size() ; i++ )
        {
            if(models[i].model_label == it->model_name )
            {
                pcl::PointCloud<myPointXYZ>::Ptr cur_cloud(new pcl::PointCloud<myPointXYZ>()); 
                pcl::transformPointCloud(*models[i].model_cloud, *cur_cloud, it->shift, it->rotation);
                
                model_cloud->insert(model_cloud->end(), cur_cloud->begin(), cur_cloud->end());
                break;
            }
        }
    }
    return model_cloud;
}

void greedyObjRansac::getPairFeas(const pcl::PointCloud<myPointXYZ>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normals, list<ObjRecRANSAC::OrientedPair> &PairFeas, float maxDist, int num)
{
    float sqr_maxDist = maxDist * maxDist;
    int cloud_size = cloud->size();
    
    int iter = 0;
    int count = 0;
    while(true)
    {
        int idx1 = rand() % cloud_size;
        int idx2;
        int iter_count = 0;
        while(true)
        {
            idx2 = rand() % cloud_size;
            float dist = sqrDistPt(cloud->at(idx1), cloud->at(idx2));
            if( fabs( dist - sqr_maxDist ) < 0.000001 )
            {
                float *p1 = cloud->at(idx1).data;
                float *p2 = cloud->at(idx2).data;
                
                float *n1 = cloud_normals->at(idx1).data_n;
                float *n2 = cloud_normals->at(idx2).data_n;
                
                ObjRecRANSAC::OrientedPair tmp;
                tmp.p1[0] = p1[0];tmp.p1[1] = p1[1];tmp.p1[2] = p1[2];
                tmp.p2[0] = p2[0];tmp.p2[1] = p2[1];tmp.p2[2] = p2[2];
                
                tmp.n1[0] = n1[0];tmp.n1[1] = n1[1];tmp.n1[2] = n1[2];
                tmp.n2[0] = n2[0];tmp.n2[1] = n2[1];tmp.n2[2] = n2[2];
                
                PairFeas.push_back(tmp);
                count++;
                break;
            }
            iter_count++;
            iter++;
            if( iter_count > 10 )
                break;
            if( iter > cloud_size*cloud_size/4 )
                return;
            
        }
        if( count >= num )
            break;
    }
}

#if SCENE_PARSING
GreedyHypothesis greedyObjRansac::getLatestAcceptedHypothesis(const bool &combined_ransac)
{
    std::vector<std::vector<AcceptedHypothesisWithConfidence> > acc_hypothesis = objrec.getShapeHypothesis();
    GreedyHypothesis result;
    if (acc_hypothesis.size() > 0)
    {
        // result.hypothesis = acc_hypothesis;
        for (std::size_t i = 0; i < acc_hypothesis.size(); ++i)
        {
            if (acc_hypothesis[i].size() > 0)
            {
                if (!combined_ransac) result.model_id = acc_hypothesis[0][0].model_entry->getUserData()->getLabel();
                if (combined_ransac) result.model_id = "combined_ransac(UNSUPPORTED)"; 
                break;
            }
        }
        
        // separate the object hypothesis by its best position
        for (std::size_t i = 0; i < acc_hypothesis.size(); ++i)
        {
            if (acc_hypothesis[i].size() == 0) continue;
             
            // skip object that has been skipped by the pose estimation
            if (acc_hypothesis[i][0].confidence < min_confidence) continue;

            // object index starts from 1, not zero.
            std::size_t object_tf_index = i + 1;
            result.by_object_hypothesis[object_tf_index] = acc_hypothesis[i];
        }
    }

    return result;
}
#endif


