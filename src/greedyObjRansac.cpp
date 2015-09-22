#include "greedyObjRansac.h"

//greedyObjRansac::greedyObjRansac(double pairWidth_, double voxelSize_) : objrec(pairWidth_, voxelSize_, 1.0)
greedyObjRansac::greedyObjRansac(double pairWidth_, double voxelSize_) : objrec(pairWidth_, voxelSize_, 0.5)
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
        
        //if ( shape->getUserData() )
        //    printf("\t%s, confidence: %lf\n", shape->getUserData()->getLabel(), shape->getConfidence());
        
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
            new_pose.rotation = rot;
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
    while(true)
    {
        //std::cerr<< "Recognizing Attempt --- " << iter << std::endl;
        pcl::PointCloud<myPointXYZ>::Ptr filtered_scene(new pcl::PointCloud<myPointXYZ>());
        poseT new_pose = recognizeOne(cur_scene, filtered_scene);
        
        if( filtered_scene->empty() == true )
            break;
        
        poses.push_back(new_pose);
        cur_scene = filtered_scene;
        iter++;
    }
    //std::cerr<< "Recognizing Done!!!" << std::endl;

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
        new_pose.rotation = rot;
        poses.push_back(new_pose);
    }
}

void greedyObjRansac::StandardRecognize(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, std::vector<poseT> &poses)
{
    vtkSmartPointer<vtkPolyData> vtk_scene = PolyDataFromPointCloud(scene_xyz);
    vtkPoints* scene = vtk_scene->GetPoints();
    //vtkPoints* scene = PolyDataFromPointCloud(scene_xyz);
    
    //list<PointSetShape*> detectedObjects;
    list< boost::shared_ptr<PointSetShape> > detectedObjects;
    objrec.doRecognition(scene, successProbability, detectedObjects);
    
    for ( list< boost::shared_ptr<PointSetShape> >::iterator it = detectedObjects.begin() ; it != detectedObjects.end() ; ++it )
    {
        boost::shared_ptr<PointSetShape> shape = (*it);
        if ( shape->getUserData() )
            printf("\t%s, confidence: %lf\n", shape->getUserData()->getLabel(), shape->getConfidence());
        
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
        /*
        new_pose.tran << mat4x4[0][0], mat4x4[0][1], mat4x4[0][2], mat4x4[0][3],
                 mat4x4[1][0], mat4x4[1][1], mat4x4[1][2], mat4x4[1][3],
                 mat4x4[2][0], mat4x4[2][1], mat4x4[2][2], mat4x4[2][3],
                 mat4x4[3][0], mat4x4[3][1], mat4x4[3][2], mat4x4[3][3];
        new_pose.model_name = std::string (shape->getUserData()->getLabel());
        */
        poses.push_back(new_pose);
    }   
    
    //vtk_scene->GetPoints()->Delete();
    //for ( list< boost::shared_ptr<PointSetShape> >::iterator it = detectedObjects.begin() ; it != detectedObjects.end() ; ++it )
    //    delete *it;
}

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
    
    // Construct Polygon Mesh
    ModelT cur_model;
    cur_model.model_mesh = pcl::PolygonMesh::Ptr (new pcl::PolygonMesh()); 
    if( exists_test(name+".obj") == true )
        pcl::io::loadPolygonFile(name+".obj", *cur_model.model_mesh); 
    else if( exists_test(name+".stl") == true )
        pcl::io::loadPolygonFile(name+".stl", *cur_model.model_mesh);
    else
    {
        std::cerr << "No OBJ or STL file!" << std::endl;
        exit(0);
    }
    
    pcl::PointCloud<myPointXYZ>::Ptr cloud(new pcl::PointCloud<myPointXYZ>()); 
    pcl::fromPCLPointCloud2(cur_model.model_mesh->cloud, *cloud);

    cur_model.model_cloud = pcl::PointCloud<myPointXYZ>::Ptr (new pcl::PointCloud<myPointXYZ>()); 
    pcl::VoxelGrid<myPointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.004, 0.004, 0.004);
    sor.filter(*cur_model.model_cloud);

    cur_model.model_label = label;
    models.push_back(cur_model);
}

void greedyObjRansac::visualize(pcl::visualization::PCLVisualizer::Ptr viewer, const std::vector<poseT> &poses, int color[3])
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

void greedyObjRansac::visualize_m(pcl::visualization::PCLVisualizer::Ptr viewer, const std::vector<poseT> &poses, std::map<std::string, int> &model_name_map, uchar model_color[11][3])
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
                int id = model_name_map[it->model_name];
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
                icp.setInputCloud(rec[i]);
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
    vtkPoints* scene = vtk_scene->GetPoints();
    
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
    vtkPoints* scene = vtk_scene->GetPoints();
    
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


/*******************************************************************************************************************************************************************/

/*
void greedyObjRansac::AdvancedRecognize(const pcl::PointCloud<PointT>::Ptr scene, std::vector<poseMessage> &poses)
{
    pcl::PointCloud<myPointXYZ>::Ptr full_cloud(new pcl::PointCloud<myPointXYZ>());
    pcl::copyPointCloud(*scene, *full_cloud);
    
    pcl::PointCloud<myPointXYZ>::Ptr link_cloud(new pcl::PointCloud<myPointXYZ>());
    pcl::PointCloud<myPointXYZ>::Ptr node_cloud(new pcl::PointCloud<myPointXYZ>());
    splitCloud(scene, link_cloud, node_cloud);
    
    pcl::PointCloud<myPointXYZ>::Ptr down_link_cloud(new pcl::PointCloud<myPointXYZ>());
    pcl::PointCloud<myPointXYZ>::Ptr down_node_cloud(new pcl::PointCloud<myPointXYZ>());
    
    //down_link_cloud = link_cloud;
    //down_node_cloud = node_cloud;
    
    pcl::VoxelGrid<myPointXYZ> sor;
    sor.setLeafSize(0.005, 0.005, 0.005);
    sor.setInputCloud(link_cloud);
    sor.filter(*down_link_cloud);
    sor.setInputCloud(node_cloud);
    sor.filter(*down_node_cloud);
    
    pcl::NormalEstimationOMP<myPointXYZ, NormalT> nest;
    pcl::search::KdTree<myPointXYZ>::Ptr tree(new pcl::search::KdTree<myPointXYZ>);
    nest.setSearchMethod (tree);
    nest.setSearchSurface(full_cloud);
    nest.setRadiusSearch(0.03);
    
    pcl::PointCloud<NormalT>::Ptr down_link_normals(new pcl::PointCloud<NormalT>());
    pcl::PointCloud<NormalT>::Ptr down_node_normals(new pcl::PointCloud<NormalT>());
    
    nest.setInputCloud (down_link_cloud);
    nest.compute(*down_link_normals);
    nest.setInputCloud (down_node_cloud);
    nest.compute(*down_node_normals);
    
    std::cerr << link_cloud->size() << " "<< node_cloud->size() << std::endl;
    std::cerr << down_link_cloud->size() << " "<< down_node_cloud->size() << std::endl;
    
    list<ObjRecRANSAC::OrientedPair> PairFeas;
    std::cerr << "Pair Features: " << PairFeas.size() << std::endl;
    getPairFeas(down_link_cloud, down_link_normals, PairFeas, pairWidth, down_link_cloud->size());
    std::cerr << "Pair Features: " << PairFeas.size() << std::endl;
    getPairFeas(down_node_cloud, down_node_normals, PairFeas, pairWidth, down_node_cloud->size());
    std::cerr << "Pair Features: " << PairFeas.size() << std::endl;
    
    vtkSmartPointer<vtkPolyData> vtk_scene = PolyDataFromPointCloud(full_cloud);
    vtkPoints* scene_ptr = vtk_scene->GetPoints();
    
    list<PointSetShape*> detectedObjects;
    objrec.doRecognition(scene_ptr, PairFeas, detectedObjects);
    
    for ( list<PointSetShape*>::iterator it = detectedObjects.begin() ; it != detectedObjects.end() ; ++it )
    {
        PointSetShape* shape = (*it);
        if ( shape->getUserData() )
            printf("\t%s, confidence: %lf\n", shape->getUserData()->getLabel(), shape->getConfidence());
        
        double **mat4x4 = mat_alloc(4, 4);
        shape->getHomogeneousRigidTransform(mat4x4);
        
        poseMessage new_pose;
        new_pose.tran << mat4x4[0][0], mat4x4[0][1], mat4x4[0][2], mat4x4[0][3],
                 mat4x4[1][0], mat4x4[1][1], mat4x4[1][2], mat4x4[1][3],
                 mat4x4[2][0], mat4x4[2][1], mat4x4[2][2], mat4x4[2][3],
                 mat4x4[3][0], mat4x4[3][1], mat4x4[3][2], mat4x4[3][3];
        new_pose.model_name = std::string (shape->getUserData()->getLabel());
        
        poses.push_back(new_pose);
    }   
    
    //vtk_scene->GetPoints()->Delete();
    for ( list<PointSetShape*>::iterator it = detectedObjects.begin() ; it != detectedObjects.end() ; ++it )
        delete *it;
}


void greedyObjRansac::sortCloud(pcl::PointCloud<PointT>::Ptr cloud)
{
    std::sort(cloud->begin(), cloud->end(), mycomp);
}

segT greedyObjRansac::regionGrowing(pcl::PointCloud<PointT>::Ptr cloud, float confT)
{
    int num = cloud->size();
    std::vector<bool> flags(num, false);
    
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
    kdtree->setInputCloud(cloud);
    
    float radius = 0.02;
    segT segment;
    segment.cloud = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>());
    
    for( int i = 0 ; i < num ; i++ )
    {
        if( flags[i] == true )
            continue;
        
        float score = *reinterpret_cast<float*>(&(cloud->at(i).rgba));
        if( fabs(score) >= confT )
        {
            // create a new segment
            float sign = score > 0 ? 1.0 : -1.0;
        
            std::vector<int> queue;
            queue.push_back(i);
            while(true)
            {
                if( queue.empty() == true )
                    break;
                int cur_idx = queue[0];
                segment.indices.push_back(cur_idx);
                segment.cloud->push_back(cloud->at(cur_idx));
                queue.erase(queue.begin());

                std::vector<int> neighIdx;
                std::vector<float> neighDist;
                if(kdtree->radiusSearch(cur_idx, radius, neighIdx, neighDist) > 0)
                {
                    for(std::vector<int>::iterator it = neighIdx.begin(); it < neighIdx.end() ; it++)
                    {
                        float tmp_score = *reinterpret_cast<float*>(&(cloud->at(*it).rgba));
                        if ( flags[*it] == false && sign*tmp_score >= confT )
                        {
                            flags[*it] = true;
                            queue.push_back(*it);
                        }
                    }
                }
            }
            return segment;
        }
      
    }
    return segment;
}

void greedyObjRansac::getAllPairFeas(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normals, list<ObjRecRANSAC::OrientedPair> &PairFeas, float maxDist)
{
    float sqr_maxDist = maxDist * maxDist;
    int num = cloud->size();
    
    for(int i = 0 ; i < num ; i++ ){
        for(int j = i + 1 ; j < num ; j++ ){
            
            float dist = sqrDistPtT(cloud->at(i), cloud->at(j));
            if( fabs( dist - sqr_maxDist ) < 0.000001 )
            {
                float *p1 = cloud->at(i).data;
                float *p2 = cloud->at(j).data;
                
                float *n1 = cloud_normals->at(i).data_n;
                float *n2 = cloud_normals->at(j).data_n;
                
                ObjRecRANSAC::OrientedPair tmp;
                tmp.p1[0] = p1[0];tmp.p1[1] = p1[1];tmp.p1[2] = p1[2];
                tmp.p2[0] = p2[0];tmp.p2[1] = p2[1];tmp.p2[2] = p2[2];
                
                tmp.n1[0] = n1[0];tmp.n1[1] = n1[1];tmp.n1[2] = n1[2];
                tmp.n2[0] = n2[0];tmp.n2[1] = n2[1];tmp.n2[2] = n2[2];
                
                PairFeas.push_back(tmp);
            }
        }
    }
}

pcl::PointCloud<PointT>::Ptr greedyObjRansac::FilterCloud(const pcl::PointCloud<PointT>::Ptr scene, const std::vector<poseMessage> &poses, float T)
{
    pcl::PointCloud<myPointXYZ>::Ptr tran_cloud(new pcl::PointCloud<myPointXYZ>()); 
    for(std::vector<poseMessage>::const_iterator it = poses.begin() ; it < poses.end() ; it++ )
    {
        for(int i = 0 ; i < models.size() ; i++)
        {
            if(models[i].model_label == it->model_name )
            {
                pcl::PointCloud<myPointXYZ>::Ptr cur_cloud(new pcl::PointCloud<myPointXYZ>()); 
                pcl::transformPointCloud(*models[i].model_cloud, *cur_cloud, it->tran);
                
                tran_cloud->insert(tran_cloud->end(), cur_cloud->begin(), cur_cloud->end());
            }
        }
        
    }
    
    pcl::search::KdTree<myPointXYZ> tree;
    tree.setInputCloud (tran_cloud);

    pcl::PointCloud<PointT>::Ptr filtered_scene(new pcl::PointCloud<PointT>());
    for(pcl::PointCloud<PointT>::const_iterator it = scene->begin() ; it < scene->end() ; it++ )
    {
        std::vector<int> indices (1);
        std::vector<float> sqr_distances (1);
        int nres = tree.nearestKSearchT(*it, 1, indices, sqr_distances);
        if ( sqrt(sqr_distances[0]) > T )
            filtered_scene->push_back(*it);
    }
    return filtered_scene;
}

void greedyObjRansac::newRecognize(pcl::PointCloud<PointT>::Ptr scene, std::vector<poseMessage> &poses, float confT, pcl::visualization::PCLVisualizer::Ptr viewer)
{
    sortCloud(scene);
    
    pcl::PointCloud<myPointXYZ>::Ptr scene_xyz(new pcl::PointCloud<myPointXYZ>());
    viewer->addPointCloud(scene_xyz, "scene_xyz");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5, "scene_xyz");
    
    poses.clear();
    pcl::PointCloud<PointT>::Ptr cur_scene = scene;
    int iter = 0;
    
    pcl::VoxelGrid<PointT> sor;
    sor.setLeafSize(0.004, 0.004, 0.004);
    
    pcl::NormalEstimationOMP<PointT, NormalT> nest;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    nest.setSearchMethod (tree);
    nest.setSearchSurface(scene);
    nest.setRadiusSearch(0.03);
    
    vtkSmartPointer<vtkPolyData> vtk_scene = PolyDataFromPointCloud(scene);
    vtkPoints* scene_ptr = vtk_scene->GetPoints();
    
    while(true)
    {
        std::cerr<< "Recognizing Attempt --- " << iter << std::endl;
        
        segT cur_seg = regionGrowing(cur_scene, confT);
        //viewer->addPointCloud(cur_seg.cloud, "cur_seg");
        //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cur_seg");
        //viewer->spin();
        //viewer->removePointCloud("cur_seg");
        
        std::cerr << "Seg: " << cur_seg.cloud->size() << std::endl;
        if( cur_seg.cloud->empty() == true )
            break;
        if( cur_seg.cloud->size() <= 100 )
        {
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
            inliers->indices = cur_seg.indices;
            // Create the filtering object
            pcl::ExtractIndices<PointT> extract;
             // Extract the inliers
            extract.setInputCloud (cur_scene);
            extract.setIndices (inliers);
            extract.setNegative (true);
            extract.filter (*cur_scene);
            continue;
        }
            
        pcl::PointCloud<PointT>::Ptr down_seg(new pcl::PointCloud<PointT>());
        
        sor.setInputCloud(cur_seg.cloud);
        sor.filter(*down_seg);
        std::cerr << "Down-Seg: " << down_seg->size() << std::endl;
        
        pcl::PointCloud<NormalT>::Ptr down_seg_normals(new pcl::PointCloud<NormalT>());
        nest.setInputCloud(down_seg);
        nest.compute(*down_seg_normals);
        
        list<ObjRecRANSAC::OrientedPair> PairFeas;
        getAllPairFeas(down_seg, down_seg_normals, PairFeas, pairWidth);
        
        std::cerr << "PairFea: " << PairFeas.size() << std::endl;
        
        list<PointSetShape*> detectedObjects;
        objrec.doRecognition(scene_ptr, PairFeas, detectedObjects);
        if( detectedObjects.empty() == true )
        {
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
            inliers->indices = cur_seg.indices;
            // Create the filtering object
            pcl::ExtractIndices<PointT> extract;
             // Extract the inliers
            extract.setInputCloud (cur_scene);
            extract.setIndices (inliers);
            extract.setNegative (true);
            extract.filter (*cur_scene);
        }
        else
        {
            std::vector<poseMessage> cur_poses;
            for ( list<PointSetShape*>::iterator it = detectedObjects.begin() ; it != detectedObjects.end() ; ++it )
            {
                PointSetShape* shape = (*it);
                if ( shape->getUserData() )
                    printf("\t%s, confidence: %lf\n", shape->getUserData()->getLabel(), shape->getConfidence());

                double **mat4x4 = mat_alloc(4, 4);
                shape->getHomogeneousRigidTransform(mat4x4);

                poseMessage new_pose;
                new_pose.tran << mat4x4[0][0], mat4x4[0][1], mat4x4[0][2], mat4x4[0][3],
                         mat4x4[1][0], mat4x4[1][1], mat4x4[1][2], mat4x4[1][3],
                         mat4x4[2][0], mat4x4[2][1], mat4x4[2][2], mat4x4[2][3],
                         mat4x4[3][0], mat4x4[3][1], mat4x4[3][2], mat4x4[3][3];
                new_pose.model_name = std::string (shape->getUserData()->getLabel());

                poses.push_back(new_pose);
                cur_poses.push_back(new_pose);
            }   

            cur_scene = FilterCloud(cur_scene, cur_poses);
        }
        iter++;
    }
    std::cerr<< "Recognizing Done!!!" << std::endl;
    //viewer->removeAllPointClouds();
}
*/
/**/
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
                
                //std::cerr << tmp.p1[0] << " " << tmp.p1[1] << " " << tmp.p1[2] << " "
                //        << tmp.p2[0] << " " << tmp.p2[1] << " " << tmp.p2[2] << " "
                //        << tmp.n1[0] << " " << tmp.n1[1] << " " << tmp.n1[2] << " "
                //        << tmp.n2[0] << " " << tmp.n2[1] << " " << tmp.n2[2] << std::endl;
                //std::cin.get();
                //std::cerr << count << " ";
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
