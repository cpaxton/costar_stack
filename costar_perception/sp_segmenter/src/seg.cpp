#include "sp_segmenter/seg.h"

void setObjID(std::map<std::string, int> &model_name_map)
{
    model_name_map["drill"] = 1;
    model_name_map["driller_small"] = 2;
    model_name_map["drill_flat"] = 3;
    model_name_map["drill_point"] = 4;
    model_name_map["mallet_ball_pein"] = 5;
    model_name_map["mallet_black_white"] = 6;
    model_name_map["mallet_drilling"] = 7;
    model_name_map["mallet_fiber"] = 8;
    model_name_map["old_hammer"] = 9;
    model_name_map["sander"] = 10;
}

void splitCloud(pcl::PointCloud<PointLT>::Ptr cloud, std::vector< pcl::PointCloud<myPointXYZ>::Ptr > &cloud_set)
{
    for(pcl::PointCloud<PointLT>::iterator it = cloud->begin() ; it < cloud->end() ; it++ )
    {
        myPointXYZ tmp;
        tmp.x = it->x;
        tmp.y = it->y;
        tmp.z = it->z;
        
        //std::cerr << it->rgba << " ";
        if( it->label >= cloud_set.size() )
        {
            while (it->label >= cloud_set.size())
            {
                pcl::PointCloud<myPointXYZ>::Ptr new_cloud(new pcl::PointCloud<myPointXYZ>());
                cloud_set.push_back(new_cloud);
            }
        }
        cloud_set[it->label]->points.push_back(tmp);
    }
}

void splitCloud(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<myPointXYZ>::Ptr link_cloud, pcl::PointCloud<myPointXYZ>::Ptr node_cloud)
{
    for(pcl::PointCloud<PointT>::iterator it = cloud->begin() ; it < cloud->end() ; it++ )
    {
        myPointXYZ tmp;
        tmp.x = it->x;
        tmp.y = it->y;
        tmp.z = it->z;
        
        //float score = *reinterpret_cast<float*>(&(it->rgba));
        
        if( it->rgba > 255 )  //link
            link_cloud->push_back(tmp);
        else if( it->rgba > 0 )   //node
            node_cloud->push_back(tmp);
    }
}

pcl::PointCloud<PointT>::Ptr cropCloud(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::ModelCoefficients::Ptr planeCoef, float elev)
{
    pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>());
    pcl::copyPointCloud(*cloud, *cloud_f);
    
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud_f);
    proj.setModelCoefficients (planeCoef);

    pcl::PointCloud<PointT>::Ptr cloud_projected(new pcl::PointCloud<PointT>());
    proj.filter (*cloud_projected);
    for(size_t i = 0 ; i < cloud_f->size() ; i++ )
    {
        if( pcl_isfinite(cloud_f->at(i).z) == true )
        {
            float diffx = cloud_f->at(i).x-cloud_projected->at(i).x;
            float diffy = cloud_f->at(i).y-cloud_projected->at(i).y;
            float diffz = cloud_f->at(i).z-cloud_projected->at(i).z;

            float dist = sqrt( diffx*diffx + diffy*diffy + diffz*diffz);
            if ( dist <= elev )
            {
                cloud_f->at(i).x = NAN;
                cloud_f->at(i).y = NAN;
                cloud_f->at(i).z = NAN;
            }
        }
    }
    cloud_f->is_dense = false;
    return cloud_f;
}


ModelT LoadMesh(std::string filename, std::string label)
{
    std::cerr << "Attempting to load \"" << filename << "\"...\n";
    
    // Construct Polygon Mesh
    ModelT cur_model;
    cur_model.model_mesh = pcl::PolygonMesh::Ptr (new pcl::PolygonMesh()); 
    if( exists_test(filename+".obj") == true )
        pcl::io::loadPolygonFile(filename+".obj", *cur_model.model_mesh); 
    else if( exists_test(filename+".stl") == true )
        pcl::io::loadPolygonFile(filename+".stl", *cur_model.model_mesh);
    else
    {
        std::cerr << "No OBJ or STL file!" << std::endl;
        exit(0);
    }
    
    //pcl::io::loadPolygonFile(filename, *cur_model.model_mesh); 
    
    pcl::PointCloud<myPointXYZ>::Ptr cloud(new pcl::PointCloud<myPointXYZ>()); 
    pcl::fromPCLPointCloud2(cur_model.model_mesh->cloud, *cloud);

    cur_model.model_cloud = pcl::PointCloud<myPointXYZ>::Ptr (new pcl::PointCloud<myPointXYZ>()); 
    pcl::VoxelGrid<myPointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.004, 0.004, 0.004);
    sor.filter(*cur_model.model_cloud);

    pcl::PointCloud<PointT>::Ptr temp(new pcl::PointCloud<PointT>());
    pcl::copyPointCloud(*cur_model.model_cloud, *temp);
    
    cur_model.model_center = pcl::PointCloud<myPointXYZ>::Ptr (new pcl::PointCloud<myPointXYZ>()); 
    ComputeCentroid(temp, cur_model.model_center);
    cur_model.model_label = label;
    return cur_model;
}



/*
int readCSV(std::string filename, std::vector< std::vector<float> > &poses)
{
    std::ifstream fp;
    fp.open(filename.c_str());
    if(fp.is_open() == false)
        return 0;
    
    int num;
    fp >> num;
    poses.resize(num);
    for(int i = 0 ; i < num ; i++)
    {
        poses[i].resize(7);
        for( int j = 0 ; j < 7 ; j++ )
            fp >> poses[i][j];
    }
    fp.close();
    return 1;
}

int writeCSV(std::string filename, const std::vector< std::vector<float> > &poses)
{
    std::ofstream fp;
    fp.open(filename.c_str());
    if( fp.is_open() == false )
    {
        std::cerr << "Failed to open files" << std::endl;
        return 0;
    }
    
    int num = poses.size();
    fp << num << std::endl;
    for(int i = 0 ; i < num ; i++)
    {
        for( int j = 0 ; j < poses[i].size() ; j++ )
            fp << poses[i][j] << " ";
        fp << std::endl;
    }
    fp.close();
    return 1;
}
*/
pcl::PointCloud<myPointXYZ>::Ptr FilterCloud(const pcl::PointCloud<myPointXYZ>::Ptr scene, const pcl::PointCloud<myPointXYZ>::Ptr tran_model, float T)
{
    pcl::search::KdTree<myPointXYZ> tree;
    tree.setInputCloud (tran_model);
    
    float sqrT = T*T;
    pcl::PointCloud<myPointXYZ>::Ptr filtered_scene(new pcl::PointCloud<myPointXYZ>());
    int num = scene->size();
    
    #pragma omp parallel for
    for(int i = 0 ; i < num ; i++ )
    {
        std::vector<int> indices (1);
        std::vector<float> sqr_dist (1);
        int nres = tree.nearestKSearch(scene->at(i), 1, indices, sqr_dist);
        if ( sqr_dist[0] > sqrT )
        {
            #pragma omp critical
            {
                filtered_scene->push_back(scene->at(i));
            }
        }
    }
    return filtered_scene;
}

float sqrDistPt(const myPointXYZ &pt1, const myPointXYZ &pt2)
{
    float diffx = pt1.x - pt2.x;
    float diffy = pt1.y - pt2.y;
    float diffz = pt1.z - pt2.z;
    return diffx*diffx + diffy*diffy + diffz*diffz;
}

float sqrDistPtT(const PointT &pt1, const PointT &pt2)
{
    float diffx = pt1.x - pt2.x;
    float diffy = pt1.y - pt2.y;
    float diffz = pt1.z - pt2.z;
    return diffx*diffx + diffy*diffy + diffz*diffz;
}

//=========================================================================================================================
/*
vtkSmartPointer<vtkCellArray> NewVertexCells(vtkIdType numberOfVerts)
{
  vtkNew<vtkIdTypeArray> cells;
  cells->SetNumberOfValues(numberOfVerts*2);
  vtkIdType* ids = cells->GetPointer(0);
  for (vtkIdType i = 0; i < numberOfVerts; ++i)
    {
    ids[i*2] = 1;
    ids[i*2+1] = i;
    }

  vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
  cellArray->SetCells(numberOfVerts, cells.GetPointer());
  return cellArray;
}
*/
//*
vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    vtkIdType nr_points = cloud->points.size();

    //vtkNew<vtkPoints> points;
    //points->SetDataTypeToFloat();
    //points->SetNumberOfPoints(nr_points);
    
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    
    if (cloud->is_dense)
    {
      for (vtkIdType i = 0; i < nr_points; ++i) {
        //float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}; 
        //points->SetPoint(i, point);
          points->InsertNextPoint(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
      }
    }
    else
    {
      vtkIdType j = 0;    // true point index
      for (vtkIdType i = 0; i < nr_points; ++i)
      {
        // Check if the point is invalid
        if (!pcl_isfinite (cloud->points[i].x) || 
            !pcl_isfinite (cloud->points[i].y) || 
            !pcl_isfinite (cloud->points[i].z))
          continue;

        //float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}; 
        //points->SetPoint(j, point);
        points->InsertNextPoint(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        j++;
      }
      //nr_points = j;
      //points->SetNumberOfPoints(nr_points);
    }

    //vtkPolyData *pd = vtkPolyData::New();
    //pd->SetPoints(points);
    
    //vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    //polyData.TakeReference(pd);
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    //polyData->SetPoints(points.GetPointer());
    //polyData->SetVerts(NewVertexCells(nr_points));
    return polyData;
}
//*/
//vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud(const pcl::PointCloud<PointT>::Ptr cloud)
/*
vtkPoints* PolyDataFromPointCloud(const pcl::PointCloud<PointT>::Ptr cloud)
{
    vtkIdType nr_points = cloud->points.size();

    vtkNew<vtkPoints> points;
    points->SetDataTypeToFloat();
    points->SetNumberOfPoints(nr_points);
    
    if (cloud->is_dense)
    {
      for (vtkIdType i = 0; i < nr_points; ++i) {
        float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}; 
        points->SetPoint(i, point);
      }
    }
    else
    {
      vtkIdType j = 0;    // true point index
      for (vtkIdType i = 0; i < nr_points; ++i)
      {
        // Check if the point is invalid
        if (!pcl_isfinite (cloud->points[i].x) || 
            !pcl_isfinite (cloud->points[i].y) || 
            !pcl_isfinite (cloud->points[i].z))
          continue;

        float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}; 
        points->SetPoint(j, point);
        j++;
      }
      nr_points = j;
      points->SetNumberOfPoints(nr_points);
    }

    //vtkPolyData *pd = vtkPolyData::New();
    //pd->SetPoints(points.GetPointer());
    //vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    //polyData.TakeReference(pd);
    //polyData->SetPoints(points.GetPointer());
    //polyData->SetVerts(NewVertexCells(nr_points));
    //return polyData;
    return points.GetPointer();
}
*/
/*
void PolyDataFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    vtkIdType nr_points = cloud->points.size();

    vtkNew<vtkPoints> points;
    points->SetDataTypeToFloat();
    points->SetNumberOfPoints(nr_points);
    
    if (cloud->is_dense)
    {
      for (vtkIdType i = 0; i < nr_points; ++i) {
        float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}; 
        points->SetPoint(i, point);
      }
    }
    else
    {
      vtkIdType j = 0;    // true point index
      for (vtkIdType i = 0; i < nr_points; ++i)
      {
        // Check if the point is invalid
        if (!pcl_isfinite (cloud->points[i].x) || 
            !pcl_isfinite (cloud->points[i].y) || 
            !pcl_isfinite (cloud->points[i].z))
          continue;

        float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}; 
        points->SetPoint(j, point);
        j++;
      }
      nr_points = j;
      points->SetNumberOfPoints(nr_points);
    }

    //vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    //polyData->SetPoints(points.GetPointer());
    //polyData->SetVerts(NewVertexCells(nr_points));
    //return polyData;
    
    //return points.GetPointer();
}
*/


