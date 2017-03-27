#include "ObjRecRANSACTool.h"

ObjRecRANSACTool::ObjRecRANSACTool(double pairWidth, double voxelSize) : 
	ObjRecRANSAC(pairWidth, voxelSize, 0.5), pairWidth_(pairWidth), voxelSize_(voxelSize),
	have_scene_points_(false)
{
	successProbability_ = 0.99;
	visibility_ = 0.1;
	relativeObjSize_ = 0.1;
	this->setVisibility(visibility_);
	this->setRelativeObjectSize(relativeObjSize_);
}

void ObjRecRANSACTool::setPointCloudData(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	vtkIdType nr_points = cloud->points.size();
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

	if (cloud->is_dense)
	{
		for (vtkIdType i = 0; i < nr_points; ++i) {
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
			points->InsertNextPoint(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
			j++;
		}
	}

	this->setSceneDataForHypothesisCheck(points.GetPointer());
	have_scene_points_ = true;
}

double ObjRecRANSACTool::getConfidence(const std::string &model_name, const btTransform &transform)
{
	if (!have_scene_points_)
	{
		std::cerr << "ERROR: Does not have any scene points yet. Returning 0 confidence.\n";
		return 0;
	}

	AcceptedHypothesis hypothesis_tmp;
	
	btScalar gl_matrix[15];
	transform.getOpenGLMatrix(gl_matrix);

	double rigid_transform[12];
	// CONVERT GL MATRIX TO OBJRECRANSAC TRANSFORM MATRIX
	// [r  r  r  0]    [ 0  1  2  3]    [ 0  1  2  - ] 
	// [r  r  r  0] -> [ 4  5  6  7] -> [ 3  4  5  - ]
	// [r  r  r  0]    [ 8  9 10 11]    [ 6  7  8  - ] 
	// [tx ty tz 1]    [11 12 13 14]    [ 9  10 11 - ] 

	for (int i = 0, gl_index = 0, objransac_matrix_index = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j, ++gl_index)
		{
			if (j < 3)
			{
				rigid_transform[objransac_matrix_index] = gl_matrix[gl_index];
				// revert the scaling effect for checking confidence.
				rigid_transform[objransac_matrix_index] /= (i == 3) ? SCALING : 1;
				// std::cerr << rigid_transform[objransac_matrix_index] << "\t";

				++objransac_matrix_index;
			}
			if (gl_index == 15) break;
		}
		// std::cerr << std::endl;
	}

	hypothesis_tmp.rigid_transform = rigid_transform;
	std::string tmp_name = model_name;
	double confidence = this->checkHypothesesConfidence(hypothesis_tmp,tmp_name);
	return confidence;
}

void ObjRecRANSACTool::addModelFromPath(const std::string &name, const std::string &label)
{
	UserData* userData;
	userData = new UserData();
	userData->setLabel(label.c_str()); // Just set an 'Amicelli' label

	vtkPolyDataReader* reader = vtkPolyDataReader::New();
	reader->SetFileName((name+".vtk").c_str());
	reader->Update();
	// Add the model to the model library

	this->addModel(reader->GetOutput(), userData);
}

