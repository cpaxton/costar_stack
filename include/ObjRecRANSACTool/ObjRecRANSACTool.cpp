#include "ObjRecRANSACTool.h"

ObjRecRANSACTool::ObjRecRANSACTool(double pairWidth, double voxelSize) : 
	ObjRecRANSAC(pairWidth, voxelSize, 0.5), pairWidth_(pairWidth), voxelSize_(voxelSize)
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

	// vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	// polyData->SetPoints(points);
	// vtkPoints* scene = polyData->GetPoints();
	this->setSceneDataForHypothesisCheck(points.GetPointer());
}

double ObjRecRANSACTool::getConfidence(const std::string &model_name, const btTransform &transform)
{
	AcceptedHypothesis hypothesis_tmp;
	
	btScalar gl_matrix[15];
	transform.getOpenGLMatrix(gl_matrix);

	double rigid_transform[12];
	for (int i = 0; i < 12; i++) rigid_transform[i] = gl_matrix[i];

	hypothesis_tmp.rigid_transform = rigid_transform;
	std::string tmp_name = model_name;
	return this->checkHypothesesConfidence(hypothesis_tmp,tmp_name);
}

void ObjRecRANSACTool::AddModel(const std::string &name, const std::string &label)
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

