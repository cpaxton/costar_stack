#include "sp_segmenter/utility/utility.h"

double get_wall_time(){
    struct timeval time;
    if (gettimeofday(&time,NULL)){
        //  Handle error
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

/****************************************************************************/
// Save matrix to binary file
int saveMat( const std::string& filename, const cv::Mat& M)
{
    if (M.empty()){
       return 0;
    }
    std::ofstream out(filename.c_str(), ios::out|ios::binary);
    if (!out)
       return 0;

    size_t cols = M.cols;
    size_t rows = M.rows;
    size_t chan = M.channels();
    size_t eSiz = (M.dataend-M.datastart)/(cols*rows*chan);

	//std::cerr<<cols<<" "<<rows<<" "<<chan<<" "<<eSiz<<" "<<M.isContinuous()<<std::endl;

    // Write header
    out.write((char*)&cols,sizeof(cols));
    out.write((char*)&rows,sizeof(rows));
    out.write((char*)&chan,sizeof(chan));
    out.write((char*)&eSiz,sizeof(eSiz));

    // Write data.
    if (M.isContinuous()){
		//std::cerr<<cols<<" "<<rows<<" "<<chan<<" "<<eSiz<<" "<<M.isContinuous()<<std::endl;
		//std::cerr<<"Hello!"<<" "<<cols*rows*chan*eSiz<<std::endl;
       out.write((char *)M.data,cols*rows*chan*eSiz);
    }
    else{
       return 0;
    }
    out.close();
    return 1;
}

/****************************************************************************/
// Read matrix from binary file
int readMat( const std::string& filename, cv::Mat& M)
{
    ifstream in(filename.c_str(), ios::in|ios::binary);
    if (!in){
       M = cv::Mat::zeros(0, 0, CV_32FC1);
       return 0;
    }
    size_t cols;
    size_t rows;
    size_t chan;
    size_t eSiz;

    // Read header
    in.read((char*)&cols,sizeof(cols));
    in.read((char*)&rows,sizeof(rows));
    in.read((char*)&chan,sizeof(chan));
    in.read((char*)&eSiz,sizeof(eSiz));

    // Determine type of the matrix 
    int type = 0;
    switch (eSiz){
    case sizeof(char):
         type = CV_8UC(chan);
         break;
    case sizeof(float):
         type = CV_32FC(chan);
         break;
	case sizeof(ushort):
         type = CV_16UC(chan);
         break;
    case sizeof(double):
         type = CV_64FC(chan);
         break;
    }

    // Alocate Matrix.
    M = cv::Mat(rows,cols,type,cv::Scalar(1));  

    // Read data.
    if (M.isContinuous()){
       in.read((char *)M.data,cols*rows*chan*eSiz);
    }
    else{
       return 0;
    }
    in.close();
    return 1;
}

void GenRandSeq(std::vector<size_t> &rand_idx, size_t len)
{
    srand(time(NULL));
    rand_idx.resize(len);
    size_t idx = 0;
    for( std::vector<size_t>::iterator it = rand_idx.begin() ; it < rand_idx.end() ; it++, idx++)
            (*it) = idx;
    std::random_shuffle(rand_idx.begin(), rand_idx.end());
}


bool exists_test (const std::string& name) {
   if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }   
}

bool exists_dir (const std::string& name) 
{
    boost::filesystem::path p( name );
    return boost::filesystem::is_directory(p);
}

bool play_dice(float prob)
{
	float tmp = (rand()+0.0) / RAND_MAX;
	return tmp <= prob; 
}

void CheckNormal(cv::Mat normal)
{
    float *ptr = (float *)normal.data;
    if( normal.cols == 3 && *ptr != *ptr)
    {
        *ptr = 0;
        *(ptr+1) = 0;
        *(ptr+2) = -1.0;
    }
}

cv::Mat getFullImage(const pcl::PointCloud<PointT>::Ptr full_cloud)
{
    if( full_cloud->isOrganized() == false )
        return cv::Mat::zeros(0, 0, CV_8UC3);
    
    int width = full_cloud->width;
    int height = full_cloud->height;
    
    cv::Mat img = cv::Mat::zeros(height, width, CV_8UC3);
    for( size_t i = 0 ; i < full_cloud->size() ; i++ )
    {
        uint32_t rgb = full_cloud->at(i).rgba;
        
        int r = i / width;
        int c = i % width;
        
        img.at<uchar>(r, c*3+2) = (rgb >> 16) & 0x0000ff;
        img.at<uchar>(r, c*3+1) = (rgb >> 8) & 0x0000ff;
        img.at<uchar>(r, c*3+0) = (rgb) & 0x0000ff;
    }
    
    return img;
}

/***************************************************My previous stuff************************************************/

void RGBToHSI(int rgb[], float hsi[])
{
    double r = rgb[0], g = rgb[1], b = rgb[2];

    double num = 0.5 * (r - g + r - b);
    double den = sqrt((r - g)*(r - g) + (r - b)* (g - b));
    double theta = acos(num/(den + EPS));

    if( b > g )
            hsi[0] = 2 * PI - theta;
    else
            hsi[0] = theta;
    hsi[0] = hsi[0] / (2*PI);

    if( r + g + b == 0 )
            hsi[1] = 1 - 3*std::min(std::min(r, g),b)/(r+g+b+EPS);
    else
            hsi[1] = 1 - 3*std::min(std::min(r, g),b)/(r+g+b);
    if( hsi[1] == 0 )
            hsi[0] = 0;
    hsi[2] = (r + g + b)/3/255.0;
}

void RGBToLab(int rgb[], float lab[])
{
    double R = ((double) rgb[0]) / (double)255.0;
    double G = ((double) rgb[1]) / (double)255.0;
    double B = ((double) rgb[2]) / (double)255.0;

    double X =  0.412453 * R + 0.357580 * G + 0.180423 * B;
    double Y =  0.212671 * R + 0.715160 * G + 0.072169 * B;
    double Z =  0.019334 * R + 0.119193 * G + 0.950227 * B;

    double xr = X / 0.950456, yr = Y / 1.000, zr = Z / 1.088854;
    if(yr > 0.008856) 
        lab[0] = 116.0 * pow(yr, 1.0 / 3.0) - 16.0;
    else 
        lab[0] = 903.3 * yr;

    double fxr, fyr, fzr;

    if(xr > 0.008856) 
        fxr = pow(xr, 1.0 / 3.0);
    else 
        fxr = 7.787 * xr + 16.0 / 116.0;

    if(yr > 0.008856) 
        fyr = pow(yr, 1.0 / 3.0);
    else 
        fyr = 7.787 * yr + 16.0 / 116.0;

    if(zr > 0.008856) 
        fzr = pow(zr, 1.0 / 3.0);
    else 
        fzr = 7.787 * zr + 16.0 / 116.0;

    lab[1] = 500.0 * (fxr - fyr);
    lab[2] = 200.0 * (fyr - fzr);
    
    //normalize lab
    lab[0] = 1.0 * lab[0] / 100,
    lab[1] = 1.0 * (lab[1] + 86.185) / 184.439,
    lab[2] = 1.0 * (lab[2] + 107.863) / 202.345;
}


void computeNormals(const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<NormalT>::Ptr &cloud_normals, float normal_ss)
{
    cloud_normals = pcl::PointCloud<NormalT>::Ptr (new pcl::PointCloud<NormalT>());

    pcl::NormalEstimationOMP<PointT, NormalT> normal_estimation;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    normal_estimation.setSearchMethod (tree);
    normal_estimation.setRadiusSearch(normal_ss);
    normal_estimation.setInputCloud (cloud);
    normal_estimation.compute (*cloud_normals);
}

void computeKeyNormals(pcl::PointCloud<myPointXYZ>::Ptr keypoints, pcl::PointCloud<NormalT>::Ptr &keypoints_normals, pcl::PointCloud<myPointXYZ>::Ptr surface, float normal_ss)
{
    keypoints_normals = pcl::PointCloud<NormalT>::Ptr (new pcl::PointCloud<NormalT>());

    pcl::NormalEstimationOMP<myPointXYZ, NormalT> normal_estimation;
    pcl::search::KdTree<myPointXYZ>::Ptr tree(new pcl::search::KdTree<myPointXYZ>);
    normal_estimation.setSearchMethod (tree);
    //normal_estimation.setNumberOfThreads(16);
    normal_estimation.setRadiusSearch(normal_ss);
    normal_estimation.setInputCloud (keypoints);
    normal_estimation.setSearchSurface (surface);
    normal_estimation.compute (*keypoints_normals);
}

void ComputeKeyRF(std::vector<keyT> &keypoints, const pcl::PointCloud<myPointXYZ>::Ptr surface, const pcl::PointCloud<NormalT>::Ptr surface_normals, float rf_rad)
{
    pcl::PointCloud<myPointXYZ>::Ptr temp_xyz(new pcl::PointCloud<myPointXYZ>());
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr keys_rf(new pcl::PointCloud<pcl::ReferenceFrame>());

    temp_xyz->resize(keypoints.size());
    int count = 0;
    for( std::vector<keyT>::iterator it = keypoints.begin() ; it < keypoints.end() ; it++, count++ )
        temp_xyz->at(count) = (*it).xyz;

    pcl::BOARDLocalReferenceFrameEstimation<myPointXYZ, NormalT, pcl::ReferenceFrame> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch(rf_rad);

    rf_est.setInputCloud (temp_xyz);
    rf_est.setInputNormals (surface_normals);
    rf_est.setSearchSurface (surface);
    rf_est.compute (*keys_rf);

    count = 0;
    for( std::vector<keyT>::iterator it = keypoints.begin() ; it < keypoints.end() ; it++, count++ )
        (*it).ref_frame = keys_rf->at(count);
}

double computeCloudResolution (const pcl::PointCloud<PointT>::ConstPtr &cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices (2);
	std::vector<float> sqr_distances (2);
	pcl::search::KdTree<PointT> tree;
	tree.setInputCloud (cloud);

	for (size_t i = 0; i < cloud->size (); ++i)
	{
		if (! pcl_isfinite ((*cloud)[i].x))
			continue;
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt (sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
		res /= n_points;
	return res;
}

void ComputeCentroid(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<myPointXYZ>::Ptr center_cloud)
{
    pcl::PointXYZ centroid;
    double cx=0, cy=0, cz=0;
    std::size_t num = cloud->points.size();
    for(std::size_t i=0; i < num ; i++ )
    {
        cx += cloud->points[i].x;
        cy += cloud->points[i].y;
        cz += cloud->points[i].z;
    }
    centroid.x = cx / num;
    centroid.y = cy / num;
    centroid.z = cz / num;

    center_cloud->push_back(centroid);
}

cv::Mat fpfh_cloud(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<PointT>::Ptr keys, const pcl::PointCloud<NormalT>::Ptr cloud_normals, float radius, bool normalzied)
{
    // Create the FPFH estimation class, and pass the input dataset+normals to it
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    pcl::FPFHEstimationOMP<PointT, NormalT, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(keys);
    //fpfh.setNumberOfThreads(16);
    fpfh.setSearchSurface(cloud);
    fpfh.setInputNormals (cloud_normals);
    fpfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch (radius);

    // Compute the features
    fpfh.compute (*fpfhs);

    //std::cerr<<fpfhs->size()<<" "<<cloud_filtered->size()<<std::endl;
    //system("pause");
    cv::Mat fea = cv::Mat::zeros(fpfhs->size(), 33, CV_32FC1);
    for( std::size_t i = 0 ; i < fpfhs->size() ; i++ )
    {
        float *ptr = (float *)fea.row(i).data;
        memcpy(ptr, fpfhs->at(i).histogram, sizeof(float)*33);
    }
    if(normalzied)
    {
        for( int i = 0 ; i < fea.rows ; i++ )
        {
            std::vector<cv::Mat> comps(3);
            comps[0] = fea.row(i).colRange(0, 11);
            comps[1] = fea.row(i).colRange(11, 22);
            comps[2] = fea.row(i).colRange(22, 33);
            cv::normalize(comps[0], comps[0]);
            cv::normalize(comps[1], comps[1]);
            cv::normalize(comps[2], comps[2]);
            cv::hconcat(comps, fea.row(i));
            cv::normalize(fea.row(i), fea.row(i));
        }
    }
    //std::cerr<<fea<<std::endl;
    return fea;	
}
/*
cv::Mat shot_cloud(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normals, float radius)
{
    pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());
    
    // SHOT estimation object.
    pcl::SHOTEstimationOMP<PointT, pcl::Normal, pcl::SHOT352> shot;
    //shot.setNumberOfThreads(16);
    shot.setInputCloud(cloud);
    shot.setInputNormals(cloud_normals);
    // The radius that defines which of the keypoint's neighbors are described.
    // If too large, there may be clutter, and if too small, not enough points may be found.
    shot.setRadiusSearch(radius);

    shot.compute(*descriptors);
    
    cv::Mat fea = cv::Mat::zeros(descriptors->size(), 352, CV_32FC1);
    for( int i = 0 ; i < descriptors->size() ; i++ )
    {
        float *ptr = (float *)fea.row(i).data;
        float temp = descriptors->at(i).descriptor[0];  // check whether descriptor is valid
        if( temp == temp )
            memcpy(ptr, descriptors->at(i).descriptor, sizeof(float)*352);
    }
    return fea;
}
*/

pcl::PointCloud<pcl::ReferenceFrame>::Ptr computeRF(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normals, 
        pcl::PointCloud<PointT>::Ptr &down_cloud, float radius, float ss)
{
    down_cloud = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>());
    if( ss <= 0 )
        down_cloud = cloud;
    else
    {
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(ss, ss, ss);
        sor.filter(*down_cloud);
    }
    
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr lrf(new pcl::PointCloud<pcl::ReferenceFrame>());
    
    pcl::BOARDLocalReferenceFrameEstimation<PointT, NormalT, pcl::ReferenceFrame> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch(radius);

    rf_est.setInputCloud (down_cloud);
    rf_est.setInputNormals (cloud_normals);
    rf_est.setSearchSurface (cloud);
    rf_est.compute (*lrf);
    
    return lrf;
}

cv::Mat shot_cloud_ss(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normals, const pcl::PointCloud<pcl::ReferenceFrame>::Ptr lrf, pcl::PointCloud<PointT>::Ptr &down_cloud, float radius, float ss)
{
    if( down_cloud->empty() == true )
    {
        down_cloud = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>());
        if( ss <= 0 )
            down_cloud = cloud;
        else
        {
            pcl::VoxelGrid<PointT> sor;
            sor.setInputCloud(cloud);
            sor.setLeafSize(ss, ss, ss);
            sor.filter(*down_cloud);
        }
    }
    
    if( lrf->empty() == false && lrf->size() != down_cloud->size() )
    {
        std::cerr << "lrf->size() != cloud->size()" << std::endl;
        exit(0);
    }
    
    pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());

    // SHOT estimation object.
    pcl::SHOTEstimationOMP<PointT, pcl::Normal, pcl::SHOT352> shot;
    //shot.setNumberOfThreads(40);
    shot.setInputCloud(down_cloud);
    if( lrf->empty() == false )
        shot.setInputReferenceFrames(lrf);
    shot.setSearchSurface(cloud);
    shot.setInputNormals(cloud_normals);
    // The radius that defines which of the keypoint's neighbors are described.
    // If too large, there may be clutter, and if too small, not enough points may be found.
    shot.setRadiusSearch(radius);

    shot.compute(*descriptors);
    cv::Mat fea = cv::Mat::zeros(descriptors->size(), 352, CV_32FC1);
    for( std::size_t i = 0 ; i < descriptors->size() ; i++ )
    {
        float *ptr = (float *)fea.row(i).data;
        float temp = descriptors->at(i).descriptor[0];  // check whether descriptor is valid
        if( temp == temp )
            memcpy(ptr, descriptors->at(i).descriptor, sizeof(float)*352);
    }
    return fea;
}

cv::Mat cshot_cloud_ss(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normals, const pcl::PointCloud<pcl::ReferenceFrame>::Ptr lrf, pcl::PointCloud<PointT>::Ptr &down_cloud, float radius, float ss)
{
    if( down_cloud->empty() == true )
    {
        down_cloud = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>());
        if( ss <= 0 )
            down_cloud = cloud;
        else
        {
            pcl::VoxelGrid<PointT> sor;
            sor.setInputCloud(cloud);
            sor.setLeafSize(ss, ss, ss);
            sor.filter(*down_cloud);
        }
    }
    
    pcl::PointCloud<pcl::SHOT1344>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT1344>());
    if( lrf->empty() == false && lrf->size() != down_cloud->size() )
    {
        std::cerr << "lrf->size() != cloud->size()" << std::endl;
        exit(0);
    }
    
    // SHOT estimation object.
    pcl::SHOTColorEstimationOMP<PointT, pcl::Normal, pcl::SHOT1344> cshot;
    
    //cshot.setNumberOfThreads(8);
    if( lrf->empty() == false )
        cshot.setInputReferenceFrames(lrf);
    cshot.setInputCloud(down_cloud);
    cshot.setSearchSurface(cloud);
    cshot.setInputNormals(cloud_normals);
    cshot.setRadiusSearch(radius);

    cshot.compute(*descriptors);
    
    int num = descriptors->size();
    cv::Mat fea = cv::Mat::zeros(num, 1344, CV_32FC1);
    
    #pragma omp parallel for schedule(dynamic, 50)
    for( int i = 0 ; i < num ; i++ )
    {
        float *ptr = (float *)fea.row(i).data;
        float temp = descriptors->at(i).descriptor[0];  // check whether descriptor is valid
        if( temp == temp )
            memcpy(ptr, descriptors->at(i).descriptor, sizeof(float)*1344);
    }
    return fea;
}

cv::Mat cshot_cloud_uni(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normals, std::vector<int> &active_idx, float radius, int snum)
{
    pcl::PointCloud<PointT>::Ptr down_cloud(new pcl::PointCloud<PointT>());
      
    std::vector<size_t> rand_idx;
    if( cloud->size() < snum )
    {
        down_cloud = cloud;
        rand_idx.resize(cloud->size());
        for(size_t i = 0 ; i < cloud->size() ; i++ )
            rand_idx[i] = i;
    }
    else
    {
        GenRandSeq(rand_idx, cloud->size());
        rand_idx.erase(rand_idx.begin()+snum, rand_idx.end());
        
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        inliers->indices.resize(rand_idx.size());
        for( size_t i = 0 ; i < rand_idx.size() ; i++ )
            inliers->indices[i] = rand_idx[i];
        
        // Create the filtering object
        pcl::ExtractIndices<PointT> extract;
         // Extract the inliers
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*down_cloud);
    }
    
    pcl::PointCloud<pcl::SHOT1344>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT1344>());

    // SHOT estimation object.
    pcl::SHOTColorEstimationOMP<PointT, pcl::Normal, pcl::SHOT1344> shot;
    shot.setInputCloud(down_cloud);
    shot.setSearchSurface(cloud);
    shot.setInputNormals(cloud_normals);
    // The radius that defines which of the keypoint's neighbors are described.
    // If too large, there may be clutter, and if too small, not enough points may be found.
    shot.setRadiusSearch(radius);

    shot.compute(*descriptors);
    
    std::vector<cv::Mat> fea_pool;
    std::size_t num = descriptors->size();
    for( std::size_t i = 0 ; i < num ; i++ )
    {
        cv::Mat cur_fea = cv::Mat::zeros(1, 1344, CV_32FC1);
        float temp = descriptors->at(i).descriptor[0];  // check whether descriptor is valid
        if( temp == temp )
        {
            memcpy(cur_fea.data, descriptors->at(i).descriptor, sizeof(float)*1344);
            fea_pool.push_back(cur_fea);
            active_idx.push_back(rand_idx[i]);
        }   
    }
    cv::Mat fea;
    cv::vconcat(fea_pool, fea);
    return fea;
}
/*
cv::Mat usc_cloud_ss(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normals, const pcl::PointCloud<pcl::ReferenceFrame>::Ptr lrf, pcl::PointCloud<PointT>::Ptr &down_cloud, float radius, float ss)
{
    if( down_cloud->empty() == true )
    {
        down_cloud = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>());
        if( ss <= 0 )
            down_cloud = cloud;
        else
        {
            pcl::VoxelGrid<PointT> sor;
            sor.setInputCloud(cloud);
            sor.setLeafSize(ss, ss, ss);
            sor.filter(*down_cloud);
        }
    }
    
    pcl::PointCloud<pcl::ShapeContext1980>::Ptr descriptors(new pcl::PointCloud<pcl::ShapeContext1980>());
    if( lrf->size() != down_cloud->size() )
    {
        std::cerr << "lrf->size() != cloud->size()" << std::endl;
        exit(0);
    }
    
    pcl::UniqueShapeContext<PointT, pcl::ShapeContext1980, pcl::ReferenceFrame> usc;
    usc.setInputCloud(down_cloud);
    // Search radius, to look for neighbors. It will also be the radius of the support sphere.
    usc.setRadiusSearch(radius);
    usc.setSearchSurface(cloud);
    // The minimal radius value for the search sphere, to avoid being too sensitive
    // in bins close to the center of the sphere.
    usc.setMinimalRadius(radius / 10.0);
    // Radius used to compute the local point density for the neighbors
    // (the density is the number of points within that radius).
    usc.setPointDensityRadius(radius / 5.0);
    // Set the radius to compute the Local Reference Frame.
    usc.setLocalRadius(radius);

    usc.compute(*descriptors);
    
    cv::Mat fea = cv::Mat::zeros(descriptors->size(), 1980, CV_32FC1);
    for( int i = 0 ; i < descriptors->size() ; i++ )
    {
        float *ptr = (float *)fea.row(i).data;
        float temp = descriptors->at(i).descriptor[0];  // check whether descriptor is valid
        if( temp == temp )
            memcpy(ptr, descriptors->at(i).descriptor, sizeof(float)*1980);
    }
    return fea;
}
*/
void RefineCloud(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr refined_cloud, float radius)
{
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointXYZRGBNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<PointT, pcl::PointXYZRGBNormal> mls;

    mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud (cloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (radius);  //*7 for preprecessing

    // Reconstruct
    mls.process (mls_points);
    refined_cloud->points.clear();
    pcl::copyPointCloud<pcl::PointXYZRGBNormal, PointT>(mls_points, *refined_cloud);
    //pcl::copyPointCloud<pcl::PointXYZRGBNormal, NormalT>(mls_points, *refined_normals);
}

void ExtractHue(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hue)
{
    std::size_t num = cloud->points.size();
    pcl::copyPointCloud<PointT>(*cloud, *cloud_hue);
    //#pragma omp parallel for firstprivate(cloud_hue)
    for(std::size_t j = 0 ; j < num ; j++ )
    {
            pcl::PointXYZHSV &temp = cloud_hue->points[j];
            int rgb[3] = { cloud->points[j].r, cloud->points[j].g, cloud->points[j].b };
            float hsi[3];
            RGBToHSI(rgb, hsi);
            cloud_hue->points[j].h = hsi[0];
            cloud_hue->points[j].s = hsi[1];
            cloud_hue->points[j].v = hsi[2];
    }
}

std::vector< std::vector<int> > connectedNodes(bool adj[][MAX_SEGMENT], int num)
{
    std::vector<std::vector<int> > cluster_id;

    std::vector<bool> flags(num, 0);
    for( int i = 0 ; i < num ; i++ )
    {
        if( flags[i] == false )
        {
            std::vector<int> new_cluster, list;
            new_cluster.push_back(i);
            list.push_back(i);
            flags[i] = true;
            while(true)
            {
                if( list.empty() == true )
                    break;
                int cur_id = list[0];
                list.erase(list.begin());
                for( int j = 0 ; j < num ; j++ ){
                    if( flags[j] == false && adj[cur_id][j] == true )
                    {
                        new_cluster.push_back(j);
                        list.push_back(j);
                        flags[j] = true;
                    }
                }
            }
            cluster_id.push_back(new_cluster);
        }
    }
    return cluster_id;
}

void showCorrs(const std::vector<keyT> &key1, const std::vector<keyT> &key2, pcl::CorrespondencesPtr corrs, 
		const pcl::PointCloud<PointT>::Ptr surface1, const pcl::PointCloud<PointT>::Ptr surface2, pcl::visualization::PCLVisualizer::Ptr &viewer)
{
	pcl::PointCloud<myPointXYZ>::Ptr key_pt1 (new pcl::PointCloud<myPointXYZ>());
	pcl::PointCloud<myPointXYZ>::Ptr key_pt2 (new pcl::PointCloud<myPointXYZ>());

	for( pcl::Correspondences::iterator it = corrs->begin() ; it < corrs->end(); it++ )
	{
		key_pt1->push_back(key1[(*it).index_query].xyz);
		key_pt2->push_back(key2[(*it).index_match].xyz);
		
	}

	pcl::PointCloud<PointT>::Ptr off_surface2(new pcl::PointCloud<PointT>());
	pcl::PointCloud<myPointXYZ>::Ptr off_key_pt2(new pcl::PointCloud<myPointXYZ>());
	pcl::transformPointCloud (*surface2, *off_surface2, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
	pcl::transformPointCloud (*key_pt2, *off_key_pt2, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

	viewer->addPointCloud (off_surface2, "off_surface2");
	viewer->addPointCloud (surface1, "surface1");
	viewer->addPointCloud (key_pt1, "key_pt1");
	viewer->addPointCloud (off_key_pt2, "off_key_pt2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "key_pt1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "key_pt1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "off_key_pt2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "off_key_pt2");
	//*
	for (std::size_t j = 0; j < key_pt1->size (); j++)
	{
		std::stringstream ss_line;
		ss_line << "corr_line" << j;

		myPointXYZ scene_point = key_pt1->at(j);
		myPointXYZ model_point = off_key_pt2->at(j);

		//std::cerr<<corrs->at(j).index_query<<std::endl;
		//std::cerr<<scene_point.x<<" "<<scene_point.y<<" "<<scene_point.z<<std::endl;

		viewer->addLine<myPointXYZ, myPointXYZ> (model_point, scene_point, 0, 255, 0, ss_line.str ());
	}
	//*
	for(std::size_t i = 0 ; i < key1.size(); i++ )
	{
            if( key1[i].shift_vec.empty() == false)
            {
                for( std::size_t j = 0 ; j < key1[i].shift_vec.size() ; j++ )
                {
                    std::stringstream ss_line;
                    ss_line << "proj_line" << i <<" " <<j;

                    pcl::PointXYZ temp1, temp2;
                    temp1.x = key1[i].xyz.x + key1[i].shift_vec[j].x;
                    temp1.y = key1[i].xyz.y + key1[i].shift_vec[j].y;
                    temp1.z = key1[i].xyz.z + key1[i].shift_vec[j].z;

                    temp2 = key1[i].xyz; 

                    //std::cerr<<key1[i].shift_vec[j].x<<" "<<key1[i].shift_vec[j].y<<" "<<key1[i].shift_vec[j].z<<std::endl;
                    viewer->addLine<myPointXYZ, myPointXYZ> (temp1, temp2, 1.0, 1.0, 0.0, ss_line.str ());
                    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, ss_line.str ());
                    //viewer->spin();
                }
            }
	}
	//*/
	viewer->spin();
	viewer->removePointCloud("off_surface2");
        viewer->removePointCloud("surface1");
        viewer->removePointCloud("key_pt1");
        viewer->removePointCloud("off_key_pt2");
	viewer->removeAllShapes();
}

void find_files(const boost::filesystem::path &root, const std::string& ext, std::vector<std::string>& ret)
{  
    if (!boost::filesystem::exists(root)) 
        return;
    if (boost::filesystem::is_directory(root))
    {
        boost::filesystem::directory_iterator it(root);
        boost::filesystem::directory_iterator endit;
        while(it != endit)
        {
            //if (boost::filesystem::is_regular_file(*it) and it->path().extension() == ext)
            std::string file_name(it->path().filename().c_str());
            size_t len = file_name.size();
            if (boost::filesystem::is_regular_file(*it) and file_name.substr(len-ext.size(), len) == ext)
            {
                //std::cerr<<it->path().string()<<std::endl;
                ret.push_back(file_name);
            }
            ++it;
        }
    }
}

void find_dirs(const boost::filesystem::path &root, std::vector<std::string>& ret)
{  
    if (!boost::filesystem::exists(root)) 
        return;
    if (boost::filesystem::is_directory(root))
    {
        boost::filesystem::directory_iterator it(root);
        boost::filesystem::directory_iterator endit;
        while(it != endit)
        {
            if( boost::filesystem::is_directory(*it) == true )
            {
                std::string file_name(it->path().filename().c_str());
                ret.push_back(file_name);
            }
            ++it;
        }
    }
}

pcl::PointCloud<PointT>::Ptr NormalizeXY(const pcl::PointCloud<PointT>::Ptr cloud)
{
    float avg_x = 0 , avg_y = 0;
    for( pcl::PointCloud<PointT>::const_iterator it = cloud->begin() ; it < cloud->end() ; it++ )
    {
        avg_x += (*it).x;
        avg_y += (*it).y;
    }
    avg_x = avg_x / cloud->size();
    avg_y = avg_y / cloud->size();
    Eigen::Matrix4f tran = Eigen::Matrix4f::Identity();
    tran(0, 3) = -avg_x;
    tran(1, 3) = -avg_y;
    
    pcl::PointCloud<PointT>::Ptr new_cloud(new pcl::PointCloud<PointT>());
    pcl::transformPointCloud(*cloud, *new_cloud, tran);
    return new_cloud;
}

void NormalizeCloud(pcl::PointCloud<PointT>::Ptr cloud)
{
    pcl::PointCloud<myPointXYZ>::Ptr centroid(new pcl::PointCloud<myPointXYZ>());
    ComputeCentroid(cloud, centroid);
    myPointXYZ center = centroid->at(0);

    float max=0, diffx, diffy, diffz, dist;
    for( pcl::PointCloud<PointT>::iterator it = cloud->begin(); it < cloud->end() ; it++ )
    {
        diffx = (*it).x - center.x;
        diffy = (*it).y - center.y;
        diffz = (*it).z - center.z;
        dist = diffx*diffx + diffy*diffy + diffz*diffz;
        if( dist > max ) 
            max = dist;
    }
    float ratio = sqrt(1/max);
    for( pcl::PointCloud<PointT>::iterator it = cloud->begin(); it < cloud->end() ; it++ )
    {
        (*it).x = ( (*it).x - center.x ) * ratio;
        (*it).y = ( (*it).y - center.y ) * ratio;
        (*it).z = ( (*it).z - center.z ) * ratio;
    }
}

pcl::PointCloud<PointLT>::Ptr SPCloud(const pcl::PointCloud<PointT>::Ptr cloud, std::vector<pcl::PointCloud<PointT>::Ptr> &segs, IDXSET &seg_to_cloud, std::multimap<uint32_t, uint32_t> &graph, 
                                    float voxel_resol, float seed_resol, float color_w, float spatial_w, float normal_w)
{
    pcl::SupervoxelClustering<PointT> super (voxel_resol, seed_resol);
    
    super.setInputCloud (cloud);
    super.setColorImportance (color_w);
    super.setSpatialImportance (spatial_w);
    super.setNormalImportance (normal_w);
    
    std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> sp_clusters;

    //pcl::console::print_highlight ("Extracting supervoxels!\n");
    super.extract(sp_clusters);
    
    pcl::PointCloud<PointLT>::Ptr labels = super.getLabeledCloud();
    super.getSupervoxelAdjacency(graph);
    
    size_t max_label = 0;
    for(size_t i = 0 ; i < labels->size() ; i++ )
    {
        if( labels->at(i).label > max_label )
            max_label = labels->at(i).label;
    }    
    size_t seg_num = max_label + 1;// = super.getMaxLabel() + 1;

    segs.clear();
    segs.resize(seg_num);
    seg_to_cloud.clear();
    seg_to_cloud.resize(seg_num);
    
//    pcl::PointCloud<PointT>::Ptr seg_center(new pcl::PointCloud<PointT>());
    for(size_t i = 0 ; i < seg_num ; i++ )
    {
//        if( sp_clusters[i] )
//            seg_center->push_back(sp_clusters[i]->centroid_);
//        else
//            seg_center->push_back(PointT ());
        segs[i] = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>());
    }
    
    for(size_t i = 0 ; i < labels->size() ; i++ )
    {
        int cur_label = labels->at(i).label;
        seg_to_cloud[cur_label].push_back(i);
        segs[cur_label]->push_back(cloud->at(i));
    }
    return labels;
}

pcl::PointCloud<PointT>::Ptr SPCloud(const pcl::PointCloud<PointT>::Ptr cloud, std::vector<pcl::PointIndices::Ptr> &segs_idx, std::multimap<uint32_t, uint32_t> &graph, 
                                    float voxel_resol, float seed_resol, float color_w, float spatial_w, float normal_w)
{
    pcl::SupervoxelClustering<PointT> super (voxel_resol, seed_resol);
    
    super.setInputCloud (cloud);
    super.setColorImportance (color_w);
    super.setSpatialImportance (spatial_w);
    super.setNormalImportance (normal_w);
    
    std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> sp_clusters;

    //pcl::console::print_highlight ("Extracting supervoxels!\n");
    super.extract(sp_clusters);
    
    pcl::PointCloud<PointLT>::Ptr labels = super.getLabeledCloud();
    super.getSupervoxelAdjacency(graph);

    size_t max_label = 0;
    for(size_t i = 0 ; i < labels->size() ; i++ )
    {
        if( labels->at(i).label > max_label )
            max_label = labels->at(i).label;
    }    

    size_t seg_num = max_label + 1;// = super.getMaxLabel() + 1;
    segs_idx.clear();
    segs_idx.resize(seg_num);
    
    pcl::PointCloud<PointT>::Ptr seg_center(new pcl::PointCloud<PointT>());
    for(size_t i = 0 ; i < seg_num ; i++ )
    {
        if( sp_clusters[i] )
            seg_center->push_back(sp_clusters[i]->centroid_);
        else
            seg_center->push_back(PointT ());
        segs_idx[i] = pcl::PointIndices::Ptr (new pcl::PointIndices());
    }
    
    for(size_t i = 0 ; i < labels->size() ; i++ )
        segs_idx[labels->at(i).label]->indices.push_back(i);
    
    return seg_center;
}


/*
#ifdef OPENCV_SIFT
void ComputeSIFT(const Cloud_Image &frame, std::vector<keyT> &key_vec, std::vector<keyDescrT> &key_descr, bool show_keys)
{
    cv::SiftFeatureDetector *sift_det = new cv::SiftFeatureDetector(
            0, // nFeatures
            4, // nOctaveLayers
            0.04, // contrastThreshold 
            10, //edgeThreshold
            1.6 //sigma
            );
    
    cv::SiftDescriptorExtractor * sift_ext = new cv::SiftDescriptorExtractor();
    // Compute keypoints and descriptor from the source image in advance
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    
    sift_det->detect(frame.gray, keypoints);
    sift_ext->compute(frame.gray, keypoints, descriptors);
    printf("%d sift keypoints are found.\n", (int)keypoints.size());
    
    if( show_keys )
    {
        cv::Mat out_image;
        cv::drawKeypoints(frame.gray, keypoints, out_image);
        cv::imshow("keypoints", out_image);
        cv::waitKey();
    }
    for(int i = 0 ; i < keypoints.size() ; i++ )
    {
        int r = keypoints[i].pt.y;
        int c = keypoints[i].pt.x;
        int idx = frame.map2D3D.at<int>(r, c);
        if ( idx < 0 )
            continue;
        
        keyDescrT temp_descr;
        temp_descr.feaDescr = cv::Mat::zeros(1, descriptors.cols, CV_32FC1);
        descriptors.row(i).copyTo(temp_descr.feaDescr);
        cv::normalize(temp_descr.feaDescr,temp_descr.feaDescr, 1.0);
        temp_descr.fea_type = 0;
        temp_descr.idx = key_vec.size();
        
        key_descr.push_back(temp_descr);
        
        keyT temp;
        temp.xyz = frame.surface->at(idx);
        if( frame.normals->empty() == false )
            temp.normal = frame.normals->at(idx);
        key_vec.push_back(temp);
    }
}
#else
void ComputeSIFT(const Cloud_Image &frame, std::vector<keyT> &key_vec, std::vector<keyDescrT> &key_descr, bool show_keys)
{
    //Extract Sift Keypoints and descriptors
    //This part is ugly but uses best sift implementation
    cv::imwrite("tmp.pgm",frame.gray);
#ifdef _WIN32
    system("siftWin32 <tmp.pgm >tmp.key");
#else
    system("./sift <tmp.pgm >tmp.key");
#endif

    FILE *fp = fopen("tmp.key","r");
    if (!fp)
    {
        std::cerr<<"Cannot open file tmp.key"<<std::endl;
        exit(0);
    }

    int key_rows, key_cols;
    fscanf(fp,"%d %d", &key_rows, &key_cols);
    if(key_cols != 128)
    {
        std::cerr<<"Invalid Keypoint Descriptors"<<std::endl;
        exit(0);
    }
    
    std::vector<cv::KeyPoint> show_key_vec;
    float y, x, octave, angle;
    int r, c, idx;
    for( int i=0; i < key_rows ; i++ ) 
    {
        if (fscanf(fp, "%f %f %f %f", &y, &x, &octave, &angle) != 4)
        {
            std::cerr<<"Invalid keypoint file format."<<std::endl;
            exit(0);
        }
        r = y;
        c = x;
        idx = frame.map2D3D.at<int>(r, c);

        keyDescrT temp_descr;
        temp_descr.feaDescr = cv::Mat::zeros(1, key_cols, CV_32FC1);
        float *ptr = (float *)temp_descr.feaDescr.data;
        for (int j = 0; j < key_cols; j++, ptr++) 
        {
            int sift_val;
            if ( fscanf(fp, "%d", &sift_val) != 1 || sift_val < 0 || sift_val > 255)
            {
                std::cerr<<"Invalid keypoint value "<<sift_val<<std::endl;
                exit(0);
            }
            *ptr = sift_val; 
        }

        if( idx < 0 )
            continue;
        
        // push feature descriptor
        temp_descr.fea_type = 0;	//sift
        temp_descr.idx = key_vec.size();
        cv::normalize(temp_descr.feaDescr,temp_descr.feaDescr, 1.0);
        key_descr.push_back(temp_descr);
        
        // push feature points
        if( show_keys )
        {
            cv::KeyPoint show_key;
            show_key.pt.x = x;
            show_key.pt.y = y;
            show_key.angle = angle;
            show_key.octave = octave;
            show_key_vec.push_back(show_key);
        }
        
        keyT temp;
        temp.xyz = frame.surface->at(idx);
        if( frame.normals->empty() == false )
            temp.normal = frame.normals->at(idx);
        key_vec.push_back(temp);
        
    }       
    fclose(fp);

    if( show_keys )
    {
        cv::Mat out_image;
        cv::drawKeypoints(frame.gray, show_key_vec, out_image);
        cv::imshow("keypoints", out_image);
        cv::waitKey();
    }
}
#endif
*/

pcl::PointIndices::Ptr ConnectedComponent3D(const pcl::PointCloud<PointT>::Ptr cloud, float radius, int min_segment)
{
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (radius); // 2cm
    ec.setMinClusterSize (min_segment);
    ec.setMaxClusterSize (1000000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for (std::vector<pcl::PointIndices>::iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        inliers->indices.insert(inliers->indices.end(), (*it).indices.begin(), (*it).indices.end());
        
    return inliers;
}

pcl::PointCloud<myPointXYZ>::Ptr GenNormalPt(const pcl::PointCloud<myPointXYZ>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normals, float len)
{
    pcl::PointCloud<myPointXYZ>::Ptr combined_cloud(new pcl::PointCloud<myPointXYZ>());
    if( cloud->size() != cloud_normals->size() )    //one to one correspondence
        return combined_cloud;
    for( std::size_t i = 0 ; i < cloud->size() ; i++ )
    {
        myPointXYZ pt = cloud->at(i);
        NormalT pt_normal = cloud_normals->at(i);
        
        if( pt_normal.normal_x == pt_normal.normal_x )
        {
            myPointXYZ temp;
            temp.x = pt.x + pt_normal.normal_x*len;
            temp.y = pt.y + pt_normal.normal_y*len;
            temp.z = pt.z + pt_normal.normal_z*len;
            combined_cloud->push_back(temp);
        }
        else
            combined_cloud->push_back(myPointXYZ (-1000,-1000,-1000));
    }
    
    return combined_cloud;
}

float distPt(const myPointXYZ &pt1, const myPointXYZ &pt2)
{
    float diffx = pt1.x - pt2.x;
    float diffy = pt1.y - pt2.y;
    float diffz = pt1.z - pt2.z;
    return sqrt(diffx*diffx + diffy*diffy + diffz*diffz);
}

void transformKeys(std::vector<keyT> &keys, const Eigen::Matrix4f &trans)
{
    Eigen::Affine3f t(trans);
    for( std::vector<keyT>::iterator it = keys.begin(); it < keys.end() ; it++ )
        (*it).xyz = pcl::transformPoint((*it).xyz, t);
}

pcl::PointCloud<myPointXYZ>::Ptr uniformSampleCloud(const pcl::PointCloud<myPointXYZ>::Ptr cloud, int num)
{
    if( cloud->size() <= num )
        return cloud;
    std::vector<size_t> rand_idx;
    GenRandSeq(rand_idx, cloud->size());
    
    pcl::PointIndices::Ptr pt_idx(new pcl::PointIndices());
    pt_idx->indices.insert(pt_idx->indices.end(), rand_idx.begin(), rand_idx.begin()+num);
    
    pcl::PointCloud<myPointXYZ>::Ptr down_cloud(new pcl::PointCloud<myPointXYZ>());
    pcl::ExtractIndices<myPointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (pt_idx);
    extract.setNegative (false);
    extract.filter (*down_cloud);
    
    return down_cloud;
}

std::vector<int> getOneClique(const Eigen::MatrixXi &adj_mat, const std::vector<int> &active_idx)
{
    std::size_t num = active_idx.size();
    bool **adj = new bool*[num];
    for(std::size_t i = 0 ; i < num ; i++ )
    {
        adj[i] = new bool[num];
        int r = active_idx[i];
        for( int j = 0 ; j < num ; j++ )
        {
            int c = active_idx[j];
            adj[i][j] = adj_mat(r, c) == 1 ? true : false; 
        }
        //memset(adj[i], 0, sizeof(bool)*num);
    } 
    Maxclique md(adj, num); 
    int *qmax;
    int qsize;
    md.mcqdyn(qmax, qsize);  // run max clique with improved coloring and dynamic sorting of vertices 
    
    std::vector<int> clique_idx(qsize);
    for (int i = 0; i < qsize; i++)
    {
        int cur_idx = active_idx[qmax[i]];
        clique_idx[i] = cur_idx;
        //std::cout << cur_idx << " ";
    }
    //std::cout << endl;
    //cout << "Size = " << qsize << endl;
    delete []qmax;
    for(int i = 0 ; i < num ; i++ )
        delete[] (adj[i]);
    
    delete []adj;
    return clique_idx;
}

std::vector< std::vector<int> > maximalClique(const Eigen::MatrixXi &adj_mat)
{
    // in-degree sorting
    int num = adj_mat.rows();
    /*
    std::vector< std::pair<int,int> > in_degree(num);
    for(int i = 0 ; i < num ; i++)
    {
        int count = 0;
        for( int j = 0 ; j < num ; j++)
            if( adj_mat(i, j) == 1 && i != j )
                count++;
        in_degree[i].first = i;
        in_degree[i].second = count;
    }
    std::sort(in_degree.begin(), in_degree.end(), comp1);
    */
    
    std::vector<bool> flag(num);
    flag.assign(num, false);
    std::vector< std::vector<int> > final_clusters;
    // going from smallest in-degree node to largest one
    //for( int i = in_degree.size()-1 ; i >= 0 ; i-- ){
    for( int i = 0 ; i < num ; i++ ){
        //int pivot_idx = in_degree[i].first;
        int pivot_idx = i;
        if( flag[pivot_idx] == false )
        {
            std::vector<int> active_idx;
            for(int j = 0 ; j < num ; j++ )
            {
                if( adj_mat(pivot_idx, j) == 1 && flag[j] == false )
                    active_idx.push_back(j);
            }
            //std::cerr << "pivot idx: " << pivot_idx << std::endl;
            std::vector<int> new_cluster = getOneClique(adj_mat, active_idx);
            //std::cerr<<"Cluster "<<final_clusters.size()<<": "<<new_cluster.size()<<std::endl;
            final_clusters.push_back(new_cluster);                   
            //for(int j = 0 ; j < new_cluster.size() ; j++ )
            //    flag[new_cluster[j]] = true;
            
        }
    }
    
    return final_clusters;
}

void saveUndirectedGraph(const Eigen::MatrixXi &adj, std::string filename)
{
    int num = adj.rows();
    std::ofstream fp(filename.c_str(), std::ios::out);
    
    std::vector< std::pair<int, int> > edges;
    for( int i = 0 ; i < num ; i++ )
        for( int j = i+1 ; j < num ; j++)
            if( adj(i, j) == 1 )
                edges.push_back( std::pair<int,int> (i,j) );
    
    if( fp.is_open() == true )
    {
        fp << num << " " << edges.size() << std::endl;
        for( std::vector< std::pair<int, int> >::iterator it = edges.begin() ; it < edges.end() ; it++ )
            fp << (*it).first << " " << (*it).second << std::endl;
        fp.close();
    }
}

Eigen::MatrixXi readUndirectedGraph(std::string filename)
{
    Eigen::MatrixXi adj;
    std::ifstream fp(filename.c_str(), std::ios::in);
    if( fp.is_open() == true )
    {
        int num, edge_num, r, c;
        fp >> num >> edge_num;
        adj = Eigen::MatrixXi::Identity(num, num);
        for( int i = 0 ; i < edge_num ; i++ )
        {
            fp >> r >> c;
            adj(r, c) = 1;
            adj(c, r) = 1;
        }
        fp.close();
    }
    return adj;
}

poseVec readRots(std::string filename)
{
    poseVec rot_set;
    std::ifstream fp(filename.c_str(), std::ios::in);
    
    if(fp.is_open())
    {
        int num, width;
        fp >> num >> width;
        for( int i = 0 ; i < num ; i++ )
        {
            Eigen::Matrix4f cur_rot = Eigen::Matrix4f::Identity();
            fp >> cur_rot(0, 0) >> cur_rot(0, 1) >> cur_rot(0, 2) >> cur_rot(1, 0) >> cur_rot(1, 1) >> cur_rot(1, 2)
                    >> cur_rot(2, 0) >> cur_rot(2, 1) >> cur_rot(2, 2);
            rot_set.push_back(cur_rot);
        }
        fp.close();
    }
    return rot_set;
}

bool checkTrans(const Eigen::Matrix4f &tran)
{
    return -(tran(0,2)+tran(1,2)+tran(2,2)) < 0.01;
}

// the default viewing direction is (0, 0, -1)
pcl::PointCloud<PointT>::Ptr createPartialView(const pcl::PointCloud<PointT>::Ptr cloud, float sampleR)
{
    float min_x = -1.0, min_y = -1.0, min_z = -100.0;
    int len = std::ceil(2.0/sampleR);
    std::vector< std::pair<int, int> > active_idx;
    Eigen::MatrixXf elev_map(len, len);
    Eigen::MatrixXi elev_idx(len, len);
    elev_map.setConstant(min_z);
    elev_idx.setConstant(-1);
    
    // go through all points
    int count = 0;
    for( pcl::PointCloud<PointT>::const_iterator it = cloud->begin() ; it < cloud->end() ; it++, count++ )
    {
        //if( fabs((*it).x) >= -min_x || fabs((*it).y) >= -min_y )
        //{
        //    std::cerr << "X-Y assumption failed!" << std::endl;
        //    exit(0);
        //}
        int x = floor(((*it).x - min_x) / sampleR);
        int y = floor(((*it).y - min_y) / sampleR);
        if( (*it).z > elev_map(x, y) )
        {
            if( elev_map(x, y) == min_z )
                active_idx.push_back(std::pair<int, int> (x, y));
            elev_map(x, y) = (*it).z;
            elev_idx(x, y) = count;
        }
    }
    pcl::PointCloud<PointT>::Ptr partial_cloud(new pcl::PointCloud<PointT>());
    for( std::vector< std::pair<int, int> >::iterator it_p = active_idx.begin() ; it_p < active_idx.end() ; it_p++ )
        partial_cloud->push_back(cloud->at(elev_idx((*it_p).first, (*it_p).second))); 
    return partial_cloud;
}

void genViews(const pcl::PointCloud<PointT>::Ptr full_cloud, const poseVec &rot_set, 
        CloudSet &partial_set, poseVec &partial_ground_tran, pcl::visualization::PCLVisualizer::Ptr viewer, float s1, float s2)
{
    float model_resol = 0;;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    if( s1 > 0 )
    {
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud(full_cloud);
        sor.setLeafSize(s1, s1, s1);
        sor.filter(*cloud);
        std::cerr << full_cloud->size() << "->" << cloud->size() << std::endl;
        model_resol = s1;
    }
    else
    {
        pcl::copyPointCloud(*full_cloud, *cloud);
        model_resol = computeCloudResolution(cloud);
    }
    std::cerr << "Model Resolution: " << model_resol << std::endl;
    if( viewer != NULL )
    {
        viewer->removePointCloud("model");
        viewer->addPointCloud(cloud, "model");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "model");
        viewer->spin();
        viewer->removePointCloud("model");
    }
    //shift the centroid of the cloud to origin
    pcl::PointCloud<myPointXYZ>::Ptr center(new pcl::PointCloud<myPointXYZ>());
    ComputeCentroid(cloud, center);
    Eigen::Matrix4f toOrigin = Eigen::Matrix4f::Identity();
    toOrigin(0, 3) = -center->at(0).x;
    toOrigin(1, 3) = -center->at(0).y;
    toOrigin(2, 3) = -center->at(0).z;
    pcl::transformPointCloud(*cloud, *cloud, toOrigin);
    
    int num = rot_set.size();
    partial_set.resize(num);
    partial_ground_tran.resize(num);
    
    #pragma omp parallel for schedule(dynamic,1)
    for( int i = 0 ; i < num ; i++ )
    {
        pcl::PointCloud<PointT>::Ptr tran_cloud(new pcl::PointCloud<PointT>());
        pcl::transformPointCloud(*cloud, *tran_cloud, rot_set[i]);
        
        pcl::PointCloud<PointT>::Ptr partial_cloud = createPartialView(tran_cloud, s2);
        // compute centers for initialization
        pcl::PointCloud<myPointXYZ>::Ptr partial_center(new pcl::PointCloud<myPointXYZ>());
        ComputeCentroid(partial_cloud, partial_center);
        Eigen::Matrix4f toOrigin1 = Eigen::Matrix4f::Identity();
        toOrigin1(0, 3) = -partial_center->at(0).x;
        toOrigin1(1, 3) = -partial_center->at(0).y;
        toOrigin1(2, 3) = -partial_center->at(0).z;
        pcl::transformPointCloud(*partial_cloud, *partial_cloud, toOrigin1); //push center to (0, 0, 0)
        partial_set[i] = partial_cloud;
        
        partial_ground_tran[i] = rot_set[i];
        partial_ground_tran[i](0, 3) = toOrigin1(0, 3);
        partial_ground_tran[i](1, 3) = toOrigin1(1, 3);
        partial_ground_tran[i](2, 3) = toOrigin1(2, 3);
        
        //pcl::PointCloud<PointT>::Ptr partial_ground(new pcl::PointCloud<PointT>());
        //pcl::transformPointCloud(*tran_cloud, *partial_ground, toOrigin1);
        //partial_ground_set.push_back(partial_ground);
        
        //if( partial_set.size() % 50 == 0)
        //    std::cerr<<partial_set.size()<<" ";
        if( viewer != NULL )
        {
            viewer->removePointCloud("partial_cloud");
            viewer->addPointCloud(partial_cloud, "partial_cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "partial_cloud");
            viewer->spin();
        }
    }
    std::cerr<<partial_set.size()<<std::endl;
    
}

void writePoseTxT(std::string filename, const Eigen::Matrix4f &mat)
{
    std::ofstream fp(filename.c_str(), std::ios::out);
    if(fp.is_open())
    {
        fp << mat << std::endl;
        fp.close();
    }
}

cv::Mat read2d(std::string filename)
{
    std::ifstream fp(filename.c_str());
    cv::Mat projs;
    if( fp.is_open() == false)
        return projs;

    std::vector<cv::Mat> proj_set;
    while(true)
    {
        cv::Mat temp = cv::Mat::zeros(1, 2, CV_32FC1);
        float *ptr = (float *)temp.data;
        fp >> *ptr >> *(ptr+1);
        if( *ptr < -1 )
            break;
        proj_set.push_back(temp);
    }
    cv::vconcat(proj_set, projs);
    return projs;
}

cv::Mat read3d(std::string filename)
{
    std::ifstream fp(filename.c_str());
    cv::Mat projs;
    if( fp.is_open() == false)
        return projs;

    std::vector<cv::Mat> proj_set;
    while(true)
    {
        cv::Mat temp = cv::Mat::zeros(1, 3, CV_32FC1);
        float *ptr = (float *)temp.data;
        fp >> *ptr >> *(ptr+1) >> *(ptr+2);
        if( *ptr < -1 )
            break;
        proj_set.push_back(temp);
    }
    cv::vconcat(proj_set, projs);
    return projs;
}

void FormFeaSparseMat(std::vector<SparseDataOneClass> &data_set, problem &prob, int total, int fea_dim)
{
    prob.l = total;
    prob.n = fea_dim + 1;	//include bias term
    prob.bias = -1.0;
    //prob.bias = 1.0;
    //prob.y = new double[prob.l];
    prob.y = new double[prob.l];
    prob.x = new feature_node *[prob.l];

    feature_node bias_term;
    bias_term.index = fea_dim+1;
    bias_term.value = prob.bias;
    feature_node end_node;
    end_node.index = -1;
    end_node.value = 0;

    int base = prob.l - 1;
    //for( int i = 0 ; i < data_set.size() ; i++ ){
    for(;;){
        if( data_set.empty() == true )
            break;
        std::vector< std::vector<feature_node> >::iterator it;
        int end_idx = data_set.size() - 1;
        for( it = data_set[end_idx].fea_vec.end()-1 ; it >= data_set[end_idx].fea_vec.begin() ; it--, base-- ){
            
            (*it).push_back(bias_term);
            (*it).push_back(end_node);
            prob.x[base] = new feature_node[(*it).size()];
            prob.y[base] = data_set[end_idx].label;
            std::copy((*it).begin(), (*it).end(), prob.x[base]);
        }
        data_set.erase(data_set.end()-1);
    }
    if( base != -1 )
    {
        std::cerr<<"Error: base != -1"<<std::endl;
        exit(0);
    }
}

void sparseCvMat(cv::Mat data, std::vector< std::vector<feature_node> > &fea_vec)
{
    fea_vec.clear();
    fea_vec.resize(data.rows);
    int num = data.rows;

    for( int r = 0 ; r < num; r++){
        for( int c = 0 ; c < data.cols ; c++ ){
            float cur_val = data.at<float>(r, c);
            if( fabs(cur_val) >= 1e-6 && cur_val == cur_val) 
            {
                feature_node cur_node;
                cur_node.index = c;  /****** DIFFERENT FROM SVM FORMAT *******/
                cur_node.value = cur_val;
                fea_vec[r].push_back(cur_node);
            }
        }
    }
}

void GenSVMParamter(parameter &param, float CC)
{
    param.solver_type = L2R_L2LOSS_SVC_DUAL;
    //param.solver_type = L2R_L2LOSS_SVC;
    //param.solver_type = L2R_L1LOSS_SVC_DUAL;
    param.C = CC;
    param.p = 0.1;

    param.nr_weight  = 0;
    param.weight_label = NULL;
    param.weight = NULL;
    param.init_sol = NULL;

#ifdef LIBLINEAR_WEIGHT
    param.nr_weight = 2;
    param.weight_label = (int*)std::malloc(sizeof(int) * param.nr_weight);
    param.weight = (double*)std::mallloc(sizeof(double) * param.nr_weight);
#endif
    switch(param.solver_type)
    {
        case L2R_LR:
        case L2R_L2LOSS_SVC:
            param.eps = 0.01;
            break;
        case L2R_L2LOSS_SVR:
            param.eps = 0.001;
            break;
        case L2R_L2LOSS_SVC_DUAL:
        case L2R_L1LOSS_SVC_DUAL:
        case MCSVM_CS:
        case L2R_LR_DUAL:
            param.eps = 0.01;
            break;
        case L1R_L2LOSS_SVC:
        case L1R_LR:
            param.eps = 0.01;
            break;
        case L2R_L1LOSS_SVR_DUAL:
        case L2R_L2LOSS_SVR_DUAL:
            param.eps = 0.1;
            break;
    }
}

cv::Mat readCvMatSparse(std::string filename)
{
    std::ifstream in(filename.c_str(), std::ios::in|std::ios::binary);
    if (in.is_open()==false)
       return cv::Mat::zeros(0, 0, CV_32FC1);
    
    size_t size_t_len = sizeof(size_t);
    size_t float_len = sizeof(float);
    
    size_t cols;
    size_t rows;
    size_t num;
    // Read header
    in.read((char*)&cols, size_t_len);
    in.read((char*)&rows, size_t_len);
    in.read((char*)&num, size_t_len);
    cv::Mat data = cv::Mat::zeros(rows, cols, CV_32FC1);
    float *ptr = (float *)data.data;
    for( size_t i = 0 ; i < num; i++ )
    {
        size_t idx;
        float val;
        in.read((char*)&idx, size_t_len);
        in.read((char*)&val, float_len);
        *(ptr+idx) = val;
    }
    in.close();
    return data;
}

cv::Mat sparseToMat(const std::vector<feature_node> &fea, int dim)
{
    cv::Mat dense_fea = cv::Mat::zeros(1, dim, CV_32FC1);
    float *ptr = (float *)dense_fea.data;
    for( std::vector<feature_node>::const_iterator it = fea.begin(); it < fea.end() ; it++ )
        *(ptr+(*it).index-1) = (*it).value;
    return dense_fea;
}

void mergeProbs(const std::vector<problem> &problem_set, problem &final_prob)
{
    if( problem_set.empty() == true )
        return;
    if(final_prob.l > 0)
    {
        for( int i = 0 ; i < final_prob.l ; i++ )
            free(final_prob.x[i]);
        free(final_prob.x);
        free(final_prob.y);
    }
    
    size_t total = 0;
    int fea_dim = problem_set[0].n;
    float bias = problem_set[0].bias;
    
    for( std::vector<problem>::const_iterator it = problem_set.begin() ; it < problem_set.end() ; it++ )
    {    
        total += (*it).l;
        if( (*it).n != fea_dim || (*it).bias != bias )
        {
            std::cerr << (*it).n << " " << fea_dim << std::endl;
            std::cerr << "Merging Dimensions Mismatched!" << std::endl;
            exit(0);
        }
    }
    
    final_prob.l = total;
    final_prob.n = fea_dim;	//include bias term
    final_prob.bias = bias;
    
    final_prob.y = new double[final_prob.l];
    final_prob.x = new feature_node *[final_prob.l];
    int base = 0;
    for( std::vector<problem>::const_iterator it = problem_set.begin() ; it < problem_set.end() ; it++ )
    {
        for( int j = 0 ; j < (*it).l ; j++, base++ )
        {
            if( base >= final_prob.l )
            {
                std::cerr << "base >= final_prob.l!" << std::endl;
                exit(0);
            }
            final_prob.y[base] = (*it).y[j];
            final_prob.x[base] = (*it).x[j];
        }
        if( (*it).l > 0 )
            free((*it).y);
    }
}

size_t readSoluSparse_piecewise(std::string filename, std::vector< std::vector<feature_node> > &fea_vec, std::vector< std::pair<int, int> > inds)
{
    std::ifstream in(filename.c_str(), std::ios::in|std::ios::binary);
    if (in.is_open()==false)
       return 0;
    
    size_t size_t_len = sizeof(size_t);
    size_t float_len = sizeof(float);
    
    size_t cols;
    size_t rows;
    size_t num;
    // Read header
    in.read((char*)&cols, size_t_len);
    in.read((char*)&rows, size_t_len);
    in.read((char*)&num, size_t_len);
    //std::cerr << (num+0.0)  / (rows*cols) << std::endl;
    //std::cerr << rows << " " << cols << " " << num << std::endl;
    
    size_t fea_len = cols;
    size_t data_num = rows;
    
    bool truncated = false;
    std::vector<int> piece_len;
    size_t truncated_len = 0;
    if( inds.empty() == false )
    {
        for( size_t k = 0 ; k < inds.size() ; k++ )
        {
            if( inds[k].second <= inds[k].first )
            {
                std::cerr << "inds[k].second <= inds[k].first!" << std::endl;
                exit(0);
            }
            if( k == 0 )
            {
                if( inds[k].first < 0 )
                {
                    std::cerr << "inds[k].first < 0!" << std::endl;
                    exit(0);
                }
                piece_len.push_back(inds[k].first-0);
            }
            else
            {
                if( inds[k].first < inds[k-1].second )
                {
                    std::cerr << "inds[k].first < inds[k-1].second!" << std::endl;
                    exit(0);
                }
                piece_len.push_back(inds[k].first-inds[k-1].second+piece_len[piece_len.size()-1]);
            }
            truncated_len += inds[k].second - inds[k].first;
        }
        truncated = true;
    }
    fea_vec.clear();
    fea_vec.resize(data_num);
    for( size_t i = 0 ; i < num; i++ )
    {
        
        size_t idx;
        float val;
        in.read((char*)&idx, size_t_len);
        in.read((char*)&val, float_len);
        size_t r = idx / fea_len;
        size_t c = idx % fea_len;
        if( truncated == true )
        {
            size_t k;
            for( k = 0 ; k < inds.size() ; k++ )
                if( c >= inds[k].first && c < inds[k].second )
                {    
                    c -= piece_len[k];
                    break;
                }
            if( k < inds.size() )
            {
                feature_node cur_node;
                cur_node.index = c+1;
                cur_node.value = val;
                fea_vec[r].push_back(cur_node);
            }
        }
        else
        {
            feature_node cur_node;
            cur_node.index = c+1;
            cur_node.value = val;
            fea_vec[r].push_back(cur_node);
        }
            
    }
    
    return truncated == true ? truncated_len : fea_len;
}

void saveCvMatSparse(std::string filename, const cv::Mat &data)
{
    std::ofstream out(filename.c_str(), std::ios::out|std::ios::binary);
    
    size_t cols = data.cols;
    size_t rows = data.rows;
    
    size_t size_t_len = sizeof(size_t);
    size_t float_len = sizeof(float);
    // Write header
    out.write((char*)&cols, size_t_len);
    out.write((char*)&rows, size_t_len);
    std::vector< std::pair<size_t, float> > list;
    float *ptr = (float *)data.data;
    size_t total_len = cols * rows;
    for( size_t i = 0 ; i < total_len; i++, ptr++ )
    {
        if( fabs(*ptr) >= 1e-6 && *ptr == *ptr )
            list.push_back(std::pair<size_t, float> (i, *ptr));
    }
    
    size_t len = list.size();
    out.write((char*)&len,sizeof(len));
    for( std::vector< std::pair<size_t, float> >::iterator it = list.begin() ; it < list.end() ; it++ )
    {
        out.write((char*)&((*it).first), size_t_len);
        out.write((char*)&((*it).second), float_len);
    }
    out.close();  
}

void saveCvMatSparse(std::string filename, const std::vector< sparseVec > &data, int dim)
{
    std::ofstream out(filename.c_str(), std::ios::out|std::ios::binary);
    
    size_t cols = dim;
    size_t rows = data.size();
    
    size_t size_t_len = sizeof(size_t);
    size_t float_len = sizeof(float);
    // Write header
    out.write((char*)&cols, size_t_len);
    out.write((char*)&rows, size_t_len);
    
    std::vector< std::pair<size_t, float> > list;
    for( size_t i = 0 ; i < rows ; i++ )
    {
        size_t s_idx = i * cols;
        for( sparseVec::const_iterator it = data[i].begin(); it < data[i].end(); it++ )
            list.push_back(std::pair<size_t, float> ((*it).index+s_idx, (*it).value));
    }
    
    size_t len = list.size();
    out.write((char*)&len,sizeof(len));
    for( std::vector< std::pair<size_t, float> >::iterator it = list.begin() ; it < list.end() ; it++ )
    {
        out.write((char*)&((*it).first), size_t_len);
        out.write((char*)&((*it).second), float_len);
    }
    out.close();  
}

void getNonNormalPCDFiles(std::string path, std::vector<std::string> &files)
{
    boost::filesystem::path p(path);
    std::vector< std::string > ret;
    find_files(p, ".pcd", ret);
    std::string prefix("normal");
    
    std::vector< std::string >::iterator it_p;
    for( it_p = ret.begin() ; it_p < ret.end() ; it_p++ )
    {
        if((*it_p).size() < 6 || (*it_p).substr(0, 6) != prefix )
            files.push_back(*it_p);
    }
}

cv::Mat KNNEncoder(cv::Mat data, cv::flann::Index &tree, int len, int K) //for L2 normalized data
{
    if( cv::norm(data, cv::NORM_L1) == 0 )
        return cv::Mat::zeros(1, len, CV_32FC1);
    
    if( K < 1 )
        K = 1;
    
    float beta = -4.0;
    std::vector<int> index(K);
    std::vector<float> dist(K);
    tree.knnSearch(data, index, dist, K, cv::flann::SearchParams());

    cv::Mat code = cv::Mat::zeros(1, len, CV_32FC1);
    float *ptr = (float *)code.data;
    if( K == 1 )
        *(ptr + index[0]) = 1.0;
    else
    {
        float norm = 0;
        for( int i = 0 ; i < K ; i++ )
        {
            //dist[i] = (2.0 - dist[i])/2.0;
            //dist[i] = 10.0 / (dist[i]+0.01);
            dist[i] = exp(beta*dist[i]);
            //if( index[i] >= len || index[i] < 0)
            //    std::cerr <<"*****" << index[i] << std::endl;

            *(ptr + index[i]) = dist[i];
            norm += dist[i];
        }
//        norm = sqrt(norm);
        for( int i = 0 ; i < K ; i++ )
            *(ptr + index[i]) = dist[i]/norm;
    }
    
    return code;
}

cv::Mat TriEncoder(cv::Mat data, cv::Mat dict)
{
    float beta = -4.0;
    if( cv::norm(data, cv::NORM_L1) == 0 )
        return cv::Mat::zeros(1, dict.rows, CV_32FC1);
    
    cv::Mat code = data * dict.t();
    code = 2.0 - 2*code;
    cv::Scalar tmp = cv::mean(code);
    float avg = tmp.val[0];
    float *ptr = (float *)code.data;
    //int count = 0;
    for( int i = 0 ; i < code.cols ; i++, ptr++ )
    {
        //if( *ptr < avg )
        //    count++;
        *ptr = *ptr < avg ? exp(beta*(*ptr)) : 0;
    }
    /*
    cv::Mat code = cv::Mat::zeros(1, dict.rows, CV_32FC1);
    float *ptr = (float *)code.data;
    float sum_dist = 0;
    float min = 1000;
    int min_idx = -1;
    int num = dict.rows;
    for( int i = 0 ; i < num ; i++, ptr++ )
    {
        *ptr = cv::norm(data-dict.row(i), cv::NORM_L2);
        std::cerr << *ptr << std::endl;
        sum_dist += *ptr;
        if( *ptr < min )
        {
            min = *ptr;
            min_idx = i;
        }
    }
    std::cerr << " **** " << min <<" "<<min_idx<< std::endl;
    std::cin.get();
            
    sum_dist /= num;
    ptr = (float *)code.data;
    int count = 0;
    for( int i = 0 ; i < num ; i++, ptr++ )
    {
        if( *ptr < sum_dist )
            count++;
        *ptr = *ptr < sum_dist ? exp(beta*(*ptr)) : 0;
    }
    */
    //std::cerr << count << std::endl;
    //std::cin.get();
    return code;
}

void MaxOP(cv::Mat dom, cv::Mat sub)
{
    float *ptr1 = (float *)dom.data;
    float *ptr2 = (float *)sub.data;

    for( int i = 0 ; i < dom.cols; i++, ptr1++, ptr2++ ){
        if( *ptr2 > *ptr1 )
            *ptr1 = *ptr2;
    }
}

cv::Mat getHSIHist(const pcl::PointCloud<PointT>::Ptr ori_cloud, cv::flann::Index &dict, int len, int K, float ss)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    if( ss > 0 )
    {
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud(ori_cloud);
        sor.setLeafSize(ss, ss, ss);
        sor.filter(*cloud);
    }
    else
        pcl::copyPointCloud(*ori_cloud, *cloud);
    
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>());
    kdtree->setInputCloud(cloud);
    
    cv::Mat diff = cv::Mat::zeros(cloud->size(), len, CV_32FC1);
    for( int i = 0 ; i < cloud->size() ; i++ )
    {
        std::vector<int> ind;
        std::vector<float> dist;
        kdtree->radiusSearch(cloud->at(i), 0.01, ind, dist, cloud->size());
        
        cv::Mat this_code = cv::Mat::zeros(1, len, CV_32FC1);
        for( int j = 0 ; j < ind.size() ; j++ )
        {
            uint32_t tmp = cloud->at(ind[j]).rgba;
            int rgba[3];
            rgba[0] = (tmp >> 16) & 0x0000ff;
            rgba[1] = (tmp >> 8)  & 0x0000ff;
            rgba[2] = (tmp)       & 0x0000ff;
            float hsi[3];
            RGBToHSI(rgba, hsi);
        
            cv::Mat cur_hsi = cv::Mat::zeros(1, 3, CV_32FC1);
            cur_hsi.at<float>(0, 0) = hsi[0];
            cur_hsi.at<float>(0, 1) = hsi[1];
            cur_hsi.at<float>(0, 2) = hsi[2];
            
            cv::Mat cur_code = KNNEncoder(cur_hsi, dict, len, K);
            MaxOP(this_code, cur_code);
        }
        cv::normalize(this_code, this_code);
        this_code.copyTo(diff.row(i));
    }
    return diff;
}

cv::Mat gen3DNormCube(int len)
{
    float scale = 1.00000001/len;
    int total = len*len*len;
    std::vector<float> seeds(len);
    for( int i = 0 ; i < len ; i++ )
        seeds[i] = scale *(i+0.5);
    
    cv::Mat dict = cv::Mat::zeros(total, 3, CV_32FC1);
    float *ptr = (float *)dict.data;
    for( int i = 0 ; i < len ; i++ ){
        for( int j = 0 ; j < len ; j++ ){
            for( int k = 0 ; k < len ; k++ ){
                *ptr = seeds[i];
                ptr++;
                *ptr = seeds[j];
                ptr++;
                *ptr = seeds[k];
                ptr++;
            }
        }
    }
    return dict;
}

bool PreCloud(MulInfoT &data, float ss, bool light_flag)
{
    if( data.cloud->empty() == true )
        return false;
    
    if( data.down_cloud->empty() == true )
    {
        if( ss <= 0 )
            data.down_cloud = data.cloud;
        else
        {
            pcl::VoxelGrid<PointT> sor;
            sor.setInputCloud(data.cloud);
            sor.setLeafSize(ss, ss, ss);
            sor.filter(*data.down_cloud);
            if( data.down_cloud->size() == data.cloud->size() )     //downsampling failed
                return false;
        }
    }
    
    int num = data.down_cloud->size();
    data.rgb = cv::Mat::zeros(num, 3, CV_32FC1);      //actually lab
 
    float max_x = -1000, min_x = 1000, max_y = -1000, min_y = 1000, max_z = -1000, min_z = 1000;
    float *ptr_lab = (float *)data.rgb.data;
    for( pcl::PointCloud<PointT>::iterator it = data.down_cloud->begin(); it < data.down_cloud->end() ; it++ )
    {
        uint32_t rgb = *reinterpret_cast<int*>(&((*it).rgb));
        int rgba[3];
        rgba[0] = (rgb >> 16) & 0x0000ff;
        rgba[1] = (rgb >> 8)  & 0x0000ff;
        rgba[2] = (rgb)       & 0x0000ff;
        float lab[3];
        RGBToLab(rgba, lab);
        
        *ptr_lab = lab[0];ptr_lab++;
        *ptr_lab = lab[1];ptr_lab++;
        *ptr_lab = lab[2];ptr_lab++;
        
        if( light_flag == false )
        {
            float x = (*it).x, y = (*it).y, z = (*it).z;
            if( max_x < x ) max_x = x;
            if( max_y < y ) max_y = y;
            if( max_z < z ) max_z = z;

            if( min_x > x ) min_x = x;
            if( min_y > y ) min_y = y;
            if( min_z > z ) min_z = z;
        }
    }
    if( light_flag == false )
    {
        data.xyz = cv::Mat::zeros(num, 3, CV_32FC1);
        float *ptr_xyz = (float *)data.xyz.data;
        float range_x = max_x - min_x + 0.0001;
        float range_y = max_y - min_y + 0.0001;
        float range_z = max_z - min_z + 0.0001;
        for( pcl::PointCloud<PointT>::iterator it = data.down_cloud->begin() ; it < data.down_cloud->end() ; it++ )
        {
            *ptr_xyz = ((*it).x - min_x) / range_x;
            ptr_xyz++;
            *ptr_xyz = ((*it).y - min_y) / range_y;
            ptr_xyz++;
            *ptr_xyz = ((*it).z - min_z) / range_z;
            ptr_xyz++;
        }
    }
    return true;
}

MulInfoT convertPCD(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normal)
{
    if( cloud_normal->empty() == false && cloud->size() != cloud_normal->size() )
    {
        std::cerr << cloud->size() << " " << cloud_normal->size() << std::endl;
        std::cerr << "cloud->size() != cloud_normal->size()" << std::endl;
        exit(0);
    }
    MulInfoT data;
    data.cloud = cloud;
    data.cloud_normals = cloud_normal;
    
    data.down_cloud = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>());
    data.down_lrf = pcl::PointCloud<pcl::ReferenceFrame>::Ptr (new pcl::PointCloud<pcl::ReferenceFrame>());
        
    return data;
}

int readCSV(std::string filename, std::string label, std::vector<poseT> &poses)
{
    std::ifstream fp;
    fp.open(filename.c_str());
    if(fp.is_open() == false)
        return 0;
    
    int num;
    fp >> num;
    for(int i = 0 ; i < num ; i++)
    {
        poseT tmp;
        fp >> tmp.shift(0) >> tmp.shift(1) >> tmp.shift(2);
        float x,y,z,w;
        fp >> x >> y >> z >> w;
        tmp.rotation = Eigen::Quaternionf (w,x,y,z);
        tmp.model_name = label;
        
        poses.push_back(tmp);
    }
    fp.close();
    return 1;
}

int writeCSV(std::string filename, std::string label, const std::vector<poseT> &poses)
{
    std::ofstream fp;
    fp.open(filename.c_str());
    if( fp.is_open() == false )
    if( fp.is_open() == false )
    {
        std::cerr << "Failed to open files" << std::endl;
        return 0;
    }
    
    int count=0;
    for( std::vector<poseT>::const_iterator it = poses.begin() ; it < poses.end() ; it++ )
        if( it->model_name == label )
            count++;
    
    fp << count << std::endl;
    for( std::vector<poseT>::const_iterator it = poses.begin() ; it < poses.end() ; it++ )
    {
        if( it->model_name == label )
        {
            fp << it->shift(0) << " " << it->shift(1) << " " << it->shift(2) << " "
               << it->rotation.x() << " " << it->rotation.y() << " " << it->rotation.z() << " " << it->rotation.w() << std::endl;
            /*
            fp << it->tran(0, 0) << " " << it->tran(0, 1) << " " << it->tran(0, 2) << " " << it->tran(0, 3) << " "
               << it->tran(1, 0) << " " << it->tran(1, 1) << " " << it->tran(1, 2) << " " << it->tran(1, 3) << " "
               << it->tran(2, 0) << " " << it->tran(2, 1) << " " << it->tran(2, 2) << " " << it->tran(2, 3) << " "
               << it->tran(3, 0) << " " << it->tran(3, 1) << " " << it->tran(3, 2) << " " << it->tran(3, 3) << std::endl;
            */
        }
    }
    fp.close();
    return 1;
}

void ReadCloudNormal(const std::string path, std::vector<std::string> &pcd_files, std::vector<std::string> &normal_files)
{
    std::vector<std::string> root_pcd;
    getNonNormalPCDFiles(path, root_pcd);
    
    for( size_t i = 0 ; i < root_pcd.size() ; i++ )
    {
        std::string cur_normal = path + "normal_" + root_pcd[i];
        std::string cur_pcd = path + root_pcd[i];
        if( exists_test(cur_normal) == true )
        {
            pcd_files.push_back(cur_pcd);
            normal_files.push_back(cur_normal);
        }
    }
}

/*
    int num = cloud->size();
    pcl::PointCloud<myPointXYZ>::Ptr lab_cloud(new pcl::PointCloud<myPointXYZ>());
    
    if( light_flag == false )
    {
        data.xyz_tree = pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>());
        data.xyz_tree->setInputCloud(cloud);
        data.normal_tree = pcl::search::KdTree<NormalT>::Ptr (new pcl::search::KdTree<NormalT>());
        data.normal_tree->setInputCloud(cloud_normal);
        data.lab_tree = pcl::search::KdTree<myPointXYZ>::Ptr (new pcl::search::KdTree<myPointXYZ>());
        lab_cloud->resize(num); 
        
        data.normal = cv::Mat::zeros(num, 3, CV_32FC1);
    }
    data.rgb = cv::Mat::zeros(num, 3, CV_32FC1);      //actually hsi

    pcl::PointCloud<PointT>::iterator it;
    pcl::PointCloud<NormalT>::iterator it_normal; 
    pcl::PointCloud<myPointXYZ>::iterator it_lab;  
    
    float *ptr_hsi = (float *)data.rgb.data;
    float *ptr_normal = (float *)data.normal.data;
    float max_x = -1000, min_x = 1000, max_y = -1000, min_y = 1000, max_z = -1000, min_z = 1000;
    for( it = cloud->begin(), it_normal = cloud_normal->begin(), it_lab = lab_cloud->begin() ; it < cloud->end() ; it++, it_normal++, *it_lab++ )
    {
        float x = (*it).x, y = (*it).y, z = (*it).z;
        if( max_x < x ) max_x = x;
        if( max_y < y ) max_y = y;
        if( max_z < z ) max_z = z;
        
        if( min_x > x ) min_x = x;
        if( min_y > y ) min_y = y;
        if( min_z > z ) min_z = z;

        uint32_t rgb = *reinterpret_cast<int*>(&((*it).rgb));
        int rgba[3];
        rgba[0] = (rgb >> 16) & 0x0000ff;
        rgba[1] = (rgb >> 8)  & 0x0000ff;
        rgba[2] = (rgb)       & 0x0000ff;
        //float sum = sqrt(rgba[0]*rgba[0]+rgba[1]*rgba[1]+rgba[2]*rgba[2]);
        float hsi[3];
        //RGBToHSI(rgba, hsi);
        RGBToLab(rgba, hsi);
        *ptr_hsi = hsi[0];ptr_hsi++;
        *ptr_hsi = hsi[1];ptr_hsi++;
        *ptr_hsi = hsi[2];ptr_hsi++;
        
        if( light_flag == false )
        {
            (*it_lab).x = hsi[0];
            (*it_lab).y = hsi[1];
            (*it_lab).z = hsi[2];

            if( cloud_normal->empty() == false )
            {
                if( (*it_normal).normal_z == (*it_normal).normal_z )
                {
                    *ptr_normal = (*it_normal).normal_x;
                    ptr_normal++;
                    *ptr_normal = (*it_normal).normal_y;
                    ptr_normal++;
                    *ptr_normal = (*it_normal).normal_z;
                    ptr_normal++;
                }
                else
                {
                    *ptr_normal = 0;
                    ptr_normal++;
                    *ptr_normal = 0;
                    ptr_normal++;
                    *ptr_normal = 0;
                    ptr_normal++;
                }
            }
        }
    }
    if( light_flag == false )
    {
        data.lab_tree->setInputCloud(lab_cloud);
    
        data.xyz = cv::Mat::zeros(num, 3, CV_32FC1);
        float *ptr_xyz = (float *)data.xyz.data;
        float range_x = max_x - min_x + 0.0001;
        float range_y = max_y - min_y + 0.0001;
        float range_z = max_z - min_z + 0.0001;
        for( it = cloud->begin(), it_normal = cloud_normal->begin() ; it < cloud->end() ; it++, it_normal++ )
        {
            *ptr_xyz = ((*it).x - min_x) / range_x;
            ptr_xyz++;
            *ptr_xyz = ((*it).y - min_y) / range_y;
            ptr_xyz++;
            *ptr_xyz = ((*it).z - min_z) / range_z;
            ptr_xyz++;
        }
    }
    */
