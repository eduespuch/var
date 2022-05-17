//Libraries grouped by:

	//Common use
	#include <iostream>
	#include <string>
	#include <vector>
	#include <sys/resource.h>

	//Common PCL
	//#include <pcl/memory.h>  // for pcl::make_shared da error, comprobar por que
	#include <pcl/point_cloud.h>
	#include <pcl/point_types.h>
	#include <pcl/point_representation.h>
	#include <pcl/io/pcd_io.h>
	#include <pcl/common/io.h> // for concatenateFields
	#include <pcl/search/kdtree.h>
	#include <pcl/kdtree/impl/kdtree_flann.hpp>//ojo
	#include <pcl/recognition/cg/geometric_consistency.h>//ojo
	#include <pcl/common/transforms.h>//ojo

	//Filters
	#include <pcl/filters/voxel_grid.h>
	#include <pcl/filters/filter.h>

	//Keypoints
	#include <pcl/keypoints/sift_keypoint.h>
	#include <pcl/keypoints/iss_3d.h>
	#include <pcl/keypoints/harris_3d.h>
	
	//Descriptors
	#include <pcl/features/normal_3d.h>
	#include <pcl/features/fpfh.h>
	#include <pcl/features/cvfh.h>
	#include <pcl/features/shot.h>

	//Correspondeces
	#include <pcl/correspondence.h>
	#include <pcl/sample_consensus/ransac.h>//ojo
	#include <pcl/registration/correspondence_estimation.h>
	#include <pcl/registration/correspondence_rejection_sample_consensus.h>
	#include <pcl/registration/sample_consensus_prerejective.h>//ojo
	
	#include <pcl/registration/icp.h>//ojo
	#include <pcl/registration/icp_nl.h>
	#include <pcl/registration/transforms.h>
	
	//Visualization
	#include <pcl/visualization/pcl_visualizer.h>
	#include <pcl/visualization/cloud_viewer.h>

	//Others

	#include <boost/thread/thread.hpp>
	#include <boost/foreach.hpp>
//Parameters definition
	//Method selection
		/*
		Keypoints methods	| Descriptors methods
		1- SIFT				| 1- FPFH
		2- ISS				| 2- CVFH
		3- HARRIS			| 3- SHOT
		*/
		#define KeypointsMethod 1
		#define DescriptorsMethod 1

		//Selected descriptor type must me same type of descriptor method
		#if DescriptorsMethod==1// 1 == FPFH
			#define DescriptorType pcl::FPFHSignature33 
		#elif DescriptorsMethod==2// 2 == CVFH
			#define DescriptorType pcl::VFHSignature308 
		#elif DescriptorsMethod==3// 3 == SHOT352
			#define DescriptorType pcl::SHOT352 
		#endif
	//Keypoints configuration

		//SIFT PARAMETERS
		#define SIFT_MIN_SCALE 0.01f
		#define SIFT_N_OCTAVES 3
		#define SIFT_N_SCALES_OCTAVE 4
		#define SIFT_MINIMUM_CONTRAST 0.0001f

		// ISS PARAMETERS
		#define ISS_SALIENT_RADIUS 6
		#define ISS_NON_MAX_RADIUS 4
		#define ISS_MIN_NEIGHBORS 5
		#define ISS_21_THRESHOLD 0.975
		#define ISS_32_THRESHOLD 0.975


		//HARRIS PARAMETERS
		#define HARRIS_NON_MAX_SUPRESSION true
		#define HARRIS_THRESHOLD 1e-9

	//Descriptors configuration

		// FPFH PARAMETERS
		#define FPFH_RADIUS_SEARCH 0.015

		// SHOT352 PARAMETERS
		#define SHOT352_RADIUS_SEARCH 0.05

		// CVFH PARAMETERS
		#define CVFH_EPS_ANGLE_THRESHOLD 5.0 / 180.0 * M_PI
		#define CVFH_CURVATURE_THRESHOLD 1.0
		#define CVFH_NORMALIZE_BINS false
	//Correspondeces configuration
		#define RANSAC_MAX_ITERATIONS 3000
		#define RANSAC_INLIER_THRESHOLD 0.05

		#define ICP_NORMAL_SEARCH 30
		#define ICP_MAX_ITERATIONS 40
		#define ICP_MAX_CORRESPONDENCE_DISTANCE 0.1
		#define ICP_TRANSFORMATION_EPSILON 1e-6
		#define ICP_EUCLIDEAN_FITNESS_EPSILON 1

	//Others
		#define NORMALS_K_NEIGHBORS 30
		#define NORMALS_RADIUS_SEARCH 0.01f
		
		#define LEAF_SIZE 0.05f

		#define DIRECTORY "src/data/dir"

		#define DEBUG_MSG 1

		#define DEBUG_VIS 0

		#define Method_Test 1

		#define RANSACMethod 1

		using namespace std;

		using pcl::visualization::PointCloudColorHandlerGenericField;
		using pcl::visualization::PointCloudColorHandlerCustom;


//Debugger visualizer

	//our visualizer
	pcl::visualization::PCLVisualizer *p;
	//its left and right viewports
	int vp_1, vp_2;

//definition of auxiliar types

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
	
/**
 * @brief 
 * 
 */
struct PCD{
  PointCloud::Ptr cloud;
  string f_name;

  PCD() : cloud (new PointCloud) {};
};

/**
 * @brief 
 * 
 */
struct PCDComparator{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};


/**
 * @brief Define a new point representation for < x, y, z, curvature >
 * 
 */
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>{
	using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
	public:
	MyPointRepresentation (){
		// Define the number of dimensions
		nr_dimensions_ = 4;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray (const PointNormalT &p, float * out) const{
		// < x, y, z, curvature >
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};

//Visualization methods

/**
 * @brief Cleans the console and shows a process bar of the actual progress
 * 
 * @param iter actual iteration
 * @param totalIter total iteration
 */
void processBar(int iter, int totalIter){
	system("clear");
	cout<<"\n Iteration of the loop number "<<iter<<"\n";
	//por quedar bonito
	cout<<"Process: [";
	for(int j=0;j<20;j++){
		if(100/20*j<(double)100/totalIter*iter){
			cout<<"#";
		}else{
			cout<<" ";
		}
	}
	cout<<"]\t"<<int((double)100/totalIter*iter)<<"%\n\n";	
}

/**
 * @brief  Display source and target on the first viewport of the visualizer
 * 
 * @param cloud_target targeted point cloud
 * @param cloud_source  source point cloud
 */
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source){
  p->removePointCloud ("vp1_target");
  p->removePointCloud ("vp1_source");

  PointCloudColorHandlerCustom<PointType> tgt_h (cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<PointType> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

  PCL_INFO ("Press q to begin the registration.\n");
  p-> spin();
}

/**
 * @brief Display source and target on the second viewport of the visualizer
 * 
 * @param cloud_target targeted point cloud
 * @param cloud_source  source point cloud
 */
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source){
  p->removePointCloud ("source");
  p->removePointCloud ("target");


  PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
  if (!tgt_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");

  PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
  if (!src_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");


  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
  p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

  p->spinOnce();
}

/**
 * @brief Draws a simple configuration point cloud
 * 
 * @param cloud point cloud to be drawn
 */
void simpleVis(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
  	pcl::visualization::CloudViewer viewer ("Iterative Cloud Viewer");
	
    while(!viewer.wasStopped())
	{
		
		
	  viewer.showCloud(cloud);
	  //boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	  //viewer.showCloud (target_pc);
	  boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
	}
    
}
/**
 * @brief Draws the point cloud with keypoints and normals marked
 * 
 * @param cloud point cloud to be drawn
 * @param keypoints point cloud of keypoints
 * @param normals point cloud of normal, expected from the cloud
 */
void complexVis(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints,
						pcl::PointCloud<pcl::Normal>::Ptr normals){
   pcl::visualization::PCLVisualizer viewer("Keypoint visualizer");
   viewer.setBackgroundColor(0, 0, 0);
   viewer.addPointCloud(cloud, "cloud");       
   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2, "cloud");   
   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1, 1, 1, "cloud");
   viewer.addPointCloud(keypoints, "keypoints");      
   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,8, "keypoints");
   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.0, 0.0, 1, "keypoints");
   viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
    while (!viewer.wasStopped())
   {      
       viewer.spinOnce();
   }
   /*viewer.spinOnce();
   boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
   */viewer.removePointCloud("cloud");
   viewer.removePointCloud("keypoints");
   viewer.removePointCloud("normals");
}

//Auxiliar methods

/**
 * @brief Load a set of PCD files that we want to register together.
 * \attention Expected to use a directory with only pcd files. No doing this may result on an unexpected error.
 * 
 * @param dir directory where the pcd files are located
 * @param totalSamples total of files to read
 * @param models the resultant vector of point cloud datasets
 */
void loadData(const string& dir, const int totalSamples, vector<PCD, Eigen::aligned_allocator<PCD> > &models){
  string extension (".pcd");
  // Suppose the first argument is the actual test model
  for (int i = 1; i < totalSamples; i++){

      // Load the cloud and saves it into the global list of models
      PCD m;
      m.f_name = dir+"/"+to_string(i-1)+".pcd";
      pcl::io::loadPCDFile (m.f_name, *m.cloud);
      //remove NAN points from the cloud
      vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

      models.push_back (m);
  }
}

/**
 * @brief Loads the source and target point clouds from a specific directory for a given iteration on the loop
 * @deprecated use loadData instead
 * @param dir directory with samples
 * @param iter actual iteration
 * @param source source point cloud, already on the map
 * @param target target point cloud, soon to be added on the map
 * @return true pcd files are loaded
 * @return false pcd files aren't loaded
 */
bool loadPointClouds(const string& dir, const int iter, 
						pcl::PointCloud<PointType>::Ptr source,
						pcl::PointCloud<PointType>::Ptr target){

	if(pcl::io::loadPCDFile<PointType>(dir+"/"+to_string(iter-1)+".pcd",*source)==-1){
		return false;
	}
	if(pcl::io::loadPCDFile<PointType>(dir+"/"+to_string(iter)+".pcd",*target)==-1){
		return false;
	}

	#if DEBUG_MSG==1
		cout<<"\tLoaded last point cloud from "<<dir<<"/"<<to_string(iter-1)<<".pcd with "<<source->size()<<" points\n";
		cout<<"\tLoadad actual point cloud from "<<dir<<"/"<<to_string(iter)<<".pcd with "<<target->size()<<" points\n";
	#endif
	return true;
}

/**
 * @brief It return the number of files on a given directory
 * 
 * @param i directory of reference
 * @return int number of files on the given directory
 */
int getTotalSamples(const string& i){
   struct stat dir;
   ifstream file;
   int totalSamples=0;
   // Compruebo la existencia del directorio
   int err=stat(i.c_str(), &dir);
 
   if(err==-1 || !S_ISDIR(dir.st_mode))
       return false;
   else {
       // obtengo el total de ficheros en una lista
       string cmd="ls "+i+" | wc -l > .num_fich"; //Extrae todas las rutas de los ficheros a un archivo .num_fich
       system(cmd.c_str()); //Ejecuta el comando cmd en el tablero de comandos
   }
 
   file.open(".num_fich");
   if(file.is_open()){
       while(file>>totalSamples){
       }
       file.close();
   }else{
       cerr << "ERROR: fichero \".num_fich\" no existe"<< endl;
       return false;
   }
   string cmd="rm -rf .num_fichero";
   system(cmd.c_str());
   return totalSamples;
}

/**
 * @brief Removes NaN points from a point cloud
 * 
 * @param cloud cloud of points to modify
 */
void remove_nan(pcl::PointCloud<PointType>::Ptr cloud){


	cout << "\tNumber of points before remove_nan: " << cloud->size() << "\n";

	pcl::PointCloud<PointType>::Ptr output(new pcl::PointCloud<PointType>());
	vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *output, indices);
	*cloud = *output;


	cout << "\tNumber of points after remove_nan: " << cloud->size() << "\n";

}

/**
 * @brief Updates de Global transform Matrix and modifies the target point cloud
 * 
 * @param transform Actual iteration transformation matrix
 * @param transform_total Global transform Matrix
 * @param cloud targeted cloud of points 
 * @param transformedCloud expected cloud of points to be modified 
 */
void transform_cloud(const Eigen::Matrix4f &transform, Eigen::Matrix4f &transform_total,
						const pcl::PointCloud<PointType>::ConstPtr &cloud,
						pcl::PointCloud<PointType>::Ptr &transformedCloud){
	transform_total *= transform;
	pcl::transformPointCloud(*cloud, *transformedCloud, transform_total);

	#if DEBUG_MSG==1
		cout << "\tTActualTransform matrix: \n" << transform << "\n";
		cout << "\tTransformTotal matrix: \n" << transform_total << "\n";
	#endif
}



/**
 * @brief aplies downsampling using preestablished parameters (LEAF_SIZE)
 * 
 * @param cloud given point cloud
 * @param cloud_filtered point cloud with the filtered result
 */
void filter_voxel_grid(const pcl::PointCloud<PointType>::ConstPtr& cloud,
						pcl::PointCloud<PointType>::Ptr cloud_filtered){
	pcl::VoxelGrid<PointType> v_grid;
	v_grid.setInputCloud(cloud);
	v_grid.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
	v_grid.filter(*cloud_filtered);

	#if DEBUG_MSG==1
		cout << "\tNumber of points after VoxelGrid: " << cloud_filtered->size() << "\n";
	#endif

}

/**
 * @brief Estimates de local normals of a point cloud
 * 
 * @param cloud given point cloud
 * @param normals normal cloud with the result
 */
void estimate_normals(const pcl::PointCloud<PointType>::ConstPtr& cloud,
						pcl::PointCloud<pcl::Normal>::Ptr normals){


	pcl::NormalEstimation<PointType, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
	ne.setSearchMethod(tree);
	//radio de vecinos
	ne.setRadiusSearch(NORMALS_RADIUS_SEARCH);
	//numero de vecinos
	ne.setKSearch(NORMALS_K_NEIGHBORS);

	ne.compute(*normals);

	
	#if DEBUG_MSG==1
		cout << "\tNumber of normal estimated: " << normals->size() << "\n";
	#endif

}

/**
 * @brief Get the actual cpu time 
 * 
 * @return double actual cpu time
 */
double get_cpu_time(void){
    struct timeval tim;
    struct rusage ru;
    getrusage(RUSAGE_SELF, &ru);
    tim=ru.ru_utime;
    double t=(double)tim.tv_sec + (double)tim.tv_usec / 1000000.0;
    tim=ru.ru_stime;
    t+=(double)tim.tv_sec + (double)tim.tv_usec / 1000000.0;
    return t;
}



//Keypoints extraction methods

/**
 * @brief Calculates de spatial resolution of a point cloud given using the average distance between a point and its closest neighbour
 * 
 * @param cloud point cloud given
 * @return double point cloud resolution
 */
double get_cloud_resolution(const pcl::PointCloud<PointType>::ConstPtr& cloud){
	double res = 0.0;
  	int n_points = 0, n_res;
  	vector<int> indices (2);
  	vector<float> sqr_distances (2);
  	pcl::search::KdTree<PointType> tree;
  	tree.setInputCloud(cloud); 
	for(size_t i=0;i<cloud->size();++i) {
		//Si es un elemento NaN (la posicion x del punto lo es al menos), se salta a la siguiente iteracion
		if (!isfinite((*cloud)[i].x)) 
			continue;
		//Se hace para todos los puntos con valores
		n_res = tree.nearestKSearch (i, 2, indices, sqr_distances); 
		if (n_res == 2) {
      		res += sqrt(sqr_distances[1]);
      		++n_points;
    	} 
	}
	if(n_points != 0)
		res /= n_points;
	return res;
}

/**
 * @brief Obtains the keypoints of cloud point using ISSKeypoint3D
 * 
 * @param cloud inpunt point cloud
 * @param keypoints output point cloud, referencing keypoints
 */
void iss_keypoints(const pcl::PointCloud<PointType>::ConstPtr& cloud,
					pcl::PointCloud<PointType>::Ptr& keypoints){

	pcl::ISSKeypoint3D<PointType, PointType> iss_detector;	
	//Get cloud resolution
	double actual_res=get_cloud_resolution(cloud);
	//iss_detector
	iss_detector.setInputCloud(cloud);
	// Set the radius of the spherical neighborhood used to compute the scatter matrix.
	iss_detector.setSalientRadius(ISS_SALIENT_RADIUS*actual_res);
	// Set the radius for the application of the non maxima supression algorithm.
	iss_detector.setNonMaxRadius(ISS_NON_MAX_RADIUS*actual_res);
	// Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
	iss_detector.setMinNeighbors(ISS_MIN_NEIGHBORS);
	// Set the upper bound on the ratio between the second and the first eigenvalue.
	iss_detector.setThreshold21(ISS_21_THRESHOLD);
	// Set the upper bound on the ratio between the third and the second eigenvalue.
	iss_detector.setThreshold32(ISS_32_THRESHOLD);
	iss_detector.compute(*keypoints);
	
	#if DEBUG_MSG==1
		cout << "\tNumber of keypoints with ISS detector: " << keypoints->size() << "\n";
	#endif
}

/**
 * @brief Obtains the keypoints of cloud point using SIFT
 * 
 * @param cloud inpunt point cloud
 * @param keypoints output point cloud, referencing keypoints
 */					
void sift_keypoints(const pcl::PointCloud<PointType>::ConstPtr& cloud,
					pcl::PointCloud<PointType>::Ptr& keypoints){
	pcl::SIFTKeypoint<PointType, pcl::PointWithScale> sift;
	pcl::PointCloud<pcl::PointWithScale> result;
	pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
	sift.setSearchMethod(tree);
	// min_scale, n_octaves, n_scales_per_octave
	sift.setScales(SIFT_MIN_SCALE, SIFT_N_OCTAVES, SIFT_N_SCALES_OCTAVE);
	sift.setMinimumContrast(SIFT_MINIMUM_CONTRAST);
	sift.setInputCloud(cloud);
	sift.compute(result);
	copyPointCloud(result, *keypoints);
	
	#if DEBUG_MSG==1
		cout << "\tNumber of keypoints with SIFT detector: " << keypoints->size() << "\n";
	#endif
}

/**
 * @brief Obtains the keypoints of cloud point using HARRISKeypoint3D
 * 
 * @param cloud inpunt point cloud
 * @param keypoints output point cloud, referencing keypoints
 */
void harris_keypoints(const pcl::PointCloud<PointType>::ConstPtr& cloud,
						pcl::PointCloud<PointType>::Ptr& keypoints){
	pcl::HarrisKeypoint3D<PointType, pcl::PointXYZI> detector;
	pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>);
	detector.setNonMaxSupression(HARRIS_NON_MAX_SUPRESSION);
	detector.setInputCloud(cloud);
	detector.setThreshold(HARRIS_THRESHOLD);
	detector.compute(*result);
	copyPointCloud(*result, *keypoints);
	
	#if DEBUG_MSG==1
		cout << "\tNumber of keypoints with HARRIS detector: " << keypoints->size() << "\n";
	#endif
}
//Descriptors definition methods



/**
 * @brief Compute the descriptors using SHOT352
 * 
 * @param keypoints input Keypoints
 * @param cloud input point clouds
 * @param descriptors output point cloud referencing the descriptors
 */
void SHOT352_descriptors(const pcl::PointCloud<PointType>::ConstPtr& keypoints,
							const pcl::PointCloud<PointType>::ConstPtr& cloud,
							pcl::PointCloud<pcl::SHOT352>::Ptr& descriptors){
	
	pcl::SHOTEstimation<PointType, pcl::Normal, pcl::SHOT352> shot;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	estimate_normals(keypoints, normals);
	shot.setRadiusSearch(SHOT352_RADIUS_SEARCH);
	shot.setInputCloud(keypoints);
	shot.setInputNormals(normals);
	shot.setSearchSurface(cloud);
	shot.compute(*descriptors);

	#if DEBUG_MSG==1
		cout << "\tNumber of descriptors with SHOT352: " << descriptors->size() << "\n";
	#endif
}

/**
 * @brief Compute the descriptors using the "FAST" PFH variant
 * 
 * @param keypoints input Keypoints
 * @param descriptors output point cloud referencing the descriptors
 */
void FPFH_descriptors(const pcl::PointCloud<PointType>::ConstPtr& keypoints,
						pcl::PointCloud<pcl::FPFHSignature33>::Ptr& descriptors){
	
	pcl::FPFHEstimation<PointType, pcl::Normal, pcl::FPFHSignature33> fpfh;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	estimate_normals(keypoints, normals);
	fpfh.setInputCloud(keypoints);
	fpfh.setInputNormals(normals);
	pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>());
	fpfh.setSearchMethod(tree);
	// Radio de busqueda, tiene que ser mayor que el utilizado al calcular las normales
	fpfh.setRadiusSearch(FPFH_RADIUS_SEARCH);
	fpfh.compute(*descriptors);

	#if DEBUG_MSG==1
		cout << "\tNumber of descriptors with FPFH: " << descriptors->size() << "\n";
	#endif
}

/**
 * @brief Compute the descriptors using CVFH
 * 
 * @param keypoints input Keypoints
 * @param descriptors output point cloud referencing the descriptors
 */
void CVFH_descriptors(const pcl::PointCloud<PointType>::ConstPtr& keypoints,
						pcl::PointCloud<pcl::VFHSignature308>::Ptr& descriptors){
	pcl::CVFHEstimation<PointType, pcl::Normal, pcl::VFHSignature308> cvfh;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	estimate_normals(keypoints, normals);	
	cvfh.setInputCloud(keypoints);
	cvfh.setInputNormals(normals);
	pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>());
	cvfh.setSearchMethod(tree);
	// Set the maximum allowable deviation of the normals,
	// for the region segmentation step.
	cvfh.setEPSAngleThreshold(CVFH_EPS_ANGLE_THRESHOLD); // 5 degrees.
	// Set the curvature threshold (maximum disparity between curvatures),
	// for the region segmentation step.
	cvfh.setCurvatureThreshold(CVFH_CURVATURE_THRESHOLD);
	// Set to true to normalize the bins of the resulting histogram,
	// using the total number of points. Note: enabling it will make CVFH
	// invariant to scale just like VFH, but the authors encourage the opposite.
	cvfh.setNormalizeBins(CVFH_NORMALIZE_BINS);
 
	cvfh.compute(*descriptors);	

	#if DEBUG_MSG==1
		cout << "\tNumber of descriptors with CVFH: " << descriptors->size() << "\n";
	#endif
}

//Correspondences methods

void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false){
	//
	// Downsample for consistency and speed
	// \note enable this for large datasets
	PointCloud::Ptr src (new PointCloud);
	PointCloud::Ptr tgt (new PointCloud);
	pcl::VoxelGrid<PointType> grid;
	if (downsample){
		grid.setLeafSize (0.05, 0.05, 0.05);
		grid.setInputCloud (cloud_src);
		grid.filter (*src);

		grid.setInputCloud (cloud_tgt);
		grid.filter (*tgt);
	}else{
		src = cloud_src;
		tgt = cloud_tgt;
	}


	// Compute surface normals and curvature
	PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

	pcl::NormalEstimation<PointType, PointNormalT> norm_est;
	pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
	norm_est.setSearchMethod (tree);
	norm_est.setKSearch (ICP_NORMAL_SEARCH);

	norm_est.setInputCloud (src);
	norm_est.compute (*points_with_normals_src);
	pcl::copyPointCloud (*src, *points_with_normals_src);

	norm_est.setInputCloud (tgt);
	norm_est.compute (*points_with_normals_tgt);
	pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

	#if DEBUG_MSG==1
		cout << "\tEstimated normals on source: " << points_with_normals_src->size() << "\n";
		cout << "\tEstimated normals on target: " << points_with_normals_tgt->size()<< "\n";
	#endif

	//
	// Instantiate our custom point representation (defined above) ...
	MyPointRepresentation point_representation;
	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[4] = {1.0, 1.0, 1.0, 1.0};
	point_representation.setRescaleValues (alpha);

	//
	// Align
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
	reg.setTransformationEpsilon (ICP_TRANSFORMATION_EPSILON);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance (ICP_MAX_CORRESPONDENCE_DISTANCE);  
	// Set the point representation
	reg.setPointRepresentation (pcl::make_shared<const MyPointRepresentation> (point_representation));
	//Set fitness equation
	reg.setEuclideanFitnessEpsilon(ICP_EUCLIDEAN_FITNESS_EPSILON);

	reg.setInputSource (points_with_normals_src);
	reg.setInputTarget (points_with_normals_tgt);



	//
	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;

	reg.setMaximumIterations (2);
	
	for (int i = 0; i < ICP_MAX_ITERATIONS; ++i){
		PCL_INFO ("Iteration Nr. %d.\n", i);

		// save cloud for visualization purpose
		points_with_normals_src = reg_result;

		// Estimate
		reg.setInputSource (points_with_normals_src);
		reg.align (*reg_result);

			//accumulate transformation between each Iteration
		Ti = reg.getFinalTransformation () * Ti;

			//if the difference between this transformation and the previous one
			//is smaller than the threshold, refine the process by reducing
			//the maximal correspondence distance
		if (std::abs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
			reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

		prev = reg.getLastIncrementalTransformation ();

		// visualize current state
		#if DEBUG_VIS == 1
			showCloudsRight(points_with_normals_tgt, points_with_normals_src);
		#endif
	}

	//
	// Get the transformation from target to source
	targetToSource = Ti.inverse();

	//
	// Transform target back in source frame
	pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);
	#if DEBUG_VIS == 1
		p->removePointCloud ("source");
		p->removePointCloud ("target");

		PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
		PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
		p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
		p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

		PCL_INFO ("Press q to continue the registration.\n");
		p->spin ();

		p->removePointCloud ("source"); 
		p->removePointCloud ("target");
	#endif
	//add the source to the transformed target
	*output += *cloud_src;

	#if DEBUG_MSG==1
		cout << "\tICP matrix transformation: \n" << targetToSource << "\n";
		system("sleep 1");
	#endif

	final_transform = targetToSource;
 }

/**
 * @brief Alignment done between the actual (targeted) point cloud, and the previous (source) one using a SampleConsensusPrerejective algorithm
 * 
 * @param cloud 
 * @param descriptors 
 * @param last_cloud 
 * @param last_descriptors 
 * @param cloud_aligned 
 * @return true 
 * @return false 
 */
bool ransac_alignment(const pcl::PointCloud<PointType>::ConstPtr& cloud,
						const pcl::PointCloud<DescriptorType>::ConstPtr& descriptors,
						const pcl::PointCloud<PointType>::ConstPtr& last_cloud,
						const pcl::PointCloud<DescriptorType>::ConstPtr& last_descriptors,
						pcl::PointCloud<PointType>::Ptr cloud_aligned){
  pcl::SampleConsensusPrerejective<PointType,PointType,DescriptorType> align;
  align.setInputSource (cloud);
  align.setSourceFeatures (descriptors);
  align.setInputTarget (last_cloud);
  align.setTargetFeatures (last_descriptors);
  align.setMaximumIterations (50000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * 0.005); // Inlier threshold
  align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
  {
    align.align (*cloud_aligned);
  }
  return align.hasConverged();
}

/**
 * @brief Reduces the distances between the points of 2 point cloud and transforms them using ICP algorithm
 * 
 * @param cloud source point cloud
 * @param last_cloud target point cloud
 * @param transformation transformation matrix for alignment which will be modify to express the new reduce alignment
 */
void iterative_closest_point(const pcl::PointCloud<PointType>::ConstPtr& cloud,
						const pcl::PointCloud<PointType>::ConstPtr& last_cloud,/*
						const pcl::PointCloud<PointType>::ConstPtr& descriptor,
						const pcl::PointCloud<PointType>::ConstPtr& last_descriptor,*/
						Eigen::Matrix4f& transformation){
	pcl::IterativeClosestPoint<PointType, PointType> icp;
	icp.setInputSource(cloud);
	icp.setInputTarget(last_cloud);
	icp.setMaxCorrespondenceDistance(ICP_MAX_CORRESPONDENCE_DISTANCE);
	icp.setMaximumIterations(ICP_MAX_ITERATIONS);
	icp.setTransformationEpsilon(ICP_TRANSFORMATION_EPSILON);
	icp.setEuclideanFitnessEpsilon(ICP_EUCLIDEAN_FITNESS_EPSILON);
	pcl::PointCloud<PointType> aligned_cloud;
	icp.align(aligned_cloud, transformation);
	if(icp.hasConverged())
		transformation = icp.getFinalTransformation();

	#if DEBUG_MSG==1
		cout << "\tICP Score: " << icp.getFitnessScore() << "\n";
		cout << "\tICP matrix transformation: \n" << transformation << "\n";
	#endif
}

/**
 * @brief Obtains the correspondences and filters them to get the best correspondences using RANSAC
 * 
 * @param cloud source keypoints cloud
 * @param last_cloud target keypoint cloud
 * @param transformation transformation matrix obtained as a result
 * @param bestCorrespondences best corresponcedes obtained as a result
 */
void ransac_correspondences(const pcl::PointCloud<PointType>::ConstPtr &cloud,
								const pcl::PointCloud<PointType>::ConstPtr& last_cloud,
								Eigen::Matrix4f& transformation,
								pcl::CorrespondencesPtr& bestCorrespondences){
	// Estimate correspondences
	pcl::CorrespondencesPtr estimateCorrespondences (new pcl::Correspondences);
	pcl::registration::CorrespondenceEstimation<PointType, PointType> corr_est;
	corr_est.setInputSource(cloud);
	corr_est.setInputTarget(last_cloud);
	corr_est.determineCorrespondences(*estimateCorrespondences);

	// Apply RANSAC
	pcl::registration::CorrespondenceRejectorSampleConsensus<PointType> crsc;
    crsc.setInputSource(cloud);
    crsc.setInputTarget(last_cloud); 
    crsc.setInlierThreshold(RANSAC_INLIER_THRESHOLD); 
    crsc.setMaximumIterations(RANSAC_MAX_ITERATIONS); 
    crsc.setInputCorrespondences(estimateCorrespondences);
	crsc.getCorrespondences(*bestCorrespondences);
	transformation = crsc.getBestTransformation();
	
	#if DEBUG_MSG==1
		cout << "\tNumber of estimation correspondences: " << estimateCorrespondences->size() << "\n";
		cout << "\tNumber of remaining correspondences: " << bestCorrespondences->size() << "\n";
		cout << "\tMatrix transformation: \n" << transformation << "\n";
	#endif

}

/**
 * @brief Transformation using CorrespondecesRejectionSampleConsensus
 * 
 * @param cloud source keypoints cloud
 * @param last_cloud target keypoint cloud
 * @param transformedCloud cloud transformed
 */
void ransac_transform(const pcl::PointCloud<PointType>::ConstPtr &cloud,
						const pcl::PointCloud<PointType>::ConstPtr& last_cloud,
						Eigen::Matrix4f& transform_total,
						pcl::PointCloud<PointType>::Ptr &transformedCloud){
	Eigen::Matrix4f transform;
	pcl::CorrespondencesPtr bestCorrespondences(new pcl::Correspondences);

	// Estimate correspondences
	pcl::CorrespondencesPtr estimateCorrespondences (new pcl::Correspondences);
	pcl::registration::CorrespondenceEstimation<PointType, PointType> corr_est;
	corr_est.setInputSource(cloud);
	corr_est.setInputTarget(last_cloud);
	corr_est.determineCorrespondences(*estimateCorrespondences);

	// Apply RANSAC
	pcl::registration::CorrespondenceRejectorSampleConsensus<PointType>::Ptr crsc(new pcl::registration::CorrespondenceRejectorSampleConsensus<PointType>);
    crsc->setInputSource(cloud);
    crsc->setInputTarget(last_cloud); 
    crsc->setInlierThreshold(0.05); 
    crsc->setMaximumIterations(5000); 
    crsc->setInputCorrespondences(estimateCorrespondences);
	crsc->getCorrespondences(*bestCorrespondences);
    crsc->setInputCorrespondences(bestCorrespondences);
	transform = crsc->getBestTransformation();


	transform_cloud(transform,transform_total, cloud, transformedCloud);
	//pcl::transformPointCloud(*keypoints, *transformedCloud, transformTotal);

			
	#if DEBUG_MSG==1
		cout << "\tBest transform matrix: " << "\n" << transform << "\n";
		cout << "\tTotal transform matrix: " << "\n" << transform_total << "\n";
		cout << "\tSize of transformed cloud: " << transformedCloud->size() << "\n";
	#endif	
}


//Main algorithm

int main(int argc, char** argv){
	#if Method_Test == 0
		// Load data
		std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
		int totalSamples=150;//getTotalSamples(DIRECTORY);
		loadData(DIRECTORY, totalSamples, data);

		// Check user input
		if (data.empty ())
		{
			PCL_ERROR ("%s is empty or doesn't have pcd files", DIRECTORY);
			return (-1);
		}
		PCL_INFO ("Loaded %d datasets.\n", (int)data.size ());


		PointCloud::Ptr result (new PointCloud), source, target;
		Eigen::Matrix4f Tt = Eigen::Matrix4f::Identity ();

		for(int i = 1; i<totalSamples;i++){

			processBar(i,totalSamples);
			//Info last point cloud

			source = data[i-1].cloud;
			target = data[i].cloud;

			#if DEBUG_MSG==1
				cout<<"Previous point cloud processing: \n";
			#endif	
			pcl::PointCloud<pcl::Normal>::Ptr last_normals(new pcl::PointCloud<pcl::Normal>);
			estimate_normals(source, last_normals);
			//Info actual point cloud

			#if DEBUG_MSG==1
				cout<<"Actual point cloud processing: \n";
			#endif	
			pcl::PointCloud<pcl::Normal>::Ptr actual_normals(new pcl::PointCloud<pcl::Normal>);
			estimate_normals(target, actual_normals);
			
			/*
			Keypoints methods	| Descriptors methods
			1- SIFT				| 1- FPFH
			2- ISS				| 2- CVFH
			3- HARRIS			| 3- SHOT
			*/

			//Obtain keypoints

			pcl::PointCloud<PointType>::Ptr last_kp(new pcl::PointCloud<PointType>());
			pcl::PointCloud<PointType>::Ptr actual_kp(new pcl::PointCloud<PointType>());
			
			#if KeypointsMethod	== 1
				
				#if DEBUG_MSG==1
					cout<<"Last point cloud keypoints info: \n";
				#endif

				
				sift_keypoints(source, last_kp);

				#if DEBUG_MSG==1
					cout<<"Actual point cloud keypoints info: \n";
				#endif

				sift_keypoints(target, actual_kp);
			#elif KeypointsMethod == 2
				
				#if DEBUG_MSG==1
					cout<<"Last point cloud keypoints info: \n";
				#endif
				iss_keypoints(source, last_kp);
				#if DEBUG_MSG==1
					cout<<"ACtual point cloud keypoints info: \n";
				#endif
				iss_keypoints(target, actual_kp);
			#elif KeypointsMethod == 3
				
				#if DEBUG_MSG==1
					cout<<"Last point cloud keypoints info: \n";
				#endif
				
				harris_keypoints(source, last_kp);

				#if DEBUG_MSG==1
					cout<<"Actual point cloud keypoints info: \n";
				#endif

				
				harris_keypoints(target, actual_kp);
			#endif
				//complexVis(target, actual_kp, actual_normals);

			// Obtain descriptors

				pcl::PointCloud<DescriptorType>::Ptr last_dc(new pcl::PointCloud<DescriptorType>());
				pcl::PointCloud<DescriptorType>::Ptr actual_dc(new pcl::PointCloud<DescriptorType>());

			#if DescriptorsMethod == 1
				
				#if DEBUG_MSG==1
					cout<<"Last point cloud descriptors info: \n";
				#endif
				
				FPFH_descriptors(last_kp, last_dc);	

				#if DEBUG_MSG==1
					cout<<"Actual point cloud descriptors info: \n";
				#endif

				FPFH_descriptors(actual_kp, actual_dc);
			#elif DescriptorsMethod == 2
				
				#if DEBUG_MSG==1
					cout<<"Last point cloud descriptors info: \n";
				#endif

				CVFH_descriptors(last_kp, last_dc);	
				
				#if DEBUG_MSG==1
					cout<<"Actual point cloud descriptors info: \n";
				#endif

				CVFH_descriptors(actual_kp, actual_dc);	
			#elif DescriptorsMethod == 3
				
				#if DEBUG_MSG==1
					cout<<"Last point cloud descriptors info: \n";
				#endif

				SHOT352_descriptors(last_kp, source, last_dc);
				
				#if DEBUG_MSG==1
					cout<<"Actual point cloud descriptors info: \n";
				#endif

				SHOT352_descriptors(actual_kp, target, actual_dc);
			#endif

			// Matching process

			pcl::CorrespondencesPtr correspondences (new pcl::Correspondences ());

			// Obtain correspondences
			
			Eigen::Matrix4f Ti=Eigen::Matrix4f::Identity ();
			pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>());




			// Apply ICP
			#if RANSACMethod==1
				ransac_correspondences(target, source, Ti, correspondences);
				iterative_closest_point(target, source, Ti);
				transform_cloud(Ti, Tt, target, transformed_cloud);
			

			//RANSAC direct

			#elif RANSACMethod==2
				ransac_transform(target, source,Tt,transformed_cloud);

			//ransac alignment

			#elif RANSACMethod==3
				ransac_alignment(target, actual_dc, source, last_dc, transformed_cloud)
			#endif

			filter_voxel_grid(transformed_cloud, cloud_filtered);

			*result += *cloud_filtered;
		}
		
		std::stringstream ss;
		ss << "src/data/iterative_mapa/end_result.pcd";
		pcl::io::savePCDFile (ss.str (), *result, true);
		processBar(totalSamples,totalSamples);
	#elif Method_Test == 1
		// Load data
		std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
		int totalSamples=150;//getTotalSamples(DIRECTORY);
		loadData(DIRECTORY, totalSamples, data);

		// Check user input
		if (data.empty ())
		{
			PCL_ERROR ("%s is empty or doesn't have pcd files", DIRECTORY);
			return (-1);
		}
		PCL_INFO ("Loaded %d datasets.\n", (int)data.size ());
		#if DEBUG_VIS == 1
			// Create a PCLVisualizer object
			/**
			 * @todo alternativa a argv, array con los nombres de los ficheros
			 */
			p = new pcl::visualization::PCLVisualizer (totalSamples, argv, "Pairwise Incremental Registration example");
			p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
			p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
		#endif

			PointCloud::Ptr result (new PointCloud), source, target;
		Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
		
		for (std::size_t i = 1; i < data.size (); ++i)
		{
			processBar(i,totalSamples);
			source = data[i-1].cloud;
			target = data[i].cloud;
			
			#if DEBUG_VIS == 1
				// Add visualization data
				showCloudsLeft(source, target);
			#endif

			PointCloud::Ptr temp (new PointCloud);
			PCL_INFO ("Aligning %s (%zu) with %s (%zu).\n", data[i-1].f_name.c_str (), static_cast<std::size_t>(source->size ()), data[i].f_name.c_str (), static_cast<std::size_t>(target->size ()));
			pairAlign (source, target, temp, pairTransform, false);

			//transform current pair into the global transform
			pcl::transformPointCloud (*temp, *result, GlobalTransform);

			//update the global transform
			GlobalTransform *= pairTransform;

			//save aligned pair, transformed into the first cloud's frame
			std::stringstream ss;
			ss << "src/data/iterative_mapa/" <<i << ".pcd";
			pcl::io::savePCDFile (ss.str (), *result, true);

		}
		std::stringstream ss;
		ss << "src/data/iterative_mapa/end_result.pcd";
		pcl::io::savePCDFile (ss.str (), *result, true);
		processBar(totalSamples,totalSamples);
	#endif
}
