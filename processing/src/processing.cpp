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
	#include<pcl/visualization/pcl_plotter.h>

	//Others

	#include <boost/thread/thread.hpp>
	#include <boost/foreach.hpp>
//Parameters definition
	//Method selection
		/*
		Keypoints methods	| Descriptors methods
		1- SIFT				| 1- FPFH
		2- ISS				| 2- SHOT
		*/
		#define KeypointsMethod 1
		#define DescriptorsMethod 2

		//Selected descriptor type must me same type of descriptor method
		#if DescriptorsMethod==1// 1 == FPFH
			#define DescriptorType pcl::FPFHSignature33 
			#define DESCRIPTOR_SIZE 33
		#elif DescriptorsMethod==2// 2 == SHOT352
			#define DescriptorType pcl::SHOT352 
			#define DESCRIPTOR_SIZE 352
		#endif
	//Keypoints configuration Fase 2 o mayor

		//SIFT PARAMETERS
		#define SIFT_MIN_SCALE 0.25f
		#define SIFT_N_OCTAVES 7
		#define SIFT_N_SCALES_OCTAVE 8
		#define SIFT_MINIMUM_CONTRAST 0.1f

		// ISS PARAMETERS
		#define ISS_SALIENT_RADIUS 6
		#define ISS_NON_MAX_RADIUS 4
		#define ISS_MIN_NEIGHBORS 5
		#define ISS_21_THRESHOLD 0.975
		#define ISS_32_THRESHOLD 0.975


	//Descriptors configuration Fase 1 o mayor

		// FPFH PARAMETERS
		#define FPFH_RADIUS_SEARCH 0.5

		// SHOT352 PARAMETERS
		#define SHOT352_RADIUS_SEARCH 0.5

	//Correspondeces configuration Fase 1 o mayor

		#define RANSAC_MAX_ITERATIONS 10000
		#define RANSAC_INLIER_THRESHOLD 0.002

		#define ICP_NORMAL_SEARCH 30
		#define ICP_MAX_ITERATIONS 40
		#define ICP_MAX_CORRESPONDENCE_DISTANCE 0.1
		#define ICP_TRANSFORMATION_EPSILON 1e-6
		#define ICP_EUCLIDEAN_FITNESS_EPSILON 1

	//Others Fase 0 o mayor
		#define NORMALS_K_NEIGHBORS 30
		#define NORMALS_RADIUS_SEARCH 0.5f
		

		#define DIRECTORY "src/data/dir"

		#define PREFIX ""

		/*
		DEBUG_MSG   | DEBUG_VIS
		0. None	    | 0. None
		1. Basic    | 1. Simple
		2. Complete | 2. Iterative 
		*/

		#define DEBUG_MSG 2

		#define DEBUG_VIS 2

		/*
		PreFilter | Fase 
		0. None	  | 0. Print all the initial data
		1. VG 	  |	1. DS-DC-RANSAC
				  | 2. KP-DC-RANSAC
				  | 3. KP-DC-RANSAC-ICP
		*/

		#define LEAF_SIZE 0.05f
		#define PreFilter 0
		//EL voxel grid esta dando problemas

		#define Fase 2

		using namespace std;

		using pcl::visualization::PointCloudColorHandlerGenericField;
		using pcl::visualization::PointCloudColorHandlerCustom;


//Debugger visualizer

	//our visualizeres
	pcl::visualization::PCLVisualizer *p;
	//its left and right viewports
	int vp_1, vp_2;

//definition of auxiliar types

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
	

//Visualization methods

/**
 * @brief Cleans the console and shows a process bar of the actual progress
 * 
 * @param iter actual iteration
 * @param totalIter total iteration
 */
void processBar(int iter, int totalIter){
	//system("sleep 3");
	#if DEGUB_MSG ==0
		system("clear");
	#endif
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

}

/**
 * @brief  Display source and target on the second viewport of the visualizer
 * 
 * @param cloud_target targeted point cloud
 * @param cloud_source  source point cloud
 */
void showCloudsRight(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source){
  p->removePointCloud ("vp2_target");
  p->removePointCloud ("vp2_source");

  PointCloudColorHandlerCustom<PointType> tgt_h (cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<PointType> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_target, tgt_h, "vp2_target", vp_2);
  p->addPointCloud (cloud_source, src_h, "vp2_source", vp_2);

}

/**
 * @brief Draws the point cloud with keypoints and normals marked
 * 
 * @param cloud point cloud to be drawn
 * @param keypoints point cloud of keypoints
 * @param normals point cloud of normal, expected from the cloud
 */
void keypointsVis(const pcl::PointCloud<PointType>::ConstPtr& cloud,
					const pcl::PointCloud<PointType>::ConstPtr& keypoints,
					const pcl::PointCloud<pcl::Normal>::ConstPtr& normals){
   pcl::visualization::PCLVisualizer viewer("Keypoint visualizer");
   viewer.setBackgroundColor(0, 0, 0);
   viewer.addPointCloud(cloud, "cloud");       
   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2, "cloud");   
   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1, 1, 1, "cloud");
   viewer.addPointCloud(keypoints, "keypoints");      
   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,8, "keypoints");
   viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.0, 0.0, 1, "keypoints");
   //muestra una normal de 10 con una distancia de 5cm
   viewer.addPointCloudNormals<PointType, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
       
    viewer.spinOnce();
   /*viewer.spinOnce();
   boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
   */viewer.removePointCloud("cloud");
   viewer.removePointCloud("keypoints");
   viewer.removePointCloud("normals");
}

/**
 * @brief Draws a simple configuration point cloud
 * 
 * @param cloud point cloud to be drawn
 */
void simpleVis(pcl::PointCloud<PointType>::Ptr cloud){
	
  	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	 while(!viewer.wasStopped())
	{
		
		
	  viewer.showCloud(cloud);
	  //boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	  //viewer.showCloud (target_pc);
	  boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
	}
    
}

//Auxiliar methods


/**
 * @brief aplies downsampling using preestablished parameters (LEAF_SIZE)
 * 
 * @param cloud given point cloud
 * @param cloud_filtered point cloud with the filtered result
 */
void filter_voxel_grid(const pcl::PointCloud<PointType>::ConstPtr& cloud,
						pcl::PointCloud<PointType>::Ptr cloud_filtered){
	pcl::VoxelGrid<PointType> v_grid;
	#if DEBUG_MSG==2
		cout << "\tBefore VoxelGrid: " << cloud->size() << "\n";
	#endif
	v_grid.setInputCloud(cloud);
	v_grid.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
	v_grid.filter(*cloud_filtered);

	#if DEBUG_MSG==2
		cout << "\tAfter VoxelGrid: " << cloud_filtered->size() << "\n";
	#endif

}

/**
 * @brief Load a set of PCD files that we want to register together.
 * \attention Expected to use a directory with only pcd files. No doing this may result on an unexpected error.
 * 
 * @param dir directory where the pcd files are located
 * @param totalSamples total of files to read
 * @param models the resultant vector of point cloud datasets
 */
void loadData(const string& dir, const int totalSamples,
				vector<PointCloud::Ptr, Eigen::aligned_allocator<PointCloud::Ptr> > &models){
  // Suppose the first argument is the actual test model
  for (int i = 0; i < totalSamples; i++){

  	PointCloud::Ptr cloud (new PointCloud);
      // Load the cloud and saves it into the global list of models
	string file= dir+"/"+PREFIX+to_string(i)+".pcd";
	if(pcl::io::loadPCDFile<PointType>(file,*cloud)==-1){
		PCL_ERROR("Fail to read %s",file);
		exit(-1);
	}
	//remove NAN points from the cloud
	vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);

	#if PreFilter == 1
  		PointCloud::Ptr temp (new PointCloud);
		temp=cloud;
		filter_voxel_grid(temp, cloud);
	#endif


	models.push_back (cloud);
  }
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

	#if DEBUG_MSG==2
		cout << "\tTransformTotal matrix: \n" << transform_total << "\n";
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
	//ne.setKSearch(NORMALS_K_NEIGHBORS);

	ne.compute(*normals);

	
	#if DEBUG_MSG==2
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

//Keypoints extraction

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
 * @brief Obtains the keypoints of cloud point 
 * 
 * @param cloud inpunt point cloud
 * @param normals normals of the input cloud, just for vis purposes
 * @param keypoints output point cloud, referencing keypoints
 */					
void extract_keypoints(const pcl::PointCloud<PointType>::ConstPtr& cloud,
					const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
					pcl::PointCloud<PointType>::Ptr& keypoints){

	#if KeypointsMethod == 1 //SHIFT
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
		
		#if DEBUG_MSG==2
			cout << "\tNumber of keypoints with SIFT detector: " << keypoints->size() << "\n";
		#endif
	#elif KeypointsMethod == 2 //ISS
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
		#if DEBUG_MSG==2
			cout << "\tNumber of keypoints with ISS detector: " << keypoints->size() << "\n";
		#endif
	#endif
	#if DEBUG_VIS == 2
		cout<<"\tShowing Keypoints extracted:\n";
		keypointsVis(cloud, keypoints, normals);
	#endif
}

//Descriptors obtention

/**
 * @brief Compute the descriptors 
 * 
 * @param keypoints input Keypoints
 * @param descriptors output point cloud referencing the descriptors
 */

/**
 * @brief Compute the descriptors 
 * 
 * @param cloud input
 * @param normals normals of the input
 * @param descriptors output 
 */
void compute_descriptors(const pcl::PointCloud<PointType>::ConstPtr& cloud,
						const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
						pcl::PointCloud<DescriptorType>::Ptr& descriptors){
	#if DescriptorsMethod == 1 //FPFH
		pcl::FPFHEstimation<PointType, pcl::Normal, DescriptorType> fpfh;
		fpfh.setInputCloud(cloud);
		fpfh.setInputNormals(normals);
		pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>());
		fpfh.setSearchMethod(tree);
		// Radio de busqueda, tiene que ser mayor que el utilizado al calcular las normales
		fpfh.setRadiusSearch(FPFH_RADIUS_SEARCH);
		fpfh.compute(*descriptors);

		#if DEBUG_MSG==2
			cout << "\tNumber of descriptors with FPFH: " << descriptors->size() << "\n";
		#endif
	#elif DescriptorsMethod == 2 //SHOT
		pcl::SHOTEstimation<PointType, pcl::Normal, DescriptorType> shot;
		shot.setRadiusSearch(SHOT352_RADIUS_SEARCH);
		shot.setInputCloud(cloud);
		shot.setInputNormals(normals);
		shot.compute(*descriptors);
		#if DEBUG_MSG==2
			cout << "\tNumber of descriptors with SHOT: " << descriptors->size() << "\n";
		#endif
	#endif
}

/**
 * @brief Compute the descriptors 
 * 
 * @param keypoints input Keypoints
 * @param descriptors output point cloud referencing the descriptors
 */
void compute_descriptors(const pcl::PointCloud<PointType>::ConstPtr& keypoints,
							const pcl::PointCloud<PointType>::ConstPtr& cloud,
							const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
						pcl::PointCloud<DescriptorType>::Ptr& descriptors){
	
	#if DescriptorsMethod == 1
		pcl::FPFHEstimation<PointType, pcl::Normal, DescriptorType> fpfh;
		fpfh.setInputCloud(keypoints);
		fpfh.setInputNormals(normals);
		fpfh.setSearchSurface(cloud);
		pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>());
		fpfh.setSearchMethod(tree);
		// Radio de busqueda, tiene que ser mayor que el utilizado al calcular las normales
		fpfh.setRadiusSearch(FPFH_RADIUS_SEARCH);
		fpfh.compute(*descriptors);

		#if DEBUG_MSG==2
			cout << "\tNumber of descriptors with FPFH: " << descriptors->size() << "\n";
		#endif
	#elif DescriptorsMethod == 2
		pcl::SHOTEstimation<PointType, pcl::Normal, DescriptorType> shot;
		shot.setRadiusSearch(SHOT352_RADIUS_SEARCH);
		shot.setInputCloud(keypoints);
		shot.setInputNormals(normals);
		shot.setSearchSurface(cloud);
		shot.compute(*descriptors);
		#if DEBUG_MSG==2
			cout << "\tNumber of descriptors with SHOT: " << descriptors->size() << "\n";
		#endif
	#endif
}

//Correspondeces computing
/**
 * @brief Obtains the correspondences and filters them to get the best correspondences using RANSAC
 * 
 * @param cloud source keypoints cloud
 * @param last_cloud target keypoint cloud
 * @param transformation transformation matrix obtained as a result
 * @param bestCorrespondences best corresponcedes obtained as a result
 */
void ransac_correspondences(const pcl::PointCloud<PointType>::ConstPtr& source,
								const pcl::PointCloud<PointType>::ConstPtr &target,
								const pcl::PointCloud<DescriptorType>::ConstPtr& source_dc,
								const pcl::PointCloud<DescriptorType>::ConstPtr& target_dc,
								Eigen::Matrix4f& transformation,
								pcl::CorrespondencesPtr& bestCorrespondences){
	// Estimate correspondences
	pcl::CorrespondencesPtr estimateCorrespondences (new pcl::Correspondences);

	pcl::registration::CorrespondenceEstimation<DescriptorType, DescriptorType> est;
	est.setInputSource(source_dc);
	est.setInputTarget(target_dc);
	est.determineCorrespondences(*estimateCorrespondences);

	// Apply RANSAC
	pcl::registration::CorrespondenceRejectorSampleConsensus<PointType> crsc;
    crsc.setInputSource(source);
    crsc.setInputTarget(target); 
    crsc.setInlierThreshold(RANSAC_INLIER_THRESHOLD); 
    crsc.setMaximumIterations(RANSAC_MAX_ITERATIONS); 
    crsc.setInputCorrespondences(estimateCorrespondences);
	crsc.getCorrespondences(*bestCorrespondences);
	transformation = crsc.getBestTransformation();
	
	#if DEBUG_MSG==2
		cout << "\tNumber of estimation correspondences: " << estimateCorrespondences->size() << "\n";
		cout << "\tNumber of remaining correspondences: " << bestCorrespondences->size() << "\n";
		cout << "\tMatrix transformation: \n" << transformation << "\n";
	#endif

}

//Main algorithm

int main(int argc, char** argv){
	// Load data
	std::vector<PointCloud::Ptr, Eigen::aligned_allocator<PointCloud::Ptr> > data;
	int totalSamples=getTotalSamples(DIRECTORY);


	loadData(DIRECTORY, totalSamples, data);
	// Check user input
	if (data.empty ())
	{
		PCL_ERROR ("%s is empty or doesn't have pcd files", DIRECTORY);
		return (-1);
	}
	cout<<"Loaded "<< (int)data.size ()<<" datasets.\n"; 

	//Initialize common use resources
	PointCloud::Ptr result (new PointCloud), source(new PointCloud), target(new PointCloud);
	Eigen::Matrix4f Tt = Eigen::Matrix4f::Identity ();
	result=data[0];

	//Initialize debugger visualizers
	#if DEBUG_VIS == 1 //Simple visualizer

	#elif DEBUG_VIS == 2 //Complex visualizer
		// Create a PCLVisualizer object
		p = new pcl::visualization::PCLVisualizer ("Actual iteration and overall result");
		p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
		p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
	#endif

	//Starts the main loop
	for(int i = 1; i<totalSamples;i++){
		//Global info
		processBar(i,totalSamples);


		#if DEBUG_MSG==1 || DEBUG_MSG==2
			cout<<"Info of the last transform: \n"<<
				"Map info: "<<result->size()<< " total points.\n"<<
				"Global matrix: \n"<<Tt<<"\n";

		#endif	

		//Info last point cloud

		#if DEBUG_MSG==2
			cout<<"Previous point cloud processing: \n";
		#endif	


		source = data[i-1];

		pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
		estimate_normals(source, source_normals);

		//Info actual point cloud

		#if DEBUG_MSG==2
			cout<<"Actual point cloud processing: \n";
		#endif	
		
		target = data[i];

		pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
		estimate_normals(target, target_normals);
	/*
	Fase 
		0. Print all the initial data
		1. DS-DC-RANSAC
		2. KP-DC-RANSAC
		3. KP-DC-RANSAC-ICP
	*/
		#if Fase == 0
		// Add visualization data
			cout<<"Source info: "<<source->size()<<" total points, with "<<source_normals->size()<<" normals\n";
			cout<<"Target info: "<<target->size()<<" total points, with "<<target_normals->size()<<" normals\n";

			#if DEBUG_VIS == 2 //Complex visualizer
				showCloudsLeft(source, target);
				p->spin();
			#endif

		#elif Fase == 1
			#if DEBUG_VIS == 2 //Complex visualizer
				showCloudsLeft(source, target);
			#endif
		//Get descriptors
			pcl::PointCloud<DescriptorType>::Ptr source_dc(new pcl::PointCloud<DescriptorType>());
			pcl::PointCloud<DescriptorType>::Ptr target_dc(new pcl::PointCloud<DescriptorType>());
			#if DEBUG_MSG==2
				cout<<"Last point cloud descriptors info: \n";
			#endif
				
			compute_descriptors(source, source_normals, source_dc);	

			#if DEBUG_MSG==2
				cout<<"Actual point cloud descriptors info: \n";
			#endif

			compute_descriptors(target,target_normals, target_dc);

		//Get correspondeces
			pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
			// Obtain correspondences
			
			Eigen::Matrix4f Ti=Eigen::Matrix4f::Identity ();
			pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>());
			ransac_correspondences(source, target, source_dc, target_dc, Ti, correspondences);
			transform_cloud(Ti, Tt, target, transformed_cloud);
			
			#if DEBUG_VIS == 2
				showCloudsRight(result, transformed_cloud);		
				p->spin();
			#endif
			*result += *transformed_cloud;
		#elif Fase == 2 

			#if DEBUG_VIS == 2 //Complex visualizer
				showCloudsLeft(source, target);
			#endif
		//Get Keypoints

			pcl::PointCloud<PointType>::Ptr source_kp(new pcl::PointCloud<PointType>());
			pcl::PointCloud<PointType>::Ptr target_kp(new pcl::PointCloud<PointType>());

				
			#if DEBUG_MSG==2
				cout<<"Last point cloud keypoints info: \n";
			#endif

				
			extract_keypoints(source, source_normals, source_kp);

			#if DEBUG_MSG==2
				cout<<"Actual point cloud keypoints info: \n";
			#endif

			extract_keypoints(target, target_normals, target_kp);

		//Get descriptors
			pcl::PointCloud<DescriptorType>::Ptr source_dc(new pcl::PointCloud<DescriptorType>());
			pcl::PointCloud<DescriptorType>::Ptr target_dc(new pcl::PointCloud<DescriptorType>());
			#if DEBUG_MSG==2
				cout<<"Last point cloud descriptors info: \n";
			#endif
				
			compute_descriptors(source_kp, source, source_normals, source_dc);	

			#if DEBUG_MSG==2
				cout<<"Actual point cloud descriptors info: \n";
			#endif

			compute_descriptors(target_kp, target,target_normals, target_dc);

		//Get correspondeces
			pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
			// Obtain correspondences
			
			Eigen::Matrix4f Ti=Eigen::Matrix4f::Identity ();
			pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>());
			ransac_correspondences(source_kp, target_kp, source_dc, target_dc, Ti, correspondences);
			transform_cloud(Ti, Tt, target, transformed_cloud);
			
			#if DEBUG_VIS == 2
				showCloudsRight(result, transformed_cloud);		
			#endif
			*result += *transformed_cloud;

		#elif Fase == 3
			pcl::PointCloud<DescriptorType>::Ptr source_dc(new pcl::PointCloud<DescriptorType>());
			pcl::PointCloud<DescriptorType>::Ptr target_dc(new pcl::PointCloud<DescriptorType>());

		#endif
		
	}
		
	std::stringstream ss;
	ss << "src/data/iterative_mapa/end_result.pcd";
	pcl::io::savePCDFileASCII (ss.str (), *result);
	processBar(totalSamples,totalSamples);
	cout<<"Map completed! \nShowing result now.\n";
	simpleVis(result);
}
