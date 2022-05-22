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

	typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

using namespace std;

        #define KeypointsMethod 1
		#define DescriptorsMethod 1

		//Selected descriptor type must me same type of descriptor method
		#if DescriptorsMethod==1// 1 == FPFH
			#define DescriptorType pcl::FPFHSignature33 
			#define DESCRIPTOR_SIZE 33
		#elif DescriptorsMethod==2// 2 == SHOT352
			#define DescriptorType pcl::SHOT352 
			#define DESCRIPTOR_SIZE 352
		#endif


        #define NORMALS_K_NEIGHBORS 30
		#define NORMALS_RADIUS_SEARCH 0.075f

	//Keypoints configuration Fase 2 o mayor

		//SIFT PARAMETERS
		#define SIFT_MIN_SCALE 0.1
		#define SIFT_N_OCTAVES 8
		#define SIFT_N_SCALES_OCTAVE 10
		#define SIFT_MINIMUM_CONTRAST 0.01

		// ISS PARAMETERS
		#define ISS_SALIENT_RADIUS 3
		#define ISS_NON_MAX_RADIUS 2
		#define ISS_MIN_NEIGHBORS 15
		#define ISS_21_THRESHOLD 3
		#define ISS_32_THRESHOLD 2


	//Descriptors configuration Fase 1 o mayor

		// FPFH PARAMETERS
		#define FPFH_RADIUS_SEARCH 0.125f

		// SHOT352 PARAMETERS
		#define SHOT352_RADIUS_SEARCH 0.125f

	//Correspondeces configuration Fase 1 o mayor

		#define RANSAC_MAX_ITERATIONS 100000
		#define RANSAC_INLIER_THRESHOLD 0.01

		#define ICP_NORMAL_SEARCH 30
		#define ICP_MAX_ITERATIONS 40
		#define ICP_MAX_CORRESPONDENCE_DISTANCE 0.1
		#define ICP_TRANSFORMATION_EPSILON 1e-6
		#define ICP_EUCLIDEAN_FITNESS_EPSILON 1

	//Others Fase 0 o mayor
		

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
		//EL voxel grid esta dando problemas

		 
    pcl::visualization::PCLVisualizer *kviewer;
    //its left and right viewports
    

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

void keypointsVis(const pcl::PointCloud<PointType>::ConstPtr& cloud,
					const pcl::PointCloud<PointType>::ConstPtr& keypoints,
					const pcl::PointCloud<pcl::Normal>::ConstPtr& normals){
   kviewer->removePointCloud("cloud");
   kviewer->removePointCloud("keypoints");
   kviewer->removePointCloud("normals");
   kviewer->addPointCloud(cloud, "cloud");       
   kviewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2, "cloud");   
   kviewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1, 1, 1, "cloud");
   kviewer->addPointCloud(keypoints, "keypoints");      
   kviewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,8, "keypoints");
   kviewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.0, 0.0, 1, "keypoints");
   //muestra una normal de 10 con una distancia de 5cm
   kviewer->addPointCloudNormals<PointType, pcl::Normal> (cloud, normals, 1, 0.1, "normals");
       
   kviewer->spin();
   /*kviewer->spinOnce();
   boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
   */kviewer->removePointCloud("cloud");
   kviewer->removePointCloud("keypoints");
   kviewer->removePointCloud("normals");
}

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

int main(){

	PointCloud::Ptr cloud (new PointCloud);
	 kviewer = new pcl::visualization::PCLVisualizer ("Keypoints and normals visualizer");
           kviewer->setBackgroundColor(0, 0, 0);

	int fichero;
	cout<<"Que fichero quieres procesar?"<<endl;
	cin>>fichero;

	string file="src/data/dir/"+to_string(fichero)+".pcd";

	if(pcl::io::loadPCDFile<PointType>(file,*cloud)==-1){
		PCL_ERROR("Fail to read %s",file);
		exit(-1);
	}
	
	vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);

	PointCloud::Ptr temp (new PointCloud);
	temp=cloud;
	filter_voxel_grid(temp, cloud);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointType>::Ptr keypoints(new pcl::PointCloud<PointType>());
	estimate_normals(cloud,normals);
	extract_keypoints(cloud,normals,keypoints);

    pcl::PointCloud<DescriptorType>::Ptr descr(new pcl::PointCloud<DescriptorType>());
				cout<<keypoints->size()<<endl;
    compute_descriptors(keypoints, cloud, normals, descr);	
	// Plotter object.
	pcl::visualization::PCLPlotter plotter("Histogram");
	// We need to set the size of the descriptor beforehand.
	plotter.addFeatureHistogram(*descr, DESCRIPTOR_SIZE);

	plotter.plot();

	return 0;

}
