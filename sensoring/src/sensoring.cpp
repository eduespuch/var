#include <iostream>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/search/kdtree.h>

using namespace std;



//Definicion de atributos globales
pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_keypoints (new pcl::PointCloud<pcl::PointXYZRGB>);

//Viscualizacion a tiempo real de la extraccion de puntos
void simpleVis ()
{
  	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	
    while(!viewer.wasStopped())
	{
		*visu_keypoints+=*visu_pc;
	  viewer.showCloud(visu_keypoints);
	  //boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	  //viewer.showCloud (visu_pc);
	  boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
    
}

//Estructura  general del algoritmo
	//Extraccion de caracteristicas Ci de los datos xi (iteracion previamente almacenada)

	//Extraccion de caracteristicas Ci+1 de los datos xi+1(iteracion actual obtenida por evento)

	//Obtener emparejamiento entre caracteristicas, par de caracteristicas {c_a, c_b}

	//RANSAC para obtener la mejor transformacion Ti que explique los emparejamientos

	//Obtener la transformacion total

	//Aplicar Ci+1 a la transofrmacion total y colocarlo en C1, acumular los datos transformados en M


//metodos de uso general
void SIFT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints){

	const float min_scale = stof("0.01");             // Establece la desviación estándar de la escala más pequeña en el espacio de la escala          
	const int n_octaves = stof("6");               // Establecer el número de grupos de pirámides gaussianas (octava)            
	const int n_scales_per_octave = stof("4");     // Establecer la escala del cálculo de cada grupo (octava)  
	const float min_contrast = stof("0.01");          // Establecer umbral para limitar la detección de puntos clave       

	pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;// Crear objeto de detección de punto clave de tamizado
	pcl::PointCloud<pcl::PointWithScale> result;
	sift.setInputCloud(cloud);// Establecer la nube de puntos de entrada
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	sift.setSearchMethod(tree);// Cree un árbol de objetos de árbol kd vacío y páselo al objeto de detección de tamizado
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);// Especifique el rango de escala del punto clave de búsqueda
	sift.setMinimumContrast(min_contrast);// Establecer umbral para limitar la detección de puntos clave
	sift.compute(result);// Realice la detección de puntos clave de tamizado y guarde el resultado como resultado

	copyPointCloud(result, *keypoints);// Convierta los datos del tipo de punto pcl :: PointWithScale a los datos del tipo de punto pcl :: PointXYZ
	
}

double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud){

    double resolution = 0.0;
    int numberOfPoints = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> squaredDistances(2);
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        if (!isfinite((*cloud)[i].x))
            continue;

        // Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
        if (nres == 2)
        {
            resolution += sqrt(squaredDistances[1]);
            ++numberOfPoints;
        }
    }
    if (numberOfPoints != 0)
        resolution /= numberOfPoints;

    return resolution;
}

void compute_iss(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints){
    
    // calcular los puntos clave
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_det;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    // calcular la resolución
    double model_resolution = computeCloudResolution(cloud);
    
    // Ajuste de parámetros públicos de ISS
    iss_det.setMinNeighbors(10);
    iss_det.setThreshold21(0.975);
    iss_det.setThreshold32(0.975);
    iss_det.setNumberOfThreads(4);

    iss_det.setInputCloud(cloud);
    iss_det.setSearchMethod(tree);
    iss_det.setSalientRadius(6*model_resolution);  // 0.5
    iss_det.setNonMaxRadius(4*model_resolution);
    iss_det.compute(*keypoints);

}

	//extraccion de descriptores
void FPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors){

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//Dataset de normales respecto a la nube de puntos.

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;//Esto se usa para estimar las normales.
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);

	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;//Esto se usa para calcular el descriptor de histograma.


	normalEstimation.setInputCloud(keypoints);//Prepara la nube de puntos capturados para extraer sus rectas normales.
	fpfh.setInputCloud(keypoints);//Prepara la nube de puntos para obtener sus descriptores, en base a la propia nube de puntos y al dataset de normales.
	
	normalEstimation.setSearchMethod(kdtree);//La búsqueda la realizará fijándose en los n vecinos que encuentre en una esfera de radio r.
	normalEstimation.setRadiusSearch(0.1);//Da valor al radio r.
	normalEstimation.compute(*normals);//Calcula las normales de la nube de puntos.

	fpfh.setInputNormals(normals);//Una vez calculadas las normales, las podemos usar para obtener el histograma.
  
	fpfh.setSearchMethod(kdtree);
  	fpfh.setRadiusSearch(0.1);//Establece el radio de búsqueda (en metros) para los vecinos,el radio usado aquí debe ser mayor que el usado en el cálculo de las normales.
	fpfh.compute(*descriptors);

	cout<<"Se han calculado: "<<descriptors->points.size()<<" descriptores."<<endl;

}


void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
{
	
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

	
	cout << "Puntos capturados: " << cloud->size() << endl;

	pcl::VoxelGrid<pcl::PointXYZRGB > vGrid;
	vGrid.setInputCloud (cloud);
	vGrid.setLeafSize (0.05f, 0.05f, 0.05f);
	vGrid.filter (*cloud_filtered);

	cout << "Puntos tras VG: " << cloud_filtered->size() << endl;

	SIFT(cloud_filtered,visu_keypoints);
	cout<<"extraidos: "<<visu_keypoints->size()<<" keypoints."<<endl;

	visu_pc = cloud_filtered;
	
	if(prev_pc->size()==0) prev_pc=cloud_filtered;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_descriptors (new pcl::PointCloud<pcl::FPFHSignature33> ());//Dataset de descriptores respecto a la nube de puntos.
	FPFH(prev_pc, source_descriptors);//Método 1 de cálculo de los descriptores.

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr tgt_descriptors (new pcl::PointCloud<pcl::FPFHSignature33> ());//Dataset de descriptores respecto a la nube de puntos.
	FPFH(cloud_filtered, tgt_descriptors);//Método 1 de cálculo de los descriptores.

	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
	est.setInputSource(source_descriptors);
	est.setInputTarget(tgt_descriptors);
	est.determineCorrespondences(*correspondences);

	prev_pc = cloud_filtered;
	

	//-------------------------------RANSAC--------------------------------------------------
	if(correspondences->size()==prev_pc->size()){

		cout<< "Correspondencias antes de RANSAC: "<<correspondences->size()<<endl;

		pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences());
		pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> correspondence_rejector;

		correspondence_rejector.setInputSource(prev_pc);//emplear keypoints como PC
		correspondence_rejector.setInputTarget(cloud_filtered);

		correspondence_rejector.setInlierThreshold(0.2);
		correspondence_rejector.setMaximumIterations(1000);
		correspondence_rejector.setRefineModel(true);//false
		correspondence_rejector.setInputCorrespondences(correspondences);

		correspondence_rejector.getCorrespondences(*correspondences_filtered);
		cout<<"Correspondencias despues de RANSAC: "<<correspondences_filtered->size()<<endl;
		Eigen::Matrix4f TPuto=correspondence_rejector.getBestTransformation();
		cout<< TPuto<<endl;

	}{
		cout<< "------------------------------Alerta: no puedo calcular RANSAC, no me pegues por favor...--------------"<<endl;
	}
	//-------------------------------------------------------------------------------------------------------

	
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth/points", 1, callback);

  boost::thread t(simpleVis);

  while(ros::ok())
  {
	ros::spinOnce();
  }

}


