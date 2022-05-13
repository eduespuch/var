#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/io.h> // for concatenateFields
#include <string>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transforms.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/search/kdtree.h>

using namespace std;

const float KeyPointRadius=0.0;
const float DescriptorRadius=0.1;
const float NormalRadius=0.1;
const float InlierTreshold=0.15;
const int RansacIter=3000;


//Viscualizacion a tiempo real de la extraccion de puntos
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

//Metodos de obtencion de keypoints
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

pcl::PointCloud<pcl::Normal>::Ptr computeNormals(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud){

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;//Esto se usa para estimar las normales.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//Dataset de normales respecto a la nube de puntos.
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);

	normalEstimation.setInputCloud(cloud);//Prepara la nube de puntos capturados para extraer sus rectas normales.
	normalEstimation.setSearchMethod(kdtree);//La búsqueda la realizará fijándose en los n vecinos que encuentre en una esfera de radio r.
	normalEstimation.setRadiusSearch(NormalRadius);//Da valor al radio r.
	normalEstimation.compute(*normals);//Calcula las normales de la nube de puntos.

	return normals;

}

double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud){

    double resolution = 0.0;
    int numberOfPoints = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> squaredDistances(2);
    pcl::search::KdTree<pcl::PointXYZRGB> tree;
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

void ISS(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints){
    
    // calcular los puntos clave
    pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_det;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());

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
void FPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors){

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//Dataset de normales respecto a la nube de puntos.
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);

	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;//Esto se usa para calcular el descriptor de histograma.
	fpfh.setInputCloud(keypoints);//Prepara la nube de puntos para obtener sus descriptores, en base a la propia nube de puntos y al dataset de normales.
	fpfh.setSearchSurface(cloud);
	fpfh.setInputNormals(computeNormals(cloud));//Una vez calculadas las normales, las podemos usar para obtener el histograma.
  
	fpfh.setSearchMethod(kdtree);
  	fpfh.setRadiusSearch(DescriptorRadius);//Establece el radio de búsqueda (en metros) para los vecinos,el radio usado aquí debe ser mayor que el usado en el cálculo de las normales.
	fpfh.compute(*descriptors);

}

void PFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints, pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors){
	
	// Create the PFH estimation class, and pass the input dataset+normals to it
	
	pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZRGB> ());

	pfh.setInputCloud (keypoints);
	pfh.setSearchSurface(cloud);
	pfh.setInputNormals (computeNormals(cloud));
	
	// Create an empty kdtree representation, and pass it to the PFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pfh.setSearchMethod (kdtree);
	 // Output datasets

	// Use all neighbors in a sphere of radius 10cm
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	pfh.setRadiusSearch (DescriptorRadius);

	// Compute the features
	pfh.compute (*descriptors);
}

//Algoritmo general
int main(int argc, char** argv){

    //Inicializacion de variables (source y target point clouds, matriz tranformacion global, nube de puntos global)
    /*
    no se como pasar argumentos ya que no se donde se genera el ejecutable, habra que ajustarlo en cada prueba manualmente
    */
	string dir="src/data/test_09_05";
	int totalSamples=getTotalSamples(dir);
	bool sift=true;
	bool fpfh=false;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr total_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
	Eigen::Matrix4f total_Matrix=Eigen::Matrix4f::Identity ();

	if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(dir+"/0.pcd",*total_pc)==-1){
		return -1;
	}
	cout<<"Cargando sistema referencia usando "<<dir<<"/0.pcd como eje, con "<<total_pc->size()<<" puntos"<<endl;

    //abrir bucle, tantas iteraciones como muestras haya
    for(int i=1;i<50/*totalSamples*/;i++){
        if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(dir+"/"+to_string(i-1)+".pcd",*source_pc)==-1){
            return -1;
        }
        cout<<"Cargando source de "<<dir<<"/"<<to_string(i-1)<<".pcd con "<<source_pc->size()<<" puntos"<<endl;
        //simpleVis(source_pc); //Si se lanza este metodo, para que cargue la siguiente nube de puntos se debe de cerrar el visualizador

        if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(dir+"/"+to_string(i)+".pcd",*target_pc)==-1){
            return -1;
        }
        cout<<"Cargando target de "<<dir<<"/"<<to_string(i)<<".pcd con "<<target_pc->size()<<" puntos"<<endl;

        //extraer Ci y Ci+1 en iteracion i

	//calcular keypoints de ambas muestras

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_kp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_kp (new pcl::PointCloud<pcl::PointXYZRGB>);
		/*
		los keypoints son de tipo pc, usar 2 metodos diferenciados para extraccion de keypoints (SIFT y ISS)
		*/
		if(sift){
			SIFT(source_pc,source_kp);
			SIFT(target_pc,target_kp);
		}else{//otro metodo que no sea sift
			ISS(source_pc,source_kp);
			ISS(target_pc,target_kp);
		}
		

	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());

	//obtener descriptores de ambas muestras, a raiz de los keypoints
	if(fpfh){
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_descriptors (new pcl::PointCloud<pcl::FPFHSignature33> ());//Dataset de descriptores respecto a la nube de puntos.
		FPFH(source_pc,source_kp, source_descriptors);//Método 1 de cálculo de los descriptores.
		cout<<"Se han calculado: "<<source_descriptors->points.size()<<" descriptores para el source."<<endl;
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr tgt_descriptors (new pcl::PointCloud<pcl::FPFHSignature33> ());//Dataset de descriptores respecto a la nube de puntos.
		FPFH(target_pc,target_kp, tgt_descriptors);//Método 1 de cálculo de los descriptores.
		
		cout<<"Se han calculado: "<<tgt_descriptors->points.size()<<" descriptores para el target."<<endl;
		pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
		est.setInputSource(source_descriptors);
		est.setInputTarget(tgt_descriptors);
		est.determineCorrespondences(*correspondences);
	}else{//si hace pfh
		pcl::PointCloud<pcl::PFHSignature125>::Ptr source_descriptors (new pcl::PointCloud<pcl::PFHSignature125> ());//Dataset de descriptores respecto a la nube de puntos.
		PFH(source_pc,source_kp, source_descriptors);//Método 1 de cálculo de los descriptores.
		cout<<"Se han calculado: "<<source_descriptors->points.size()<<" descriptores para el source."<<endl;
		pcl::PointCloud<pcl::PFHSignature125>::Ptr tgt_descriptors (new pcl::PointCloud<pcl::PFHSignature125> ());//Dataset de descriptores respecto a la nube de puntos.
		PFH(target_pc,target_kp, tgt_descriptors);//Método 1 de cálculo de los descriptores.
		cout<<"Se han calculado: "<<tgt_descriptors->points.size()<<" descriptores para el target."<<endl;
		
		pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> est;
		est.setInputSource(source_descriptors);
		est.setInputTarget(tgt_descriptors);
		est.determineCorrespondences(*correspondences);

	}
		/*
		los descriptores son signature especificas al metodos(FPFHSignature y PFHSignature)
		*/
	//obtener correspondencias con los descripotres
		/*
		Un tipo correspondencia que se obtiene con un metodo con dos alternativas de argumentos,
		cada alternativa en funcion al metodo de descriptores seleccionado
		*/
	//rechazar las malas correspondencias, utilizar los keypoints
	//-------------------------------RANSAC--------------------------------------------------


	cout<< "Correspondencias antes de RANSAC: "<<correspondences->size()<<endl;

	pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences());
	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> correspondence_rejector;

	correspondence_rejector.setInputSource(source_kp);//emplear keypoints como PC
	correspondence_rejector.setInputTarget(target_kp);

	correspondence_rejector.setInlierThreshold(InlierTreshold);
	correspondence_rejector.setMaximumIterations(RansacIter);
	correspondence_rejector.setRefineModel(true);//false
	correspondence_rejector.setInputCorrespondences(correspondences);

	correspondence_rejector.getCorrespondences(*correspondences_filtered);
	cout<<"Correspondencias despues de RANSAC: "<<correspondences_filtered->size()<<endl;
	Eigen::Matrix4f parcial_Matrix=correspondence_rejector.getBestTransformation();//obtenemos la mejor transformacion
	cout<< total_Matrix<<endl;
	//-------------------------------------------------------------------------------------------------------

	
		//calcular el fitness de la transformacion por aqui

	//acumular la transformacion actual a la total
	total_Matrix*=parcial_Matrix;
	pcl::transformPointCloud(*target_pc,*source_pc,total_Matrix);
	//concatenar el Ci+1 modificada por la transfomracion actual al mapa (nube de puntos global)
    *total_pc += *source_pc;
        cout<<"Concatenacion de ambas nubes con "<<total_pc->size()<<" puntos en total"<<endl;
    }
	
	pcl::io::savePCDFileASCII ("src/mapa.pcd", *total_pc);
	simpleVis(total_pc);
}