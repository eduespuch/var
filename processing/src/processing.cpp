#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/io.h> // for concatenateFields
#include <string>

using namespace std;


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

int main(int argc, char** argv){

    //Inicializacion de variables (source y target point clouds, matriz tranformacion global, nube de puntos global)
    /*
    no se como pasar argumentos ya que no se donde se genera el ejecutable, habra que ajustarlo en cada prueba manualmente
    */


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr total_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
    //ls -a directorio >> fichero con todos los arcivhos del directorio dado
    //ls -a directorio | wc -l cantidad de ficheros en el directorio


    //abrir bucle, tantas iteraciones como muestras haya
    for(int i=1;i<=81;i++){
        if(pcl::io::loadPCDFile<pcl::PointXYZRGB>("src/data/test_09_05/"+to_string(i-1)+".pcd",*source_pc)==-1){
            return -1;
        }
        cout<<"Cargando source de src/data/test_09_05/"<<to_string(i-1)<<".pcd con "<<source_pc->size()<<" puntos"<<endl;
        //simpleVis(source_pc); //Si se lanza este metodo, para que cargue la siguiente nube de puntos se debe de cerrar el visualizador

        if(pcl::io::loadPCDFile<pcl::PointXYZRGB>("src/data/test_09_05/"+to_string(i)+".pcd",*target_pc)==-1){
            return -1;
        }
        cout<<"Cargando target de src/data/test_09_05/"<<to_string(i)<<".pcd con "<<target_pc->size()<<" puntos"<<endl;

        //extraer Ci y Ci+1 en iteracion i

	//calcular keypoints de ambas muestras
		/*
		los keypoints son de tipo pc, usar 2 metodos diferenciados para extraccion de keypoints (SIFT y ISS)
		*/

	//obtener descriptores de ambas muestras, a raiz de los keypoints
		/*
		los descriptores son signature especificas al metodos(FPFHSignature y PFHSignature)
		*/
	//obtener correspondencias con los descripotres
		/*
		Un tipo correspondencia que se obtiene con un metodo con dos alternativas de argumentos,
		cada alternativa en funcion al metodo de descriptores seleccionado
		*/
	//rechazar las malas correspondencias, utilizar los keypoints

	//obtener la mejor transformacion

		//calcular el fitness de la transformacion por aqui

	//acumular la transformacion actual a la total

	//concatenar el Ci+1 modificada por la transfomracion actual al mapa (nube de puntos global)
	*total_pc=*source_pc;
    *total_pc += *target_pc;
        cout<<"Concatenacion de ambas nubes con "<<total_pc->size()<<" puntos en total"<<endl;
	simpleVis(total_pc);
    }
}