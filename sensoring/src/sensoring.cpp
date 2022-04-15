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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_pc (new pcl::PointCloud<pcl::PointXYZRGB>);


void simpleVis ()
{
  	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	
    while(!viewer.wasStopped())
	{
	  viewer.showCloud (visu_pc);
	  boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
    
}

void FPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//Dataset de normales respecto a la nube de puntos.
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors (new pcl::PointCloud<pcl::FPFHSignature33> ());//Dataset de descriptores respecto a la nube de puntos.

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;//Esto se usa para estimar las normales.
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);

	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;//Esto se usa para calcular el descriptor de histograma.


	normalEstimation.setInputCloud(cloud);//Prepara la nube de puntos capturados para extraer sus rectas normales.
	fpfh.setInputCloud(cloud);//Prepara la nube de puntos para obtener sus descriptores, en base a la propia nube de puntos y al dataset de normales.
	
	normalEstimation.setSearchMethod(kdtree);//La búsqueda la realizará fijándose en los n vecinos que encuentre en una esfera de radio r.
	normalEstimation.setRadiusSearch(0.03);//Da valor al radio r.
	normalEstimation.compute(*normals);//Calcula las normales de la nube de puntos.

	fpfh.setInputNormals(normals);//Una vez calculadas las normales, las podemos usar para obtener el histograma.
  
	fpfh.setSearchMethod(kdtree);
  	fpfh.setRadiusSearch(0.05);//Establece el radio de búsqueda (en metros) para los vecinos,el radio usado aquí debe ser mayor que el usado en el cálculo de las normales.
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

	visu_pc = cloud_filtered;

	FPFH(cloud_filtered);//Método 1 de cálculo de los descriptores.

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


