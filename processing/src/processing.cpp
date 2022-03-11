#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> 
#include <ctime> 
#include <std_msgs/String.h>

class Procesing {
	protected:
	//The node handle we'll be using
	ros::NodeHandle nh_;
	ros::Publisher chatter_pub;
	ros::Subscriber laserSub; 
	int count;


	public:
	Procesing(ros::NodeHandle& nh) { 
		// Suscribe el método commandCallback al tópico base_scan (el láser proporcionado por Stage)
		// El método commandCallback será llamado cada vez que el emisor (stage) publique datos 
		laserSub = nh.subscribe("scan", 1, &Procesing::commandCallback, this);

		chatter_pub = nh.advertise<std_msgs::String>("info",1000);
		nh_ = nh;
		count=0;
	};

	// Procesa los datos de láser
	void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
		ROS_INFO_STREAM("AngleMin: " << msg->angle_min); // Mínimo valor angular del láser
		ROS_INFO_STREAM("AngleMax: " << msg->angle_max); // Máximo valor angular del láser
		ROS_INFO_STREAM("AngleIncrement: " << msg->angle_increment); // Incremento angular entre dos beams
		ROS_INFO_STREAM("RangeMin: " << msg->range_min); // Mínimo valor que devuelve el láser
		ROS_INFO_STREAM("RangeMax: " << msg->range_max); // Máximo valor que devuelve el láser. Valores por debajo y por encima de estos rangos no deben ser tenidos en cuenta.
		int totalValues = ceil((msg->angle_max-msg->angle_min)/msg->angle_increment); // Total de valores que devuelve el láser
		for (int i=0; i< totalValues; i++) {
			ROS_INFO_STREAM("Values[" << i << "]:" << msg->ranges[i]); // Acceso a los valores de rango
		}

		std_msgs::String msgInfo;

	    std::stringstream ss;
        ss << msg->ranges[45]<< ":" << msg->ranges[0]<< ":"<< msg->ranges[315];
  	    msgInfo.data = ss.str();

		chatter_pub.publish(msgInfo);

		//ROS_INFO("%s", msgInfo.data.c_str());

	};

	// Bucle principal
	void bucle() {
		ros::Rate rate(1); // Especifica el tiempo de bucle en Hertzios. Ahora está en ciclo por segundo, pero normalmente usaremos un valor de 10 (un ciclo cada 100ms).
		while (ros::ok()) { // Bucle que estaremos ejecutando hasta que paremos este nodo o el roscore pare.	
			ros::spinOnce(); // Se procesarán todas las llamadas pendientes, es decir, llamará a callBack
			rate.sleep(); // Espera a que finalice el ciclo
		}
	};
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "Processing"); // Inicializa un nuevo nodo llamado Procesing
	ros::NodeHandle nh;
	Procesing sf(nh); // Crea un objeto de esta clase y lo asocia con roscore
	sf.bucle(); // Ejecuta el bucle
	return 0;
};