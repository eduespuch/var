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
		
		std_msgs::String msgInfo;
	    std::stringstream ss;

		//Obtiene el angulo de interseccion a la pared de la izquierda

		int firstAngle=50;
		int secAngle=51;
		bool first=false;

		for(int i= 50;i<=100;i++){
			if(!first&&msg->ranges[i]<=3.5){
				first=true;
				firstAngle=i;
			}
			if(first&&msg->ranges[i]<=3.5){
				secAngle=i;
			}
		}

		float o=(sin(firstAngle)*msg->ranges[firstAngle]-sin(secAngle)*msg->ranges[secAngle]); 
		float a=(cos(firstAngle)*msg->ranges[firstAngle]-cos(secAngle)*msg->ranges[secAngle]);
		float d=(sqrt(pow(o,2)+pow(a,2)));
		float wallAngle=1/sin(o/d);//esto creo que no esta bien

		//Extraer los datos de interes

		float minDistance=msg->range_max;
		float frontUmbral=0.5;
		float lateralUmbral=0.3;
		float angleCurveUmbral=0.436332; //25º
		int relAngle=360;

		for(int i=0;i<=25;i++){
			if(msg->ranges[i]<minDistance){
				minDistance=msg->ranges[i];
				relAngle=i;
			}
		}
		for(int i=335;i<=358;i++){
			if(msg->ranges[i]<minDistance){
				minDistance=msg->ranges[i];
				relAngle=i;
			}
		}

		if(msg->ranges[50]>3.5){
			
		}

		if(minDistance<frontUmbral){//obstruccion delante, tiene que girar
			ss<< "3";
		}else if(wallAngle>angleCurveUmbral){
			ss<<"2";
		}else{
			ss<<"1";
		}

		ss<<":"<<msg->ranges[firstAngle]<<","<<msg->ranges[secAngle]<<":sqrt("<<o<<"²+"<<a<<"²)"<<"="<<d<<"<->"<<wallAngle<<":"<<minDistance;


  	    msgInfo.data = ss.str();

		chatter_pub.publish(msgInfo);

		ROS_INFO("%s", msgInfo.data.c_str());

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