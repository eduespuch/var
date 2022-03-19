#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> 
#include <ctime> 
#include <std_msgs/String.h>
#include <math.h>

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

		int firstAngle=50;//punto mas cercano al frente
		int secAngle=51;//ultimo punto posible
		bool first=false;

		for(int i= 50;i<=100;i++){
			if(!first&&msg->ranges[i]<=3.5){
				first=true;
				firstAngle=i;
				secAngle=i+1;
			}
			if(first&&msg->ranges[i]<=3.5){
				secAngle=i;
			}
		}

		double o=(sin((msg->range_max-secAngle)*M_PI/180)*msg->ranges[secAngle]-sin((msg->range_max-firstAngle)*M_PI/180)*msg->ranges[firstAngle]); 
		double a=(cos((msg->range_max-secAngle)*M_PI/180)*msg->ranges[secAngle]-cos((msg->range_max-firstAngle)*M_PI/180)*msg->ranges[firstAngle]);
		double d=(sqrt(pow(o,2)+pow(a,2)));
		double wallAngle=asin(o/d);//genera un posible error de +-0.05 (aprox 3 grados de desvio)

		//Extraer los datos de interes

		double minDistance=msg->range_max;
		double detectableRange=0.18;
		double frontUmbral=0.25;
		double lateralUmbral=0.3;
		double angleCurveUmbral=0.349066; //20º
		int relAngle=360;

		for(int i=0;i<=25;i++){
			if(detectableRange<msg->ranges[i] && msg->ranges[i]<minDistance){
				minDistance=msg->ranges[i];
				relAngle=i;
			}
		}
		for(int i=335;i<=358;i++){
			if(detectableRange<msg->ranges[i] && msg->ranges[i]<minDistance){
				minDistance=msg->ranges[i];
				relAngle=i;
			}
		}

		if(!first){
			ROS_INFO_STREAM("Posible ruta a la izquierda");
			wallAngle=M_PI;			
		}


		//Retocar condiciones de estado

		if(minDistance<frontUmbral){//obstruccion delante, tiene que girar
			ss<< "3";
		}else if(wallAngle>angleCurveUmbral||wallAngle<(-1*angleCurveUmbral)){
			ss<<"2";
		}else{
			ss<<"1";
		}

		ss<<":"<<wallAngle<<":"<<minDistance;

  	    msgInfo.data = ss.str();

		chatter_pub.publish(msgInfo);

		ROS_INFO("%s", msgInfo.data.c_str());
		ROS_INFO_STREAM("Sensores empleados: ("<<firstAngle<<","<<secAngle<<","<<relAngle<<")\n");


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