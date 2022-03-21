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
	double const frontUmbral=0.65;//vigilar la sensibilidad con la que hace el giro
	double const shortLateralUmbral=0.45;//vigilar la sensibilidad con la que se le permite acercarse a la pared
	double const largeLateralUmbral=0.55;//vigilar la sensibilidad con la que se le permite alejarse de la pared
	double const angleCurveUmbral=0.349066; //20º sensibilidad de curva
	double const angleTurnUmbral=0.7;//40º sensibilidad de giro
	double const detectableRange=0.15;//Para gestionar error que se genera con el lidar al moverse el robot
	double const increaseFactor=1.2;//Factor para forzar a a giros/curvas mas intensas



	public:
	Procesing(ros::NodeHandle& nh) { 
		// Suscribe el método commandCallback al tópico base_scan (el láser proporcionado por Stage)
		// El método commandCallback será llamado cada vez que el emisor (stage) publique datos 
		laserSub = nh.subscribe("scan", 1, &Procesing::commandCallback, this);

		chatter_pub = nh.advertise<std_msgs::String>("info",1000);
		nh_ = nh;
	};

	// Procesa los datos de láser
	void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

		//parametros necesarios

		std_msgs::String msgInfo;
	    std::stringstream ss;
		int firstAngle=50;//punto mas cercano al frente
		int secAngle=51;//ultimo punto posible
		bool first=false;//indica si se ha encontrado un punto al menos en el area de deteccion a la izquierda
		double minDistanceFrontal=msg->range_max;
		double minDistanceLateral=msg->range_max;
		int relAngleFrontal=360;
		int relAngleLateral=360;

		//Obtiene el angulo de interseccion a la pared de la izquierda

		for(int i= 50;i<=100;i++){
			if(!first&&msg->ranges[i]<=3.5){
				first=true;
				firstAngle=i;
				secAngle=i+1;
			}
			if(first&&msg->ranges[i]<=3.5){
				secAngle=i;
			}
			if(detectableRange<msg->ranges[i] && msg->ranges[i]<minDistanceLateral){
				minDistanceLateral=msg->ranges[i];
				relAngleLateral=i;
			}
		}

		double o=(sin((msg->range_max-secAngle)*M_PI/180)*msg->ranges[secAngle]-sin((msg->range_max-firstAngle)*M_PI/180)*msg->ranges[firstAngle]); 
		double a=(cos((msg->range_max-secAngle)*M_PI/180)*msg->ranges[secAngle]-cos((msg->range_max-firstAngle)*M_PI/180)*msg->ranges[firstAngle]);
		double d=(sqrt(pow(o,2)+pow(a,2)));
		double wallAngle=asin(o/d);//genera un posible error de +-0.05 (aprox 3 grados de desvio)

		//Extraer los datos de interes

		for(int i=0;i<=25;i++){
			if(detectableRange<msg->ranges[i] && msg->ranges[i]<minDistanceFrontal){
				minDistanceFrontal=msg->ranges[i];
				relAngleFrontal=i;
			}
		}
		for(int i=335;i<=358;i++){
			if(detectableRange<msg->ranges[i] && msg->ranges[i]<minDistanceFrontal){
				minDistanceFrontal=msg->ranges[i];
				relAngleFrontal=i;
			}
		}

		if(!first){
			ROS_INFO_STREAM("Posible ruta a la izquierda");
			wallAngle=M_PI/2;			
		}


		//Retocar condiciones de estado

		if(wallAngle>angleTurnUmbral||wallAngle<(-1*angleTurnUmbral)){//umbral de giro alto
			ss<< "3";
			ROS_INFO_STREAM("Caso giro de umbral alto. Realiza giro de: "<<wallAngle*180/M_PI<<"º");

		}else if(minDistanceFrontal<frontUmbral){//obstruccion delante, tiene que girar pero angulo de giro escaso
			ss<< "3";
			wallAngle=wallAngle>0?M_PI/2:-M_PI/2;//metodo temporal
			ROS_INFO_STREAM("Caso giro de colision. Realiza giro de: "<<wallAngle*180/M_PI<<"º");


			//Implementar este pseudocodigo mirando los parametros necesarios
			//modificar angulo segun condiciones de la situacion
			//si delante y a la izquierda parece estar bloqueado, ir a la derecha (minDistanceLateral<ranges[270], por ejemplo)
				//wallAngle=M_PI/2*(-1);
			//si no, a la izquierda se puede realizar un giro de 90º
				//wallAngle=M_PI/2;


		}else if(wallAngle>angleCurveUmbral||wallAngle<(-1*angleCurveUmbral)){//umbral curva leve
			ss<<"2";
			ROS_INFO_STREAM("Caso curva de umbral leve. Realiza giro de: "<<wallAngle*180/M_PI<<"º");

			
		}else if(minDistanceLateral<shortLateralUmbral){//acercarse a la pared, curva hacia fuera (angulo)
				//se aleja pared, curva hacia ella (angulo*-1)
			ss<<"2";
			wallAngle=wallAngle*increaseFactor;
			ROS_INFO_STREAM("Caso curva de acercamiento de pared. Realiza giro de: "<<wallAngle*180/M_PI<<"º");

			
		}else if(minDistanceLateral>largeLateralUmbral){//acercarse a la pared, curva hacia fuera (angulo)
				//se aleja pared, curva hacia ella (angulo*-1)
			ss<<"2";

			wallAngle=sqrt(pow(wallAngle*increaseFactor,2));
			ROS_INFO_STREAM("Caso curva de alejamiento de pared. Realiza giro de: "<<wallAngle*180/M_PI<<"º");

			
		}else{
			ROS_INFO_STREAM("Caso recto. No realiza giro.");
			ss<<"1";
		}

		ss<<":"<<wallAngle<<":"<<minDistanceFrontal;

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