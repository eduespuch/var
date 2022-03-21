#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>


using namespace std_msgs;

class Commander
{
    private:
        //The node handle we'll be using
        ros::NodeHandle nh_;
        //We will be publishing to the "/base_controller/command" topic to issue commands
        ros::Publisher cmd_vel_pub_;
        ros::Subscriber sub;

    public:
        //ROS node initialization
        Commander(ros::NodeHandle &nh) {
            nh_ = nh;
            //set up the publisher for the cmd_vel topic
            cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
            sub=nh_.subscribe("info",1000,&Commander::transmissionACK,this);
        }

         void order(int estado,float angulo,float distancia){//Toma las decisiones en base a los valores extraídos del mensaje.

           

            geometry_msgs::Twist base_cmd;

            ROS_INFO_STREAM("Estado: " << estado); // Mínimo valor angular del láser
            ROS_INFO_STREAM("Angulo: " << angulo); // Mínimo valor angular del láser
            ROS_INFO_STREAM("Distancia: " << distancia); // Mínimo valor angular del láser

            switch(estado){

                case 1:     //Ir recto
                    base_cmd.angular.z = 0.00;
                    base_cmd.linear.x = 0.25;
                    break;


                case 2:
                    base_cmd.angular.z = 0.45*angulo;
                    base_cmd.linear.x = 0.15;
                    break;

                case 3:
                    base_cmd.angular.z = 0.8*angulo;
                    base_cmd.linear.x = 0.15;
                    break;

                default:
                    ROS_INFO_STREAM("Invalid data.");

            }

            cmd_vel_pub_.publish(base_cmd);

        }

     void extractInfo(float* ptr){//Separa los valores individuales del mensaje.

            float aux;
            
            int estado=(int)*ptr;
            float angulo=*(ptr+1);
            float distancia=*(ptr+2);

            order(estado,angulo,distancia);

        }

         void transmissionACK(const std_msgs::String::ConstPtr& msg){//Recibe los mensajes y construye un formato usable por nuestro código.

            std::stringstream ss;

            int i=0;

            float info[3];
            ss<<msg->data.c_str();

            std::string aux;
            
            while(getline(ss,aux,':')){

                info[i]=stof(aux);
                i++;

            }

            extractInfo(info);

        }

};

int main(int argc, char** argv) {
        
        //Inicializa el nodo.
        ros::init(argc, argv, "Commander");
        ros::NodeHandle n;
        Commander driver(n);
        ros::spin();
        //driver.driveKeyboard();
        return 0;
}