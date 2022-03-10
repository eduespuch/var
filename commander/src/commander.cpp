#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
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
            sub=nh_.subscribe("info",1000,Commander::transmissionACK);
        }

        static void transmissionACK(const std_msgs::String::ConstPtr& msg){

            ROS_INFO("Recibido mensaje: [%s]",msg->data.c_str());

        }

        //Loop forever while sending drive commands based on keyboard input
        bool driveKeyboard() {
             std::cout << "Type a command and then press enter. " <<
                   "Use '+' to move forward, 'l' to turn left, " << "'r' to turn right, '.' to exit.\n";
             //we will be sending commands of type "twist"
             geometry_msgs::Twist base_cmd;
             char cmd[50];
             while(nh_.ok()) {
                 std::cin.getline(cmd, 50);
                 if(cmd[0]!='+' && cmd[0]!='l' && cmd[0]!='r' && cmd[0]!='.') {
                     std::cout << "unknown command:" << cmd << "\n";
                     continue;
                 }
                 base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0; 
                 //move forward
                 if(cmd[0]=='+') {
                     base_cmd.linear.x = 0.25;
                 } 
                 //turn left (yaw) and drive forward at the same time
                 else if(cmd[0]=='l') {
                     base_cmd.angular.z = 0.75;
                     base_cmd.linear.x = 0.25;
                 } 
                 //turn right (yaw) and drive forward at the same time
                 else if(cmd[0]=='r') {
                     base_cmd.angular.z = -0.75;
                     base_cmd.linear.x = 0.25;
                 } 
                 //quit
                 else if(cmd[0]=='.') {
                     break;
                 }
                 //publish the assembled command
                 cmd_vel_pub_.publish(base_cmd);
            }
            return true;
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