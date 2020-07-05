#include "ros/ros.h"
#include "ukf_state_msg/State.h"

int main(int argc,char **argv){
    
    ros::init(argc,argv,"pubMsg");
    ros::NodeHandle n;

    ros::Publisher msgPub = n.advertise<ukf_state_msg::State("ukf_state",1000);
    ros::Rate loop_rate(1);

    int counter = 0;
    srand(time(NULL));
    while(ros::ok()){
        ukf_state_msg::State msg;
        ros::spinOnce();
        loop_rate.sleep();
        counter++;

    }
    return 0;
}