/*
This node filters a sensor_msgs::PointCloud2 message by channel and angle and outputs a sensor_msgs::PointCloud message.
The angle starts with 0 on the x-axis of the car and goes to 360 counter-clockwise.

Examples:
angle 0 / 360 : being in front of the car.
angle 90: left to the car
angle 180: behind the car
angle 270: right to car
and so on...
*/

#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <std_msgs/Header.h>
#include <vector>

#include "filter_pointcloud.hpp"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace sensor_msgs;
using namespace std;

//Variables
bool init = false;
FilterPC filter_pc;
sensor_msgs::PointCloud new_pc;

void callback(const PointCloud2::ConstPtr &point_cloud)
{
    if (!init)
    {
        init = true;
        //Set pairs of angles. Points within those two angles will be filtered
        AnglePairs angle_pairs;
        angle_pairs.AddPair(AnglePair(0, 90));
        angle_pairs.AddPair(AnglePair(180, 270));
        // angle_pairs.AddPair(AnglePair(105, 165));
        // angle_pairs.AddPair(AnglePair(195, 255));

        //Set laser channels that should be used. Note: customChannels needs to be chosen in order to use.
        vector<float> custom_layers = {0, 3, 6};

        //---------Create Filter------------//
        filter_pc = FilterPC(angle_pairs.all_angle_pairs, ChannelMode::allChannels, custom_layers);
    }
    sensor_msgs::convertPointCloud2ToPointCloud(*point_cloud, new_pc);
    new_pc = filter_pc.filterAngle(new_pc);
    new_pc = filter_pc.filterLaserChannel(new_pc);
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "pointcloud_filter_node");
    ros::NodeHandle n;
    //Subscriber
    ros::Subscriber sub = n.subscribe("/as_tx/point_cloud", 1000, callback);
    //Publisher
    auto new_pc_pub = n.advertise<sensor_msgs::PointCloud>("filtered_pointcloud", 100);
    ros::Rate rate(120);
    while (ros::ok())
    {
        //Show Filtered PointCloud
        new_pc.header.stamp = ros::Time::now();
        new_pc.header.frame_id = "ibeo_lux";
        new_pc_pub.publish(new_pc);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}