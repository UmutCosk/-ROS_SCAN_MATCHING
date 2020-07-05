#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <std_msgs/Header.h>
#include <vector>
#include <math.h>
#include <stdio.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#define PI 3.14159265

using namespace sensor_msgs;
using namespace std;

enum ChannelMode
{
    topLayers,
    botLayers,
    allChannels,
    customChannels
};

struct AnglePair
{
public:
    AnglePair(float angle1_in, float angle2_in);

public:
    float angle1;
    float angle2;
};

class AnglePairs
{
public:
    AnglePairs() = default;

public:
    void AddPair(AnglePair);

public:
    vector<AnglePair> all_angle_pairs;
};

class FilterPC
{
public:
    FilterPC() = default;
    FilterPC(vector<AnglePair> angle_pairs_in, ChannelMode channel_mode_in, vector<float> use_custom_channels = {});

public:
    sensor_msgs::PointCloud filterLaserChannel(const sensor_msgs::PointCloud &pc);
    sensor_msgs::PointCloud filterAngle(const sensor_msgs::PointCloud &pc);
    bool checkIfCustomChannel(float check_channel);
    bool checkIfWithinAngle(float check_angle);
    float calcAngle(float x, float y);

public:
    vector<AnglePair> angle_pairs;

    ChannelMode channel_mode;
    vector<float> custom_channels;
};