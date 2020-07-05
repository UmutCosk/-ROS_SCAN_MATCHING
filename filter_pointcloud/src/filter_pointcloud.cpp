#include "filter_pointcloud.hpp"

class warning : public std::exception
{
public:
    warning(const std::string &msg) {}
    const char *what() { return msg.c_str(); } //message of warning
private:
    std::string msg;
};

AnglePair::AnglePair(float angle1_in, float angle2_in)
    : angle1(angle1_in),
      angle2(angle2_in)
{
    if (angle1 > angle2)
    {
        throw invalid_argument("angle2 needs to be greater than angle1");
    }
}

void AnglePairs::AddPair(AnglePair angle_pair)
{
    all_angle_pairs.push_back(angle_pair);
}

//@ input: angle_pairs, mirror_x , mirror_y , channel_mode , use_custom_channels if channel_mode is customChannels
FilterPC::FilterPC(vector<AnglePair> angle_pairs_in, ChannelMode channel_mode_in, vector<float> use_custom_channels_in)
    : angle_pairs(angle_pairs_in),
      channel_mode(channel_mode_in),
      custom_channels(use_custom_channels_in)
{
    if (!use_custom_channels_in.empty() && channel_mode_in == ChannelMode::customChannels)
    {
        for (float channel : use_custom_channels_in)
        {
            if (channel < 0 || channel > 7)
            {
                throw invalid_argument("Channel has to be between 0 and 7");
            }
        }
    }
    if (use_custom_channels_in.empty() && channel_mode_in == ChannelMode::customChannels)
    {

        cout << "WARNING!: You use custom Channels but use_custom_channels vector is empty!" << endl
             << "All channels are used instead now!" << endl;
        channel_mode = ChannelMode::allChannels;
    }

    for (AnglePair angle_pair : angle_pairs_in)
    {
        if (angle_pair.angle1 < 0 || angle_pair.angle2 < 0 || angle_pair.angle1 > 360 || angle_pair.angle2 > 360)
        {
            throw invalid_argument("One of your angles are smaller than 0 or greater than 360!");
        }
    }
}

bool FilterPC::checkIfCustomChannel(float check_channel)
{
    for (float channel : custom_channels)
    {
        if (check_channel == channel)
        {
            return true;
        }
    }
    return false;
}
sensor_msgs::PointCloud FilterPC::filterLaserChannel(const sensor_msgs::PointCloud &pc)
{
    //--------Use All Channels , nothing to filter
    if (channel_mode == ChannelMode::allChannels)
    {
        return pc;
    }
    sensor_msgs::PointCloud new_pc;
    new_pc.header = pc.header;
    geometry_msgs::Point32 point;
    sensor_msgs::ChannelFloat32 channel;
    //----------Channels from 4-7 (Top Layers)
    if (channel_mode == ChannelMode::topLayers)
    {
        for (int i = 0; i < pc.channels[0].values.size(); i++)
        {

            if (pc.channels[0].values[i] == 4 || pc.channels[0].values[i] == 5 || pc.channels[0].values[i] == 6 || pc.channels[0].values[i] == 7)
            {
                channel.values.push_back(pc.channels[0].values[i]);
                new_pc.points.push_back(pc.points[i]);
            }
        }
        new_pc.channels.push_back(channel);
        return new_pc;
    }
    //----------Channels from 0-3 (Bottom Layers)
    if (channel_mode == ChannelMode::botLayers)
    {
        for (int i = 0; i < pc.channels[0].values.size(); i++)
        {

            if (pc.channels[0].values[i] == 0 || pc.channels[0].values[i] == 1 || pc.channels[0].values[i] == 2 || pc.channels[0].values[i] == 3)
            {
                channel.values.push_back(pc.channels[0].values[i]);
                new_pc.points.push_back(pc.points[i]);
            }
        }
        new_pc.channels.push_back(channel);
        return new_pc;
    }
    //---------Custom Channels
    if (channel_mode == ChannelMode::customChannels)
    {
        for (int i = 0; i < pc.channels[0].values.size(); i++)
        {

            if (checkIfCustomChannel(pc.channels[0].values[i]))
            {
                channel.values.push_back(pc.channels[0].values[i]);
                new_pc.points.push_back(pc.points[i]);
            }
        }
        new_pc.channels.push_back(channel);
        return new_pc;
    }
    return pc;
}

//@ input: x , y
float FilterPC::calcAngle(float x, float y)
{
    float result = atan2(y, x) * 180 / PI - 90;
    if (result < 0)
    {
        result += 360;
    }
    return result;
}

bool FilterPC::checkIfWithinAngle(float check_angle)
{
    for (AnglePair angle_pair : angle_pairs)
    {
        if (check_angle >= angle_pair.angle1 && check_angle <= angle_pair.angle2)
        {
            return true;
        }
    }
    return false;
}

sensor_msgs::PointCloud FilterPC::filterAngle(const sensor_msgs::PointCloud &pc)
{

    sensor_msgs::PointCloud new_pc;
    vector<geometry_msgs::Point32> points = pc.points;
    sensor_msgs::ChannelFloat32 channel;
    new_pc.header = pc.header;
    float check_angle;
    int index = 0;
    for (geometry_msgs::Point32 point : points)
    {

        check_angle = calcAngle(point.x, point.y);
        if (!checkIfWithinAngle(check_angle))
        {
            new_pc.points.push_back(point);
            channel.values.push_back(pc.channels[0].values[index]);
        }
        index++;
    }
    new_pc.channels.push_back(channel);

    return new_pc;
}
