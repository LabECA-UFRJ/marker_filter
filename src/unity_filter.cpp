#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "simulation_msgs/PoseRobotArray.h"

#include <iostream>

using namespace std;

ros::Publisher pub;
int id;

void arrayReceived(const simulation_msgs::PoseRobotArray::ConstPtr &array)
{
    for (int i = 0; i < array->robots.size(); i++) {
        if (array->robots[i].id == id) {
            geometry_msgs::Pose pose;
            pose.position = array->robots[i].pose.position;
            pose.orientation = array->robots[i].pose.orientation;

            pub.publish(pose);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "unity_filter");
    ros::NodeHandle nodeHandle;

    if (ros::param::get("~id", id) == false)
    {
        ROS_FATAL("Parameter id not set.");
        return -1;
    }

    pub = nodeHandle.advertise<geometry_msgs::Pose>("out_pose", 1000);

    ros::Subscriber sub = nodeHandle.subscribe("robots", 1, arrayReceived);

    ros::spin();

    return 0;
}