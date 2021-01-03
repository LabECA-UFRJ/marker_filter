#include "ros/ros.h"
#include "aruco_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"

#include <iostream>
#include <inttypes.h>
#include <cmath>

using namespace std;

ros::Publisher pub;
int id;

void markerReceived(const aruco_msgs::MarkerArray::ConstPtr &markers)
{
    for (int i = 0; i < markers->markers.size(); i++) {
        if (markers->markers[i].id == id) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position = markers->markers[i].pose.pose.position;
            pose.pose.orientation = markers->markers[i].pose.pose.orientation;

            pub.publish(pose);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_filter");
    ros::NodeHandle nodeHandle;

    if (ros::param::get("~id", id) == false)
    {
        ROS_FATAL("Parameter id not set.");
        return -1;
    }

    pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("pose", 5);

    ros::Subscriber sub = nodeHandle.subscribe("markers", 1, markerReceived);

    ros::spin();

    return 0;
}
