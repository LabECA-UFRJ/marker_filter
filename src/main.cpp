#include "ros/ros.h"
#include "aruco_msgs/MarkerArray.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;

int id;
string frame_name;

void markerReceived(const aruco_msgs::MarkerArray::ConstPtr &markers)
{
    for (int i = 0; i < markers->markers.size(); i++) {
        if (markers->markers[i].id == id) {
            static tf2_ros::TransformBroadcaster br;
            geometry_msgs::TransformStamped transformStamped;

            transformStamped.header.stamp = ros::Time::now();
            // In our case, the world frame is the camera default frame
            transformStamped.header.frame_id = "world";
            transformStamped.child_frame_id = frame_name;

            // This swaps and minus signals makes no sense but works
            transformStamped.transform.translation.x = -markers->markers[i].pose.pose.position.x;
            transformStamped.transform.translation.y = -markers->markers[i].pose.pose.position.z;
            transformStamped.transform.translation.z = -markers->markers[i].pose.pose.position.y;

            transformStamped.transform.rotation.x = markers->markers[i].pose.pose.orientation.x;
            transformStamped.transform.rotation.y = markers->markers[i].pose.pose.orientation.y;
            transformStamped.transform.rotation.z = -markers->markers[i].pose.pose.orientation.z;
            transformStamped.transform.rotation.w = markers->markers[i].pose.pose.orientation.w;

            br.sendTransform(transformStamped);
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

    if (ros::param::get("~frame_name", frame_name) == false)
    {
        ROS_FATAL("Parameter frame_name not set.");
        return -1;
    }    

    ros::Subscriber sub = nodeHandle.subscribe("markers", 1, markerReceived);

    ros::spin();

    return 0;
}
