#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <tf2_ros/transform_broadcaster.h>

class odom_tf2 {

private:

    ros::NodeHandle n;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    geometry_msgs::TransformStamped transformStamped;
    tf2_ros::TransformBroadcaster br;

public:

    odom_tf2() {
        sub1 = n.subscribe("/odom", 1, &odom_tf2::from_odom_to_footprint, this);
        //sub2 = n.subscribe("/base_footprint", 1, &odom_tf2::from_footprint_to_link, this);
    }

    void from_odom_to_footprint(const nav_msgs::Odometry::ConstPtr& msg) {

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";
        // set x, y, z
        transformStamped.transform.translation.x = msg->pose.pose.position.x;
        transformStamped.transform.translation.y = msg->pose.pose.position.y;
        transformStamped.transform.translation.z = msg->pose.pose.position.z;
        // set theta
        transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
        transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
        transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
        transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;
        // send transform

        br.sendTransform(transformStamped);
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "transform");
    odom_tf2 odom;
    ros::spin();
    return 0;
}
