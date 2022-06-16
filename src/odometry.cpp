#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <tf2_ros/transform_broadcaster.h>

class odom_tf2 {

private:

    ros::NodeHandle n;

    ros::Subscriber sub;
    ros::Publisher pub;
    geometry_msgs::TransformStamped transformStamped;

public:

    odom_tf2() {
        sub = n.subscribe("/odometry/local", 1, &odom_tf2::callback, this);
    }

    void callback(const nav_msgs::Odometry::ConstPtr& msg){
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
        tf2_ros::TransformBroadcaster br;
        br.sendTransform(transformStamped);

    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "subscribe_and_publish");
    odom_tf2 my_pub_sub;
    ros::spin();
    return 0;
}
