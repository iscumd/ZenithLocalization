#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"

ros::Publisher roboPose;

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom){

roboPose.publish(odom->pose.pose);

}

int main(int argc, char **argv){


ros::init(argc, argv, "zenith_localizer");

ros::NodeHandle n;

roboPose = n.advertise<geometry_msgs::Pose>("/zenith/pose", 5);

ros::Subscriber sub = n.subscribe("/zed/odom", 5, odomCallback);

ros::spin();

return 0;

}
