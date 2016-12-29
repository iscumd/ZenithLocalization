#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "zenith_localization/AprilTagDetectionArray.h"

ros::Publisher roboPose;

geometry_msgs::Pose2D pose2D_msg;

//Soruce https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
void toEulerianAngle(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw)
{
	ROS_INFO("Euler Convesion");
	double ysqr = q.y * q.y;

	// roll (x-axis rotation)
	double t0 = +2.0f * (q.w * q.x + q.y * q.z);
	double t1 = +1.0f - 2.0f * (q.x * q.x + ysqr);
	roll = std::atan2(t0, t1);

	// pitch (y-axis rotation)
	double t2 = +2.0f * (q.w * q.y - q.z * q.x);
	t2 = t2 > 1.0f ? 1.0f : t2;
	t2 = t2 < -1.0f ? -1.0f : t2;
	pitch = std::asin(t2);

	// yaw (z-axis rotation)
	double t3 = +2.0f * (q.w * q.z + q.x *q.y);
	double t4 = +1.0f - 2.0f * (ysqr + q.z * q.z);  
	yaw = std::atan2(t3, t4);
}

void aprilTagCallBack(const zenith_localization::AprilTagDetectionArray::ConstPtr& aprilTagArray){
	double roll, pitch, yaw;

	ROS_INFO("April Tag List");

	ROS_INFO("%d Tags Detected", aprilTagArray->detections.size());

	for(int i = 0; i < aprilTagArray->detections.size(); i++){

		ROS_INFO("Tag %d Detected", aprilTagArray->detections[i].id );
		toEulerianAngle(aprilTagArray->detections[i].pose.pose.orientation, roll, pitch, yaw);
		ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);

	}



}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom){
	ROS_INFO("odomCallback");
	double roll, pitch;
	double yaw = -1;
	
	pose2D_msg.x = odom->pose.pose.position.x;
	pose2D_msg.y = odom->pose.pose.position.y;
	
	toEulerianAngle(odom->pose.pose.orientation, roll, pitch, yaw);
	
	ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
	
	pose2D_msg.theta = yaw;
	
	roboPose.publish(pose2D_msg);

}

int main(int argc, char **argv){


ros::init(argc, argv, "zenith_localizer");

ROS_INFO("Hello World");

ros::NodeHandle n;

roboPose = n.advertise<geometry_msgs::Pose2D>("/zenith/pose2D", 5);

ros::Subscriber sub1 = n.subscribe("/zed/odom", 5, odomCallback);
ros::Subscriber sub2 = n.subscribe("/zenith/tag_detections", 5, aprilTagCallBack);

ros::spin();

return 0;

}
