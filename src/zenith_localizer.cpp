#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "apriltags_ros/AprilTagDetectionArray.h"
#include "apriltags_ros/AprilTagDetection.h"
#include <XmlRpcException.h>

ros::Publisher roboPose;

geometry_msgs::Pose2D pose2D_msg;

std::map<int, apriltags_ros::AprilTagDetection> TagGlobalPose;

//Parsing ROS Params
std::map<int, apriltags_ros::AprilTagDetection> parse_tag_descriptions(XmlRpc::XmlRpcValue& tag_descriptions){


  std::map<int, apriltags_ros::AprilTagDetection> descriptions;

  ROS_ASSERT(tag_descriptions.getType() == XmlRpc::XmlRpcValue::TypeArray);

  for (int32_t i = 0; i < tag_descriptions.size(); ++i) {

    XmlRpc::XmlRpcValue& tag_description = tag_descriptions[i];

    ROS_ASSERT(tag_description.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    ROS_ASSERT(tag_description["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);

    ROS_ASSERT(tag_description["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    ROS_ASSERT(tag_description["x"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    ROS_ASSERT(tag_description["y"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    ROS_ASSERT(tag_description["z"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    ROS_ASSERT(tag_description["qx"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    ROS_ASSERT(tag_description["qy"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    ROS_ASSERT(tag_description["qz"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    ROS_ASSERT(tag_description["qw"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    int id = (int)tag_description["id"];

    double size = (double)tag_description["size"];

    double x = (double)tag_description["x"];

    double y = (double)tag_description["y"];

    double z = (double)tag_description["z"];

    double qx = (double)tag_description["qx"];

    double qy = (double)tag_description["qy"];

    double qz = (double)tag_description["qz"];

    double qw = (double)tag_description["qw"];

    std::string frame_name;

    if(tag_description.hasMember("frame_id")){

      ROS_ASSERT(tag_description["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);

      frame_name = (std::string)tag_description["frame_id"];

    }

    else{

      std::stringstream frame_name_stream;

      frame_name_stream << "tag_" << id;

      frame_name = frame_name_stream.str();

    }

    //AprilTagDescription description(id, size, frame_name);

    apriltags_ros::AprilTagDetection description;

    ROS_INFO_STREAM("Loaded tag config: "<<id<<", size: "<<size<<", frame_name: "<<frame_name);
    description.id = id;
    description.size = size;
    description.pose.pose.position.x = x;
    description.pose.pose.position.y = y;
    description.pose.pose.position.z = z;
    description.pose.pose.orientation.x = qx;
    description.pose.pose.orientation.y = qy;
    description.pose.pose.orientation.z = qz;
    description.pose.pose.orientation.w = qw;

    
    descriptions.insert(std::make_pair(id, description));

  }

  return descriptions;

}


//Soruce https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
void toEulerianAngle(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw)
{
//	ROS_INFO("Euler Convesion");
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

void aprilTagCallBack(const apriltags_ros::AprilTagDetectionArray::ConstPtr& aprilTagArray){
	double roll, pitch, yaw;

//	ROS_INFO("April Tag List");

	ROS_INFO("%d Tags Detected", aprilTagArray->detections.size());

	for(int i = 0; i < aprilTagArray->detections.size(); i++){

		ROS_INFO("Tag %d Detected", aprilTagArray->detections[i].id );
		toEulerianAngle(aprilTagArray->detections[i].pose.pose.orientation, roll, pitch, yaw);
		ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);

	}



}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom){
//	ROS_INFO("odomCallback");
	double roll, pitch;
	double yaw = -1;
	
	pose2D_msg.x = odom->pose.pose.position.x;
	pose2D_msg.y = odom->pose.pose.position.y;
	
	toEulerianAngle(odom->pose.pose.orientation, roll, pitch, yaw);
	
//	ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
	
	pose2D_msg.theta = yaw;
	
	roboPose.publish(pose2D_msg);

}

int main(int argc, char **argv){
	
	
	ros::init(argc, argv, "zenith_localizer");
	
	ROS_INFO("Hello World");
	ROS_INFO_STREAM("Hello World 2");
	
	ros::NodeHandle n;
	//Retirve ROS Params
	  XmlRpc::XmlRpcValue april_tag_descriptions;
	  if(!n.getParam("/apriltag_detector/tag_descriptions", april_tag_descriptions)){
	    ROS_WARN("No april tags specified");
	  }
	  else{
	    try{
	      TagGlobalPose = parse_tag_descriptions(april_tag_descriptions);
	    } catch(XmlRpc::XmlRpcException e){
	      ROS_ERROR_STREAM("Error loading tag descriptions: "<<e.getMessage());
	    }
	  }
	

	for (std::map<int, apriltags_ros::AprilTagDetection>::iterator it=TagGlobalPose.begin(); it!=TagGlobalPose.end(); ++it)
	{    
		ROS_INFO_STREAM("New It" << std::endl << it->second);	
		//ROS_INFO("ID: %d, Size: %f", it->second.id, it->second.size);
	}

	
	roboPose = n.advertise<geometry_msgs::Pose2D>("/zenith/pose2D", 5);
	
	ros::Subscriber sub1 = n.subscribe("/zed/odom", 5, odomCallback);
	ros::Subscriber sub2 = n.subscribe("/zenith/tag_detections", 5, aprilTagCallBack);
	
	ros::spin();
	
	return 0;
	
}
