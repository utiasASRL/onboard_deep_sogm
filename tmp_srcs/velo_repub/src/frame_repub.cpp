#include "frame_repub.h"
#include <chrono>
#include <thread>

//-----------------------------------------------------------------------------------------------------------------------------
// Repub function
// **************

void Republisher::gotCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	//////////////////////
	// Optional verbose //
	//////////////////////

	// Re-publishpointcloud for visu
	frame_visu_pub.publish(*msg);

	if (n_frames > 20)
		return;

	// Update number of frames
	n_frames++;

	return;
}

//-----------------------------------------------------------------------------------------------------------------------------
// Main call
// *********

int main(int argc, char **argv)
{

	///////////////////
	// Init ROS node //
	///////////////////

	// ROS init
	ROS_WARN("Initializing velo_repub");
	ros::init(argc, argv, "velo_repub");

	// Node handler and publishers
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	//ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("slam_pose", 1000);
	//ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("pointmap", 1000);

	//////////////////////
	// Init repub class //
	//////////////////////

	// Create a the SLAM class
	Republisher republisher;
	republisher.frame_visu_pub = nh.advertise<sensor_msgs::PointCloud2>("velo_visu", 1000, true);

	///////////////////////
	// Start subscribing //
	///////////////////////

	// Subscribe to the lidar topic and the transforms topic
	//ros::Subscriber tf_sub = nh.subscribe("tf", 1000, mapper.update_transforms);
	string points_topic = "/velodyne_points";
	ROS_WARN_STREAM("Subscribing to " << points_topic);
	ros::Subscriber lidar_sub = nh.subscribe(points_topic, 1000, &Republisher::gotCloud, &republisher);
	ros::spin();

	return 0;
}
