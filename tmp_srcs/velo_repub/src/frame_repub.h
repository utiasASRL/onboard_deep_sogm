#pragma once

#include <cstdint>
#include <cstdio>
#include <ctime>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

using namespace std;


// repub class
// **********

class Republisher
{
public:

	// Elements
	// ********

	// Current number of aligned frames
	int n_frames;
	ros::Publisher frame_visu_pub;

	// Methods
	// *******

	// Constructor
	Republisher()
	{
		// Init parameters
		n_frames = 0;
	}

	// Method	
	void gotCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);

};

// Main
// ****

int main(int argc, char **argv);