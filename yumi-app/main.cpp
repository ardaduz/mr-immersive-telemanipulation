
#include "MRTelemanipulationWithCameraApp.h"
#include <ros/ros.h>

int main(int argc, char** argv) {

	ros::init(argc, argv, "MRTelemanipulationWithCameraApp");

	MRTelemanipulationWithCameraApp app;
	app.runMainLoop();

	return 0;
}


