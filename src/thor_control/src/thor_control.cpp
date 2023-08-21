#include <ros/ros.h>
#include <thor_control/pick_place_task.h>

constexpr char LOGNAME[] = "thor_control";

int main(int argc, char** argv) {
	ros::init(argc, argv, "thor_control");
	ros::NodeHandle nh, pnh("~");

	// Handle Task introspection requests from RViz & feedback during execution
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Construct and run pick/place task
	thor_control::PickPlaceTask pick_place_task("pick_place_task", pnh);
	if (!pick_place_task.init()) {
		ROS_INFO_NAMED(LOGNAME, "Initialization failed");
		return 1;
	}

	if (pick_place_task.plan()) {
		ROS_INFO_NAMED(LOGNAME, "Planning succeded");
		if (pnh.param("execute", false)) {
			pick_place_task.execute();
			ROS_INFO_NAMED(LOGNAME, "Execution complete");
		} else {
			ROS_INFO_NAMED(LOGNAME, "Execution disabled");
		}
	} else {
		ROS_INFO_NAMED(LOGNAME, "Planning failed");
	}

	// Keep introspection alive
	ros::waitForShutdown();
	return 0;
}