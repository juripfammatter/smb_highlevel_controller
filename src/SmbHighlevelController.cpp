#include "smb_highlevel_controller/SmbHighlevelController.hpp"

namespace smb_highlevel_controller {


/* Constructor */
SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
	// get param from param.yaml
	if(!nodeHandle_.getParam("subscriber_topic", topic) || !nodeHandle_.getParam("queue_size", queue_size)){
		 ROS_ERROR("Could not find params!");
		 ros::requestShutdown();
	 }

	// create subscriber
	subscriber_= nodeHandle_.subscribe(topic,queue_size, &SmbHighlevelController::ScanCallback, this);
	ROS_INFO("Successfully launched node.");
}

/* Destructor */
SmbHighlevelController::~SmbHighlevelController()
{
}

void SmbHighlevelController::ScanCallback(const sensor_msgs::LaserScan& msg)
{
	auto min_elem = std::min_element(msg.ranges.cbegin(), msg.ranges.cend());
	ROS_INFO_STREAM("\n Minimum range:" << *min_elem);
}

} /* namespace */
