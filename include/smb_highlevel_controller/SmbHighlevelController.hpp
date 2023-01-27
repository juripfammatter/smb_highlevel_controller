#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>

namespace smb_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class SmbHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	SmbHighlevelController(ros::NodeHandle& nodeHandle);


	SmbHighlevelController(const SmbHighlevelController &) = delete;
	SmbHighlevelController& operator=(const SmbHighlevelController &) = delete;


	/*!
	 * Destructor.
	 */
	virtual ~SmbHighlevelController();

private:
	/* data */
	ros::NodeHandle nodeHandle_;
	ros::Subscriber subscriber_;
	std::string topic;
	int queue_size;

	/* Callback Function*/
	void ScanCallback(const sensor_msgs::LaserScan& msg);
};

} /* namespace */
