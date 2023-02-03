#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
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
	ros::Publisher publisher_;
	std::string sub_topic;
	std::string pub_topic;
	geometry_msgs::Twist vel_message;

	int queue_size;
	float kp_ang, kp_lin;

	/* parameter function*/
	bool readParameters(void);

	/* limit function*/
	float limit(const float value, const float limit);

	/* calculate gains*/
	float getGain(const float angle, const std::string &dof);

	/* Callback Function*/
	void scanCallback(const sensor_msgs::LaserScan& msg);
};

} /* namespace */
