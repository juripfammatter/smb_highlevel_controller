#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <std_srvs/SetBool.h>

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
	tf::TransformListener tf_listener_;
	ros::Publisher publisher_;
	ros::Publisher vis_pub_;
	ros::ServiceServer service_;

	std::string sub_topic;
	std::string pub_topic;
	std::string emergency_stop_service;
	geometry_msgs::Twist vel_message;
	

	int queue_size;
	float kp_ang, kp_lin;
	bool emergency_stop = true;

	/* parameter function*/
	bool readParameters(void);

	/* saturated P-controller*/
	// limit function
	float limit(const float value, const float limit);

	// calculate gains
	float getGain(const float angle, const std::string &dof);

	/* visualize marker*/
	void visMarker(const tf::Vector3 pos);

	/* transform */
	// Transforms from source_frame to target_frame using tf transforms
	// returns tf object
	tf::StampedTransform getTransform(tf::StampedTransform transform, const std::string target_frame, const std::string source_frame);

	/* Callback Function*/
	void scanCallback(const sensor_msgs::LaserScan& msg);

	/* emergency stop*/
	// true : emergency stop
	bool emergencyStop(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
};

} /* namespace */
