#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
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
	tf::TransformListener tf_listener_;
	ros::Publisher publisher_;
	ros::Publisher vis_pub_;

	std::string sub_topic;
	std::string pub_topic;
	geometry_msgs::Twist vel_message;

	int queue_size;
	float kp_ang, kp_lin;

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
};

} /* namespace */
