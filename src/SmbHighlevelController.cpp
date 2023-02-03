#include "smb_highlevel_controller/SmbHighlevelController.hpp"

namespace smb_highlevel_controller {


/* Constructor */
SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
	// get param from param.yaml
	if(!SmbHighlevelController::readParameters()){
		 ROS_ERROR("Could not find params!");
		 ros::requestShutdown();
	 }

	// create subscriber
	subscriber_= nodeHandle_.subscribe(sub_topic,queue_size, &SmbHighlevelController::scanCallback, this);

	// create publisher
	publisher_ = nodeHandle_.advertise<geometry_msgs::Twist>(pub_topic,1);

	ROS_INFO("Successfully launched node.");
}

/* Destructor */
SmbHighlevelController::~SmbHighlevelController()
{
}


bool SmbHighlevelController::readParameters(void){
	if (!nodeHandle_.getParam("subscriber_topic", sub_topic) ||
			!nodeHandle_.getParam("publisher_topic", pub_topic) ||
			!nodeHandle_.getParam("queue_size", queue_size) ||
			!nodeHandle_.getParam("kp_ang", kp_ang) ||
			!nodeHandle_.getParam("kp_lin", kp_lin))
	{
		return 0;
	} else {
		return 1;
	}
}

float SmbHighlevelController::limit(const float value, const float limit){
	if(value > limit){
		return limit;
	} else if (value < 0){
		return 0;
	} else {
		return value;
	}
}

float SmbHighlevelController::getGain(const float value, const std::string &dof){
	float kp;

	if(dof == "angular"){
		kp = limit(kp_ang*value,1);
	} else if (dof == "linear"){
		kp = limit(kp_lin*(value-1.0),1);
	}

	return kp;
}

void SmbHighlevelController::scanCallback(const sensor_msgs::LaserScan& msg)
{
	std::vector<float> ranges;
	int n, i, index;
	float minDist = 1e6, angle;

	ranges = msg.ranges;
	n = ranges.size();

	for(i = 0; i<n;i++){
		if(ranges[i]<minDist){
			minDist = ranges[i];
			index = i;
		}
	}

	angle = msg.angle_min+index*msg.angle_increment;

	ROS_INFO_STREAM("Distance:" << minDist<<"angle:" << angle);


	// Publish
	vel_message.angular.z = getGain(angle, "angular");
	vel_message.linear.x = getGain(minDist, "linear");

	publisher_.publish(vel_message);
	ROS_INFO_STREAM("lin:" << vel_message.linear.x << "ang:" <<vel_message.angular.z);

}

} /* namespace */
