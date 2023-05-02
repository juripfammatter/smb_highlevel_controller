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
	vis_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

	// advertise emergency service
	service_ = nodeHandle_.advertiseService(emergency_stop_service, &SmbHighlevelController::emergencyStop, this);

	ROS_INFO("Successfully launched node.");
}

/* Destructor */
SmbHighlevelController::~SmbHighlevelController()
{
}


/* parameter function */
bool SmbHighlevelController::readParameters(void){
	if (!nodeHandle_.getParam("subscriber_topic", sub_topic) ||
		!nodeHandle_.getParam("publisher_topic", pub_topic) ||
		!nodeHandle_.getParam("queue_size", queue_size) ||
		!nodeHandle_.getParam("kp_ang", kp_ang) ||
		!nodeHandle_.getParam("kp_lin", kp_lin) ||
		!nodeHandle_.getParam("service_name", emergency_stop_service))
	{
		return 0;
	} else {
		return 1;
	}
}

/* saturated P-controller */
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

/* marker visualizer */
void SmbHighlevelController::visMarker(const tf::Vector3 pos){
	visualization_msgs::Marker marker;
	marker.header.frame_id = "odom";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = pos.x();
	marker.pose.position.y = pos.y();
	marker.pose.position.z = pos.z();
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1.8;
	marker.scale.y = 1.8;
	marker.scale.z = 3;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	vis_pub_.publish( marker );
}

/* transform */
tf::StampedTransform SmbHighlevelController::getTransform(tf::StampedTransform transform, const std::string target_frame, const std::string source_frame){
	try
	{
		tf_listener_.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
	}
	catch(tf::TransformException &ex)
	{
		ROS_WARN("%s", ex.what());
		ros::Duration(1.0).sleep();
	}
	return transform;
}

/* callback */
void SmbHighlevelController::scanCallback(const sensor_msgs::LaserScan& msg)
{
	std::vector<float> ranges;
	int n, i, index;
	float min_dist = 1e6, angle;
	float x_pos_rslidar, y_pos_rslidar;
	float x_pos_odom, y_pos_odom;

	// get distance and angle
	ranges = msg.ranges;
	n = ranges.size();

	for(i = 0; i<n;i++){
		if(ranges[i]<min_dist){
			min_dist = ranges[i];
			index = i;
		}
	}

	angle = msg.angle_min+index*msg.angle_increment;

	ROS_INFO_STREAM("Distance:" << min_dist<<"angle:" << angle);

	// visualize marker
	x_pos_rslidar = cos(angle)*(min_dist+1);
	y_pos_rslidar = sin(angle)*(min_dist+1);

	// transform coordinates
	tf::StampedTransform tf;
	tf::Vector3 pos_rslidar = tf::Vector3(x_pos_rslidar,y_pos_rslidar,0);
	tf::Vector3 pos_odom;

	tf = getTransform(tf, "/odom", "/rslidar");
	pos_odom = tf*pos_rslidar;
	visMarker(pos_odom);

	// Publish
	if(!emergency_stop){
		vel_message.angular.z = getGain(angle, "angular");
		vel_message.linear.x = getGain(min_dist, "linear");

		publisher_.publish(vel_message);
		ROS_INFO_STREAM("lin:" << vel_message.linear.x << "ang:" <<vel_message.angular.z);
	}
}

/* emergency stop*/
bool SmbHighlevelController::emergencyStop(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response){
	emergency_stop = request.data;

	if(emergency_stop){
		vel_message.angular.z = 0;
		vel_message.linear.x = 0;
		publisher_.publish(vel_message);
		response.message = "emergency flag set";
	} else {
		response.message = "emergency flag reset";
	}

	response.success = true;
}


} /* namespace */