#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <iostream>
#include <vector>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <nodelet/nodelet.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <localizer/Localize.h>

namespace localizer {

using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;

// Encapsulate differences between processing float and uint16_t depths
template<typename T> struct DepthTraits {};

template<>
struct DepthTraits<uint16_t>
{
	static inline bool valid(uint16_t depth) { return depth != 0; }
	static inline float toMeters(uint16_t depth) { return depth * 0.001f; } // originally mm
	static inline uint16_t fromMeters(float depth) { return (depth * 1000.0f) + 0.5f; }
	static inline void initializeBuffer(std::vector<uint8_t>& buffer) {} // Do nothing - already zero-filled
};

template<>
struct DepthTraits<float>
{
	static inline bool valid(float depth) { return std::isfinite(depth); }
	static inline float toMeters(float depth) { return depth; }
	static inline float fromMeters(float depth) { return depth; }

	static inline void initializeBuffer(std::vector<uint8_t>& buffer)
	{
		float* start = reinterpret_cast<float*>(&buffer[0]);
		float* end = reinterpret_cast<float*>(&buffer[0] + buffer.size());
		std::fill(start, end, std::numeric_limits<float>::quiet_NaN());
	}
};

const std::string stripSlashTF2(const std::string& str) {

	if (str.find("/") == 0) {
		return std::string(str, 1, std::string::npos);
	} else
		return str;

}

class LocalizerNodelet : public nodelet::Nodelet {

	// Subscriptions
	message_filters::Subscriber<sensor_msgs::Image> sub_depth_image_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> sub_depth_info_;
	boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	boost::shared_ptr<tf2_ros::TransformListener> tf_;
	typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
	typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
	boost::shared_ptr<Synchronizer> sync_;

	ros::ServiceServer service;

	// Publications
	boost::mutex request_mutex_;

	image_geometry::PinholeCameraModel depth_model_;

	sensor_msgs::ImageConstPtr saved_depth_msg;

	virtual void onInit();

	void imageCallback(const sensor_msgs::ImageConstPtr& depth_image_msg,
		       const sensor_msgs::CameraInfoConstPtr& depth_info_msg);

	bool localize(localizer::Localize::Request &req, localizer::Localize::Response &res);

	template<typename T>
	bool estimate(const sensor_msgs::ImageConstPtr& depth_msg, const geometry_msgs::Point& point, int radius, const Eigen::Affine3d& target_to_depth, geometry_msgs::Pose& pose);
};

bool LocalizerNodelet::localize(localizer::Localize::Request &req, localizer::Localize::Response &res) {

	boost::lock_guard<boost::mutex> lock(request_mutex_);
	
	if (!saved_depth_msg) return false;

	Eigen::Affine3d target_to_depth;
	try
	{
		geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform (
				              stripSlashTF2(saved_depth_msg->header.frame_id), stripSlashTF2(req.header.frame_id),
				              saved_depth_msg->header.stamp);

		tf::transformMsgToEigen(transform.transform, target_to_depth);
	}
	catch (tf2::TransformException& ex)
	{
		NODELET_WARN_THROTTLE(2, "TF2 exception:\n%s", ex.what());
		return false;
	}

	if (saved_depth_msg->encoding == enc::TYPE_16UC1) {
		estimate<uint16_t>(saved_depth_msg, req.point, req.scope, target_to_depth, res.pose);
	}
	else if (saved_depth_msg->encoding == enc::TYPE_32FC1) {
		estimate<float>(saved_depth_msg, req.point, req.scope, target_to_depth, res.pose);
	}
	else {
		NODELET_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", saved_depth_msg->encoding.c_str());
		return false;
	}

	return true;

}


void LocalizerNodelet::onInit()
{

	ros::NodeHandle& nh = getNodeHandle();
	ros::NodeHandle& private_nh = getPrivateNodeHandle();
	//nh_depth_.reset( new ros::NodeHandle(nh, "depth") );
	//it_depth_.reset( new image_transport::ImageTransport(*nh_depth_) );
	tf_buffer_.reset( new tf2_ros::Buffer );
	tf_.reset( new tf2_ros::TransformListener(*tf_buffer_) );

	saved_depth_msg.reset();

	// Read parameters.
	int queue_size;
	private_nh.param("queue_size", queue_size, 10);

	//image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_depth_image_.subscribe(nh, "/camera/depth_registered/image_raw", 1);
  	sub_depth_info_.subscribe(nh, "/camera/depth_registered/camera_info", 1);

	// Synchronize inputs.
	sync_.reset( new Synchronizer(SyncPolicy(queue_size), sub_depth_image_, sub_depth_info_) );
	sync_->registerCallback(boost::bind(&LocalizerNodelet::imageCallback, this, _1, _2));

	service = nh.advertiseService<LocalizerNodelet, localizer::Localize::Request, localizer::Localize::Response>("localize", &LocalizerNodelet::localize, this);

}

void LocalizerNodelet::imageCallback(const sensor_msgs::ImageConstPtr& depth_image_msg,
                              const sensor_msgs::CameraInfoConstPtr& depth_info_msg) {

	boost::lock_guard<boost::mutex> lock(request_mutex_);
	
	depth_model_.fromCameraInfo(depth_info_msg);
	saved_depth_msg = depth_image_msg;

}

template<typename T>
bool LocalizerNodelet::estimate(const sensor_msgs::ImageConstPtr& depth_msg, const geometry_msgs::Point& point, int scope, const Eigen::Affine3d& target_to_depth, geometry_msgs::Pose& pose) {

	// Extract all the parameters we need
	double depth_fx = depth_model_.fx();
	double depth_fy = depth_model_.fy();
	double depth_cx = depth_model_.cx(), depth_cy = depth_model_.cy();
	double depth_Tx = depth_model_.Tx(), depth_Ty = depth_model_.Ty();    

	Eigen::Vector4d xyz_target;
	xyz_target << point.x, point.y, point.z, 1;

	// Transform to depth camera frame
	Eigen::Vector4d xyz_depth = target_to_depth * xyz_target;

	// Project to (u,v) in depth image
	double inv_Z = 1.0 / xyz_depth.z();
	int u = (depth_fx * xyz_depth.x() + depth_Tx) * inv_Z + depth_cx + 0.5;
	int v = (depth_fy * xyz_depth.y() + depth_Ty) * inv_Z + depth_cy + 0.5;

    if (u < 0 || u >= (int)depth_msg->width || v < 0 || v >= (int)depth_msg->height)
        return false;
    
	const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
	int row_step = depth_msg->step / sizeof(T);

	double depth = 1;

	// in case we need a more robust estimate
	if (scope > 1) {
		scope = MIN(scope, 30);

		std::vector<double> values;

		for (int i = u - scope; i < u + scope; i++) 
		for (int j = v - scope; j < v + scope; j++) {
			if (i < 0 || i >= (int)depth_msg->width || j < 0 || j >= (int)depth_msg->height)
				continue;

			T raw_depth = depth_row[i + row_step * j];
			if (!DepthTraits<T>::valid(raw_depth))
				continue;

			values.push_back(DepthTraits<T>::toMeters(raw_depth));

		} 

		if (values.size() == 0) return false;
		if (values.size() == 1) {
			depth = values[0];
		}
		
		// Computing a median of values
		std::sort(values.begin(), values.end());

		if (values.size() % 2 == 0)
			depth = (values[values.size() / 2 - 1] + values[values.size() / 2]) / 2;
		else
			depth = values[values.size() / 2];

	} else {

		T raw_depth = depth_row[u + row_step * v];
		if (!DepthTraits<T>::valid(raw_depth))
			return false;

		depth = DepthTraits<T>::toMeters(raw_depth);

	}


	pose.position.x = ((u - depth_cx) * depth - depth_Tx) / depth_fx;
	pose.position.y = ((v - depth_cy) * depth - depth_Ty) / depth_fy;
	pose.position.z = depth;

	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 1;
	pose.orientation.w = 0;

	return true;

}

} 

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(localizer::LocalizerNodelet,nodelet::Nodelet);

