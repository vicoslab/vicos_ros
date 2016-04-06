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

typedef struct DepthSnapshot {

	image_geometry::PinholeCameraModel camera;
	sensor_msgs::ImageConstPtr depth;

} DepthSnapshot;

const std::string stripSlashTF2(const std::string& str) {

	if (str.find("/") == 0) {
		return std::string(str, 1, std::string::npos);
	} else
		return str;

}

class LocalizerNodelet : public nodelet::Nodelet {
private:
	int queue_size;

	// Subscriptions
	message_filters::Subscriber<sensor_msgs::Image> sub_depth_image;
	message_filters::Subscriber<sensor_msgs::CameraInfo> sub_depth_info;
	boost::shared_ptr<tf2_ros::Buffer> tf_buffer;
	boost::shared_ptr<tf2_ros::TransformListener> tf_listener;
	typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
	typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
	boost::shared_ptr<Synchronizer> synchronized_listener;

	ros::ServiceServer localize_service;
	boost::mutex request_mutex;
	std::vector<DepthSnapshot> snapshot_buffer;

	template<typename T>
	bool estimate(const sensor_msgs::ImageConstPtr& depth_msg, const image_geometry::PinholeCameraModel& camera_model, const geometry_msgs::Point& point, int radius, const Eigen::Affine3d& target_to_depth, geometry_msgs::Pose& pose);

public:
	virtual void onInit();

	void imageCallback(const sensor_msgs::ImageConstPtr& depth_image_msg,
		       const sensor_msgs::CameraInfoConstPtr& depth_info_msg);

	bool localize(localizer::Localize::Request &req, localizer::Localize::Response &res);

};

bool LocalizerNodelet::localize(localizer::Localize::Request &req, localizer::Localize::Response &res) {

	boost::lock_guard<boost::mutex> lock(request_mutex);

	res.pose.position.x = 0;
	res.pose.position.y = 0;
	res.pose.position.z = 0;

	res.pose.orientation.x = 0;
	res.pose.orientation.y = 0;
	res.pose.orientation.z = 1;
	res.pose.orientation.w = 0;

	unsigned int best, i;
	double best_time = 1000;
	for (i = 0; i < snapshot_buffer.size(); i++) {
		double timediff = std::abs(req.header.stamp.toSec() - snapshot_buffer[i].depth->header.stamp.toSec());
		if (timediff < best_time) {
			best = i;
			best_time = timediff;
		}
	}

	if (best_time > 1) return true;

	NODELET_INFO_THROTTLE(2, "Selected depth image time difference: %fs (buffer size: %ld)", best_time, snapshot_buffer.size());

	DepthSnapshot snapshot = snapshot_buffer[best];

	Eigen::Affine3d target_to_depth;
	try
	{
		geometry_msgs::TransformStamped transform = tf_buffer->lookupTransform (
				              stripSlashTF2(snapshot.depth->header.frame_id), stripSlashTF2(req.header.frame_id),
				              snapshot.depth->header.stamp);

		tf::transformMsgToEigen(transform.transform, target_to_depth);
	}
	catch (tf2::TransformException& ex)
	{
		NODELET_WARN_THROTTLE(2, "TF2 exception:\n%s", ex.what());
		return false;
	}

	if (snapshot.depth->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
		estimate<uint16_t>(snapshot.depth, snapshot.camera, req.point, req.scope, target_to_depth, res.pose);
	}
	else if (snapshot.depth->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
		estimate<float>(snapshot.depth, snapshot.camera, req.point, req.scope, target_to_depth, res.pose);
	}
	else {
		NODELET_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", snapshot.depth->encoding.c_str());
		return false;
	}

	return true;

}


void LocalizerNodelet::onInit()
{

	ros::NodeHandle& nh = getNodeHandle();
	ros::NodeHandle& private_nh = getPrivateNodeHandle();
	tf_buffer.reset( new tf2_ros::Buffer );
	tf_listener.reset( new tf2_ros::TransformListener(*tf_buffer) );

	snapshot_buffer.clear();

	// Read parameters.
	private_nh.param("queue_size", queue_size, 20);

	sub_depth_image.subscribe(nh, "depth/image", 1);
	sub_depth_info.subscribe(nh, "depth/camera_info", 1);

	synchronized_listener.reset( new Synchronizer(SyncPolicy(queue_size), sub_depth_image, sub_depth_info) );
	synchronized_listener->registerCallback(boost::bind(&LocalizerNodelet::imageCallback, this, _1, _2));

	std::string service_name = ros::this_node::getName() + std::string("/localize");
	localize_service = nh.advertiseService<LocalizerNodelet, localizer::Localize::Request, localizer::Localize::Response>(service_name, &LocalizerNodelet::localize, this);

}

void LocalizerNodelet::imageCallback(const sensor_msgs::ImageConstPtr& depth_image_msg,
                              const sensor_msgs::CameraInfoConstPtr& depth_info_msg) {

	boost::lock_guard<boost::mutex> lock(request_mutex);
	if (!depth_image_msg) return;

	DepthSnapshot snapshot;

	snapshot.camera.fromCameraInfo(depth_info_msg);
	snapshot.depth = depth_image_msg;

	snapshot_buffer.push_back(snapshot);

	if (snapshot_buffer.size() > queue_size) {
		snapshot_buffer.erase(snapshot_buffer.begin());
	}
}

template<typename T>
bool LocalizerNodelet::estimate(const sensor_msgs::ImageConstPtr& depth_msg, const image_geometry::PinholeCameraModel& camera_model, const geometry_msgs::Point& point, int scope, const Eigen::Affine3d& target_to_depth, geometry_msgs::Pose& pose) {

	// Extract all the parameters we need
	double depth_fx = camera_model.fx();
	double depth_fy = camera_model.fy();
	double depth_cx = camera_model.cx(), depth_cy = camera_model.cy();
	double depth_Tx = camera_model.Tx(), depth_Ty = camera_model.Ty();    

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
		scope = MIN(scope, 30); // Some reasonable bounds

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

	xyz_depth << ((u - depth_cx) * depth - depth_Tx) / depth_fx,
		 ((v - depth_cy) * depth - depth_Ty) / depth_fy, depth, 1;

	// Transform to target camera frame
	xyz_target = target_to_depth.inverse() * xyz_depth;

	pose.position.x = xyz_target.x();
	pose.position.y = xyz_target.y();
	pose.position.z = xyz_target.z();

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

