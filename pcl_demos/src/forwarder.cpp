#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;
 
void callback(const pcl::PCLPointCloud2ConstPtr& cloud_blob) {
  pub.publish(cloud_blob);
}

int main(int argc, char** argv) {
  ros::init (argc, argv, "forwarder");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe ("input", 1, callback);
  pub = nh.advertise<pcl::PCLPointCloud2>("output", 1);
  ros::spin ();
}
