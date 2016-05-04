#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;
 
void callback(const pcl::PCLPointCloud2ConstPtr& cloud_blob) {

  pcl::PCLPointCloud2 cloud_filtered;

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  pub.publish(cloud_filtered);
}

int main(int argc, char** argv) {
  ros::init (argc, argv, "voxelgrid");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe ("input", 1, callback);
  pub = nh.advertise<pcl::PCLPointCloud2>("output", 1);
  ros::spin ();
}
