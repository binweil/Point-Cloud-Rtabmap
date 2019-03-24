#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_msgs/PolygonMesh.h"
#include "pcl_ros/transforms.h"
#include "pcl/io/vtk_lib_io.h"
#include "pcl/PolygonMesh.h"
#include "pcl/geometry/polygon_mesh.h"
#include "pcl/surface/vtk_smoothing/vtk_utils.h"

using namespace std;
using namespace pcl;

void Callback(const sensor_msgs::PointCloud2Ptr& msg)
{
  pcl::PCDWriter w;
  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  pcl::fromROSMsg(*msg,cloud);
  cout<<"writing"<<endl;
  w.writeASCII("/home/lamy/Desktop/Kinect_Output.pcd",cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PointCould_writter");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/camera/depth/points", 1000, Callback);

  ros::spin();
}

