#include "rtabmap/core/OdometryF2M.h"
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/RtabmapThread.h"
#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/CameraStereo.h"
#include "rtabmap/core/CameraThread.h"
#include "rtabmap/core/OdometryThread.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/utilite/UEventsManager.h"
#include "pcl/io/pcd_io.h"
#include "pcl/io/ply_io.h"
#include "pcl/filters/filter.h"
#include "rtabmap/core/OctoMap.h"
#include "MapBuilder.h"

#include "pcl/common/common_headers.h"
#include "pcl/features/normal_3d.h"
#include "pcl/io/pcd_io.h"
#include "pcl/io/ply_io.h"
#include "pcl/io/vtk_io.h"
#include "pcl/io/io.h"
#include "pcl/io/vtk_lib_io.h"
#include "pcl/io/file_io.h"
#include "pcl/io/ply/ply_parser.h"
#include "pcl/io/ply/ply.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/console/print.h"
#include "pcl/console/parse.h"
#include "pcl/console/time.h"
#include "pcl/range_image/range_image.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/common/transforms.h"
#include "pcl/common/geometry.h"
#include "pcl/common/common.h"
#include "pcl/surface/vtk_smoothing/vtk_utils.h"
#include "pcl/surface/gp3.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/filters/crop_box.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/surface/convex_hull.h"
#include "pcl/io/vtk_lib_io.h"
#include "pcl/point_types.h"
#include "pcl/features/normal_3d.h"
//#include "pcl/features/gasd.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/surface/poisson.h"
#include "pcl/surface/mls.h"
#include "pcl/surface/simplification_remove_unused_vertices.h"
#include "pcl/filters/crop_hull.h"
#include "pcl/search/search.h"
#include "pcl/search/kdtree.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "octomap/octomap.h"
#include "octomap/ColorOcTree.h"
#include "octomap/OcTree.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include <octomap/AbstractOcTree.h>

#include <QApplication>
#include <stdio.h>
#include <assert.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <list>

#include "sqlite3.h"
#include "pcd2stl.cpp"

using namespace rtabmap;
using namespace std;
using namespace octomap;

std::string prefix = "/home/rflin/Desktop/BINWEI/catkin_ws";
float transformation[7];

static int sql_callback(void* data, int argc, char** argv, char** azColName)
{
    int i;
    for (i = 0; i < argc; i++) {
        transformation[i] = strtof(argv[i],NULL);
    }
    return 0;
}

void Create_Cubes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transform,float upperbound)
{
  for (size_t i = 0; i < 100; ++i)
  {
    cloud_transform->points[i].x = 1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].y = -1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].z = -0.2+0.1*rand()/(RAND_MAX + 1.0f);
  }
  for (size_t i = 100; i < 200; ++i)
  {
    cloud_transform->points[i].x = -1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].y = 1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].z = -0.2+0.1*rand()/(RAND_MAX + 1.0f);
  }
  for (size_t i = 200; i < 300; ++i)
  {
    cloud_transform->points[i].x = 1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].y = 1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].z = -0.2+0.1*rand()/(RAND_MAX + 1.0f);
  }
  for (size_t i = 300; i < 400; ++i)
  {
    cloud_transform->points[i].x = -1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].y = -1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].z = -0.2+0.1*rand()/(RAND_MAX + 1.0f);
  }
  for (size_t i = 400; i < 500; ++i)
  {
    cloud_transform->points[i].x = 1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].y = -1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].z = upperbound+0.1*rand()/(RAND_MAX + 1.0f);
  }
  for (size_t i = 500; i < 600; ++i)
  {
    cloud_transform->points[i].x = -1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].y = 1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].z = upperbound+0.1*rand()/(RAND_MAX + 1.0f);
  }
  for (size_t i = 600; i < 700; ++i)
  {
    cloud_transform->points[i].x = 1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].y = 1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].z = upperbound+0.1*rand()/(RAND_MAX + 1.0f);
  }
  for (size_t i = 700; i < 800; ++i)
  {
    cloud_transform->points[i].x = -1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].y = -1.2+0.1*rand()/(RAND_MAX + 1.0f);
    cloud_transform->points[i].z = upperbound+0.1*rand()/(RAND_MAX + 1.0f);
  }
  pcl::io::savePCDFile(prefix+"/data/Cloud_with_cube.pcd",*cloud_transform);
}

int main(int argc, char **argv){

  ULogger::setType(ULogger::kTypeConsole);
  ULogger::setLevel(ULogger::kWarning);

  // Here is the pipeline that we will use:
  // CameraOpenni -> "CameraEvent" -> OdometryThread -> "OdometryEvent" -> RtabmapThread -> "RtabmapEvent"

  // Create the OpenNI camera, it will send a CameraEvent at the rate specified.
  // Set transform to camera so z is up, y is left and x going forward
  Camera * camera = 0;
  Transform opticalRotation(0,0,1,0, -1,0,0,0, 0,-1,0,0);
  camera = new CameraFreenect(0, CameraFreenect::kTypeColorDepth, 0, opticalRotation);
  while(!camera->init())
  {
   UERROR("Camera init failed!");
   camera = new CameraFreenect(0, CameraFreenect::kTypeColorDepth, 0, opticalRotation);
   sleep(1);
  }

  CameraThread cameraThread(camera);
  // GUI stuff, there the handler will receive RtabmapEvent and construct the map
  // We give it the camera so the GUI can pause/resume the camera
  QApplication app(argc, argv);
  MapBuilder mapBuilder(&cameraThread);

  // Create an odometry thread to process camera events, it will send OdometryEvent.
  OdometryThread odomThread(new OdometryF2M());
  //OdometryThread odomThread(Odometry::create());

  //ParametersMap params;
  // Create RTAB-Map to process OdometryEvent
  Rtabmap * rtabmap = new Rtabmap();
  rtabmap->init();
  RtabmapThread rtabmapThread(rtabmap); // ownership is transfered

  // Setup handlers
  odomThread.registerToEventsManager();
  rtabmapThread.registerToEventsManager();
  mapBuilder.registerToEventsManager();

  // The RTAB-Map is subscribed by default to CameraEvent, but we want
  // RTAB-Map to process OdometryEvent instead, ignoring the CameraEvent.
  // We can do that by creating a "pipe" between the camera and odometry, then
  // only the odometry will receive CameraEvent from that camera. RTAB-Map is
  // also subscribed to OdometryEvent by default, so no need to create a pipe between
  // odometry and RTAB-Map.
  UEventsManager::createPipe(&cameraThread, &odomThread, "CameraEvent");

  // Let's start the threads
  rtabmapThread.start();
  odomThread.start();
  cameraThread.start();

  mapBuilder.show();

  //app.exec(); // main loop
  app.exec();

  // Save 3D map
  //printf("Saving rtabmap_cloud.pcd...\n");
  std::map<int, Signature> nodes;
  std::map<int, Transform> optimizedPoses;
  std::multimap<int, Link> links;
  rtabmap->get3DMap(nodes, optimizedPoses, links, false, true);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for(std::map<int, Transform>::iterator iter=optimizedPoses.begin(); iter!=optimizedPoses.end(); ++iter)
  {
    Signature node = nodes.find(iter->first)->second;
   // uncompress data
   node.sensorData().uncompressData();
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = util3d::cloudRGBFromSensorData(
       node.sensorData(),
       4,           // image decimation before creating the clouds
       4.0f,        // maximum depth of the cloud
       0.0f);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpNoNaN(new pcl::PointCloud<pcl::PointXYZRGB>);
   std::vector<int> index;
   pcl::removeNaNFromPointCloud(*tmp, *tmpNoNaN, index);



   if(!tmpNoNaN->empty())
   {
     *cloud += *util3d::transformPointCloud(tmpNoNaN, iter->second); // transform the point cloud to its pose
   }
  }
  if(cloud->size())
  {
   printf("Voxel grid filtering of the assembled cloud (voxel=%f, %d points)\n", 0.01f, (int)cloud->size());
   cloud = util3d::voxelize(cloud, 0.01f);

   pcl::io::savePCDFile(prefix+"/data/kinect_original.pcd", *cloud);
   printf("Saving kinect_original.pcd... done! (%d points)\n", (int)cloud->size());

  }
  else
  {
   printf("Saving kinect_original.pcd... failed! The cloud is empty.\n");
  }

  // Save trajectory
  printf("Saving rtabmap_trajectory.txt ...\n");
  if(optimizedPoses.size() && graph::exportPoses(prefix+"/data/rtabmap_trajectory.txt", 0, optimizedPoses, links))
  {
   printf("Saving rtabmap_trajectory.txt... done!\n");
  }
  else
  {
   printf("Saving rtabmap_trajectory.txt... failed!\n");
  }

  // remove handlers
  mapBuilder.unregisterFromEventsManager();
  rtabmapThread.unregisterFromEventsManager();
  odomThread.unregisterFromEventsManager();

  // Kill all threads
  cameraThread.kill();
  odomThread.join(true);
  rtabmapThread.join(true);

  rtabmap->close(false);

  /**********************************************************************************************************/
  // Convert Pointcloud:cloud to mesh and save as stl
  pcl::console::print_color(stdout,10,pcl::console::TT_YELLOW,"Processing Point Cloud\n");

  cloud->width = (int) cloud->points.size ();
  cloud->height = 1;
  cloud->is_dense = true;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(*cloud,*cloud_xyz);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_filtered (new pcl::PointCloud<pcl::PointXYZ>());
  cloudPointFilter(cloud_xyz,cloud_xyz_filtered);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transform (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(*cloud_xyz_filtered,*cloud_transform);

  //Apply Transformation
  Eigen::Vector3f kinect_self_translate(0,0,0);
  Eigen::Quaternionf kinect_self_rotate1(0.5,-0.5,-0.5,-0.5);
  pcl::transformPointCloud(*cloud_transform,*cloud_transform,kinect_self_translate,kinect_self_rotate1);
  Eigen::Quaternionf kinect_self_rotate2(0,0,0,1);
  pcl::transformPointCloud(*cloud_transform,*cloud_transform,kinect_self_translate,kinect_self_rotate2);
  sqlite3 *database;

  std::string database_path = "/data/transform_matrix.db";
  sqlite3_open((prefix+database_path).c_str(),&database);
  sqlite3_exec(database,"SELECT * FROM translation",sql_callback,NULL,NULL);
  sqlite3_close(database);
  Eigen::Vector3f translation(transformation[0],transformation[1],transformation[2]);
  Eigen::Quaternionf quat(transformation[3],transformation[4],transformation[5],transformation[6]);
  pcl::transformPointCloud(*cloud_transform,*cloud_transform,translation,quat);

  const float xmin = 0.2;
  const float ymin = 0.2;
  //Add Eight cubes
  for (size_t j=0; j<cloud_transform->points.size();++j)
  {
    if ((cloud_transform->points[j].x<xmin) && (cloud_transform->points[j].y<ymin))
    {
      cloud_transform->points[j].x = 0;
      cloud_transform->points[j].y = 0;
      cloud_transform->points[j].z = -1;
    }
  }

  Create_Cubes(cloud_transform,2.0);

  pcl::PolygonMesh cloud_mesh;
  createMeshFromCloud(cloud_transform,cloud_mesh); //create mesh and save
  createOctomap(prefix+"/data/Cloud_with_cube.pcd");
  vizualizeMesh(cloud_transform,cloud_mesh);
  return 0;
}
