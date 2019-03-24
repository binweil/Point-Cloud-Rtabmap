
#include <Eigen/Eigen>
#include <pcl/io/pcd_io.h>
#include "pcl/PolygonMesh.h"
#include "pcl/common/centroid.h"
#include "pcl/common/transforms.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/surface/convex_hull.h"
#include "pcl/surface/mls.h"

using namespace std;

static void create_mesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PolygonMesh &mesh,int nThreads=8,
                           int setKsearch=10, int depth=10, float pointWeight=4.0,float samplePNode=1.5,
                           float scale=1.0,int isoDivide=5, bool confidence=true, bool outputPolygons=true,
                           bool manifold=true,int solverDivide=5){

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  std::cout << centroid << std::endl;

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << -centroid[0], -centroid[1], -centroid[2];

  std::cout << transform.matrix() << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTranslated(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*cloud, *cloudTranslated, transform);


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);


    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (10);

    seg.setInputCloud (cloudTranslated);
    seg.segment (*inliers, *coefficients);

    // Project the model inliers
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloudTranslated);
    proj.setIndices (inliers);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);

    // Create a Convex Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloud_projected);
    chull.reconstruct (*cloud_hull);

    std::cout << "Convex hull has: " << cloud_hull->points.size () << " data points." << std::endl;
    std::cout << "Convex hull volume:" << chull.getTotalVolume() << std::endl;

    pcl::PolygonMesh ms2;

    pcl::PCDWriter writer;
    writer.write ("/home/rflin/Desktop/BINWEI/catkin_ws/cloud_convex_hull.pcd", *cloud_hull, false);
   // pcl::io::savePolygonFilePLY("cloud_convex_hull2.ply", ms2, false);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointNormal>());

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (cloudTranslated);
  mls.setDilationIterations(10);
  mls.setPointDensity(30);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(5);

  // Reconstruct
  mls.process (*mls_points);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals (new pcl::PointCloud<pcl::PointNormal> ());
  pcl::concatenateFields (*cloudTranslated, *mls_points, *cloud_smoothed_normals);//x

  pcl::Poisson<pcl::PointNormal> poisson;

  poisson.setDepth(depth);//9
  poisson.setInputCloud(cloud_smoothed_normals);
  poisson.setPointWeight(pointWeight);//4
  poisson.setDegree(2);
  poisson.setSamplesPerNode(samplePNode);//1.5
  poisson.setScale(scale);//1.1
  poisson.setIsoDivide(isoDivide);//8
  poisson.setConfidence(confidence);
  poisson.setOutputPolygons(outputPolygons);
  poisson.setManifold(manifold);
  poisson.setSolverDivide(solverDivide);//8

  pcl::PolygonMesh mesh2;
  poisson.reconstruct(mesh2);

  pcl::surface::SimplificationRemoveUnusedVertices rem;
  rem.simplify(mesh2,mesh);

}

bool lamy_savePolygonFileSTL (const std::string &file_name,
                             const pcl::PolygonMesh& mesh,
                             const bool binary_format)
{
  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New ();

  pcl::io::mesh2vtk (mesh, poly_data);
  vtkSmartPointer<vtkSTLWriter> poly_writer = vtkSmartPointer<vtkSTLWriter>::New ();
  poly_writer->SetInputData (poly_data);

  if (binary_format)
    poly_writer->SetFileTypeToBinary ();
  else
    poly_writer->SetFileTypeToASCII ();

  poly_writer->SetFileName (file_name.c_str ());
  return (poly_writer->Write ());
}

 void createMeshFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,pcl::PolygonMesh& triangles){

     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
     pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

     Eigen::Vector4f centroid;

     //pcl::compute3DCentroid(*cloud, centroid);
     //std::cout << centroid << std::endl;

     Eigen::Affine3f transform = Eigen::Affine3f::Identity();
     transform.translation() << 0,0,0;

     //std::cout << transform.matrix() << std::endl;

     pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTranslated(new pcl::PointCloud<pcl::PointXYZ>());
     pcl::transformPointCloud(*cloud, *cloudTranslated, transform);

     tree->setInputCloud (cloudTranslated);
     n.setInputCloud (cloudTranslated);
     n.setSearchMethod (tree);
     n.setKSearch (20);        //It was 20
     n.compute (*normals);                //Normals are estimated using standard method.

     pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal> ());
     pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);//x

     // Create search tree*
     pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
     tree2->setInputCloud (cloud_with_normals);

     // Initialize objects
     pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

     std::cout << "Applying surface meshing..." <<std::endl;

     // Set the maximum distance between connected points (maximum edge length)
     gp3.setSearchRadius(0.025);           //It was 0.025

     // Set typical values for the parameters
     gp3.setMu (2.5); //It was 2.5
     gp3.setMaximumNearestNeighbors (100);    //It was 100
     gp3.setNormalConsistency(false); //It was false

     // Get result
     gp3.setInputCloud (cloud_with_normals);
     gp3.setSearchMethod (tree2);
     gp3.reconstruct (triangles);

     vtkSmartPointer<vtkPolyData> polydata= vtkSmartPointer<vtkPolyData>::New();

     pcl::PolygonMesh mms2;

     pcl::VTKUtils::convertToVTK(triangles,polydata);
     pcl::VTKUtils::convertToPCL(polydata,mms2);

     std::cout << "Saving Mesh to stl" <<std::endl;
     //pcl::io::savePolygonFilePLY("output_mesh.ply", mms2);
     //pcl::io::savePolygonFileSTL("output_mesh.stl", mms2);
     lamy_savePolygonFileSTL("/home/rflin/Desktop/BINWEI/catkin_ws/data/map_test.stl",mms2,true);
     std::cout << "Mesh Saved" <<std::endl;

 }

void vizualizeMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,pcl::PolygonMesh &mesh){

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("MAP3D MESH"));

int PORT1 = 0;
viewer->createViewPort(0.0, 0.0, 0.5, 1.0, PORT1);
viewer->setBackgroundColor (0, 0, 0, PORT1);
viewer->addText("ORIGINAL", 10, 10, "PORT1", PORT1);
viewer->addPointCloud(cloud,"original_cloud",PORT1);

int PORT2 = 0;
viewer->createViewPort(0.5, 0.0, 1.0, 1.0, PORT2);
viewer->setBackgroundColor (0, 0, 0, PORT2);
viewer->addText("MESH", 10, 10, "PORT2", PORT2);
viewer->addPolygonMesh(mesh,"mesh",PORT2);

viewer->setBackgroundColor (0, 0, 0);
viewer->addCoordinateSystem (1.0);
viewer->initCameraParameters ();
viewer->resetCamera();

while (!viewer->wasStopped ()){
    viewer->spin();
}
}

void cloudPointFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& filterCloud){

  std::cout << "Filtering point cloud..." << std::endl;
  std::cout << "Point cloud before filter:" << cloud->points.size()<< std::endl;

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier_removal;
  radius_outlier_removal.setInputCloud(cloud);
  radius_outlier_removal.setRadiusSearch(0.01);
  radius_outlier_removal.setMinNeighborsInRadius(1);
  radius_outlier_removal.filter(*filterCloud);

  std::cout << "Point cloud after filter:" << filterCloud->points.size() << std::endl;
}


void createOctomap(std::string pcdfilename)
{
  pcl::PointCloud<pcl::PointXYZRGBA> pointcloud;
  pcl::io::loadPCDFile<pcl::PointXYZRGBA> (pcdfilename, pointcloud );
  octomap::ColorOcTree tree(0.01);
  for (auto p:pointcloud.points)
  {
    tree.updateNode(octomap::point3d(p.x,p.y,p.z),true);
  }
  for (auto p:pointcloud.points)
  {
      tree.integrateNodeColor( p.x, p.y, p.z, p.r, p.g, p.b );
  }
  tree.updateInnerOccupancy();
  std::cout << "Saving Octomap to bt & ot" << std::endl;
  tree.write("/home/rflin/Desktop/BINWEI/catkin_ws/data/map_test.ot");
  tree.writeBinary("/home/rflin/Desktop/BINWEI/catkin_ws/data/map_test.bt");
  std::cout << "Octomap Saved" <<std::endl;
}

