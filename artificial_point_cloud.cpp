/*
 * artificial_point_cloud.cpp
 *
 *  Created on: Mar 20, 2014
 *      Author: dbazazian
 */

 //#define CUBE_POINT_CLOUD_SIMPLE // Simple cube Point cloud
//#define CUBE_POINT_CLOUD
//#define ELLIPSE_AND_Cylinder
//#define CYLINDER_B_W
//#define SPHER_VTK
//#define SPHER_PCL
// #define SPHER_MULTIPLE
//#define SPHER_BW
//#define UNIFORM_SAMPLING_SPHERE
//#define UNIFORM_SAMPLING_ELLIPSOID
//#define PARABOLOIDE_AND_CONE
//#define PARABOLOIDE_B_W
//#define SADDLE_HOURSE_FIRST
//#define SADDLE_HOURSE_SECOND
//#define HYPERBOLIC_PARABOLOID
//#define SHARP_EDGE
// #define CUBE_FOUR_EGDE
// #define CUBE_CYLINDER
// #define CUBE_WITHOUT_CYLINDER
// #define WEDGE_SHARP
// #define WEDGE_CYLINDER_NONSYM
//  #define WEDGE_SYM
//#define SacModel
// #define STAIRS_THREE_CUBE
// #define FRACTAL_SEAT
// #define FRACTAL_THREE_CUBE
// #define FRACTAL_FOUR_HOLE
// #define TETRAHEDRON
 // #define TETRAHEDRON_PARTITE
     #define TETRAHEDRON_MULTIPLE
 // #define TWO_PLANE
// #define INTERSECTION_THREEPLANE_DIFFERENTSIZES
// #define INTERSECTION_TWOPLANE    // Three Planes
// #define ONE_PLANE
//  #define ADD_NOISE

#ifdef PARABOLOIDE_AND_CONE

#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

int
main (int argc, char** argv)
{


  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "Genarating example point clouds.\n\n";
  // Creat an spher extruded along the z-axis. The colour for the XYZRGB cloud will gradually go from red to green to blue.
  uint8_t r(255), g(15), b(15);
  for (float z(-2.0); z <= 2.0; z += 0.0009)
  {
    for (float theta(0.0); theta <= 360.0; theta += 4.0)
    {
       // for (float phi(0.0); phi <= 180.0; phi += 4.0)
        //{
      pcl::PointXYZ basic_point;

      basic_point.x = z * cosf (pcl::deg2rad(theta));
      basic_point.y = z * sinf (pcl::deg2rad(theta));
      basic_point.z = z * z ; // Paraboloide
      // basic_point.z = z ;  // Cone

      basic_cloud_ptr->points.push_back(basic_point);

      pcl::PointXYZRGB point;
      point.x = basic_point.x;
      point.y = basic_point.y;
      point.z = basic_point.z;
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
       //}
    }
    if (z < 0.0)
    {
      r -= 12;
      g += 12;
    }
    else
    {
      g -= 12;
      b += 12;
    }
  }
  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;
  point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
  point_cloud_ptr->height = 1;

  // write point cloud to disk
  //pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/Cone.pcd", *point_cloud_ptr);
  pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/Paraboloid.pcd", *point_cloud_ptr);
   // Show the cloud
  //pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/Cone.pcd", *point_cloud_ptr);
  pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/Paraboloid.pcd", *point_cloud_ptr);
  // pcl::visualization::CloudViewer viewer("Cone");
    pcl::visualization::CloudViewer viewer("Paraboloide");

viewer.showCloud(point_cloud_ptr);
while (!viewer.wasStopped ())
{}

  return 0;

}

#endif

#ifdef PARABOLOIDE_B_W

#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  for (float zz(0.0000); zz <= 1.0000; zz += 0.025)
  {
	  float DeltaZ = 0.025;
	  float r = sqrt (zz);
	    float DeltaTheta = 360/((2*3.1415*r)/DeltaZ);

    for (float theta(0.0000) ; theta < 360 ; theta += DeltaTheta)
    {
       // for (float phi(0.0); phi <= 180.0; phi += 4.0)
        //{
      pcl::PointXYZ basic_point;
      basic_point.x = zz * cosf (pcl::deg2rad(theta));   // cone
      //basic_point.x = r * cosf (pcl::deg2rad(theta));   // Parabolide
      basic_point.y = zz * sinf (pcl::deg2rad(theta));     // Cone
     // basic_point.y = r * sinf (pcl::deg2rad(theta));    // parabolide
         basic_point.z = zz ;  // Cone
      //basic_point.z = zz*zz ; // Paraboloide


      cloud->points.push_back(basic_point);
       }
     }

  for (float zz(-1.0000); zz < 0.0000; zz += 0.025)
  {
	  float DeltaZ = 0.025;
	   float r = sqrt (abs(zz));
	    float DeltaTheta = 360/((2*3.1415*r)/DeltaZ);

    for (float theta(0.0000) ; theta < 360 ; theta += DeltaTheta)
    {
       // for (float phi(0.0); phi <= 180.0; phi += 4.0)
        //{
      pcl::PointXYZ basic2_point;
      basic2_point.x = zz* cosf (pcl::deg2rad(theta));   // Cone
      //basic_point.x = r * cosf (pcl::deg2rad(theta));   // Parabolide
      basic2_point.y = zz* sinf (pcl::deg2rad(theta));   // Cone
      // basic_point.y = r * sinf (pcl::deg2rad(theta));    // parabolide
      basic2_point.z = zz ;  // Cone
      // basic2_point.z =- (zz*zz) ; // Paraboloide


      cloud->points.push_back(basic2_point);
       }
     }

  cloud->width = (int) cloud->points.size ();
  cloud->height = 1;

  // write point cloud to disk
 //pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/Parabolid2.pcd", *cloud);
 pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/Cone2.pcd", *cloud);
 std::cout<< "Number of points in the cloud is:" << cloud ->points.size ()<< std::endl;
 //Write PLY
 pcl::PLYWriter writePLY;
 //writePLY.write ("/Path/TO/ArtificialPointClouds/Parabolid2ply.ply", *cloud,  false);
 writePLY.write ("/Path/TO/ArtificialPointClouds/Cone2ply.ply", *cloud,  false);

   // Show the cloud
 //pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/CylinderBW.pcd", *cloud);
 pcl::visualization::CloudViewer viewer("Parabolid");
 viewer.showCloud(cloud);
while (!viewer.wasStopped ())
{}

  return 0;

}

#endif

#ifdef SPHER_PCL

#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

int
main (int argc, char** argv)
{


  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "Genarating example point clouds.\n\n";
  // Creat an spher extruded along the z-axis. The colour for the XYZRGB cloud will gradually go from red to green to blue.
  uint8_t r(255), g(15), b(15);
  for (float z(-1.0); z <= 1.0; z += 0.025)
  {
    for (float theta(0.0); theta <= 360.0; theta += 5.0)
    {
        for (float phi(0.0); phi <= 180.0; phi += 5.0)
        {
      pcl::PointXYZ basic_point;

      basic_point.x = 2* cosf (pcl::deg2rad(theta))* sinf (pcl::deg2rad(phi));
      basic_point.y = 2* sinf (pcl::deg2rad(theta))* sinf (pcl::deg2rad(phi));
      basic_point.z = 2* cosf (pcl::deg2rad(phi));
      basic_cloud_ptr->points.push_back(basic_point);

      pcl::PointXYZRGB point;
      point.x = basic_point.x;
      point.y = basic_point.y;
      point.z = basic_point.z;
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
       }
    }
    if (z < 0.0)
    {
      r -= 12;
      g += 12;
    }
    else
    {
      g -= 12;
      b += 12;
    }
  }
  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;
  point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
  point_cloud_ptr->height = 1;

  std::cout<< "Number of points in the input cloud is:" << point_cloud_ptr ->points.size ()<< std::endl;

  // write point cloud to disk
pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/Spher.pcd", *point_cloud_ptr);
// Show the cloud
pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/Spher.pcd", *point_cloud_ptr);

  pcl::visualization::CloudViewer viewer(" Spher");
  viewer.showCloud(point_cloud_ptr);
  while (!viewer.wasStopped ())
  {}

  return 0;

}

#endif

#ifdef SPHER_MULTIPLE

#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char** argv)
{



  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);


  float rr=1.00;
  float* theta = new float [1500];
  float* phi = new float [1500];
  srand ( time(NULL) );

  for (int j=0 ; j < 1500; j++)
{
	      phi[j] = float (rand() )* 360 / float (RAND_MAX);
	      theta[j] = float (rand() )* 180 / float (RAND_MAX);

      pcl::PointXYZRGB basic_point1;

      basic_point1.x =  rr * ( sinf (pcl::deg2rad(phi[j]))* cosf (pcl::deg2rad(theta[j])));
      basic_point1.y = rr  * (sinf (pcl::deg2rad(phi[j]))* sinf (pcl::deg2rad(theta[j])));
      basic_point1.z = rr  * cosf (pcl::deg2rad(phi[j]));
      basic_point1.r= 255;
      basic_point1.g= 0;
      basic_point1.b= 0;
      cloud->points.push_back(basic_point1);
        }
  delete[] phi;
  delete[] theta;
std:cout<< "cloud for the first sphere has been created " << std::endl;

  float rr2=0.50;
  float* theta2 = new float [800];
  float* phi2 = new float [800];
  srand ( time(NULL) );

  for (int j=0 ; j < 750; j++)
{
	      phi2[j] = float (rand() )* 360 / float (RAND_MAX);
	      theta2[j] = float (rand() )* 180 / float (RAND_MAX);

      pcl::PointXYZRGB basic_point2;

      basic_point2.x =  1+ (   rr2 * ( sinf (pcl::deg2rad(phi2[j]))* cosf (pcl::deg2rad(theta2[j])))    );
      basic_point2.y =  (  rr2  * (sinf (pcl::deg2rad(phi2[j]))* sinf (pcl::deg2rad(theta2[j])))   );
      basic_point2.z = 1+ (  rr2  * cosf (pcl::deg2rad(phi2[j]))   ) ;
      basic_point2.r= 0;
      basic_point2.g= 255;
      basic_point2.b= 0;
      cloud->points.push_back(basic_point2);
        }
  delete[] phi2;
  delete[] theta2;



float rr3=0.70;
float* theta3 = new float [1000];
float* phi3 = new float [1000];
srand ( time(NULL) );

for (int j=0 ; j < 1000; j++)
{
	      phi3[j] = float (rand() )* 360 / float (RAND_MAX);
	      theta3[j] = float (rand() )* 180 / float (RAND_MAX);

    pcl::PointXYZRGB basic_point3;

    basic_point3.x =   (   rr3 * ( sinf (pcl::deg2rad(phi3[j]))* cosf (pcl::deg2rad(theta3[j])))    );
    basic_point3.y = 1+ (  rr3  * (sinf (pcl::deg2rad(phi3[j]))* sinf (pcl::deg2rad(theta3[j])))   );
    basic_point3.z = 1+ (  rr3  * cosf (pcl::deg2rad(phi3[j]))   ) ;
    basic_point3.r= 0;
    basic_point3.g= 0;
    basic_point3.b= 255;
    cloud->points.push_back(basic_point3);
      }
delete[] phi3;
delete[] theta3;

float rr4=0.25;
float* theta4 = new float [800];
float* phi4 = new float [800];
srand ( time(NULL) );

for (int j=0 ; j < 500; j++)
{
	      phi4[j] = float (rand() )* 360 / float (RAND_MAX);
	      theta4[j] = float (rand() )* 180 / float (RAND_MAX);

    pcl::PointXYZRGB basic_point4;

    basic_point4.x =  0.75+ (   rr4 * ( sinf (pcl::deg2rad(phi4[j]))* cosf (pcl::deg2rad(theta4[j])))    );
    basic_point4.y = 0.75+ (  rr4  * (sinf (pcl::deg2rad(phi4[j]))* sinf (pcl::deg2rad(theta4[j])))   );
    basic_point4.z =  (  rr4  * cosf (pcl::deg2rad(phi4[j]))   ) ;
    basic_point4.r= 255;
    basic_point4.g= 255;
    basic_point4.b= 0;
    cloud->points.push_back(basic_point4);
      }
delete[] phi4;
delete[] theta4;



float rr5=0.50;
 float* theta5 = new float [800];
 float* phi5 = new float [800];
 srand ( time(NULL) );

 for (int j=0 ; j < 750; j++)
{
	      phi5[j] = float (rand() )* 360 / float (RAND_MAX);
	      theta5[j] = float (rand() )* 180 / float (RAND_MAX);

     pcl::PointXYZRGB basic_point5;

     basic_point5.x =  -1+ (   rr5 * ( sinf (pcl::deg2rad(phi5[j]))* cosf (pcl::deg2rad(theta5[j])))    );
     basic_point5.y =  (  rr5  * (sinf (pcl::deg2rad(phi5[j]))* sinf (pcl::deg2rad(theta5[j])))   );
     basic_point5.z = -1+ (  rr5  * cosf (pcl::deg2rad(phi5[j]))   ) ;
     basic_point5.r= 255;
     basic_point5.g= 0;
     basic_point5.b= 255;
     cloud->points.push_back(basic_point5);
       }
 delete[] phi5;
 delete[] theta5;



float rr6=0.25;
float* theta6 = new float [800];
float* phi6 = new float [800];
srand ( time(NULL) );

for (int j=0 ; j < 500; j++)
{
	      phi6[j] = float (rand() )* 360 / float (RAND_MAX);
	      theta6[j] = float (rand() )* 180 / float (RAND_MAX);

   pcl::PointXYZRGB basic_point6;

   basic_point6.x =   (   rr6 * ( sinf (pcl::deg2rad(phi6[j]))* cosf (pcl::deg2rad(theta6[j])))    );
   basic_point6.y = -0.75+ (  rr6  * (sinf (pcl::deg2rad(phi6[j]))* sinf (pcl::deg2rad(theta6[j])))   );
   basic_point6.z = -0.75+ (  rr6  * cosf (pcl::deg2rad(phi6[j]))   ) ;
   basic_point6.r= 0;
   basic_point6.g= 255;
   basic_point6.b= 255;
   cloud->points.push_back(basic_point6);
     }
delete[] phi6;
delete[] theta6;

float rr7=0.70;
float* theta7 = new float [1000];
float* phi7 = new float [1000];
srand ( time(NULL) );

for (int j=0 ; j < 1000; j++)
{
	      phi7[j] = float (rand() )* 360 / float (RAND_MAX);
	      theta7[j] = float (rand() )* 180 / float (RAND_MAX);

   pcl::PointXYZRGB basic_point7;

   basic_point7.x =  -1+ (   rr7 * ( sinf (pcl::deg2rad(phi7[j]))* cosf (pcl::deg2rad(theta7[j])))    );
   basic_point7.y = -1+ (  rr7  * (sinf (pcl::deg2rad(phi7[j]))* sinf (pcl::deg2rad(theta7[j])))   );
   basic_point7.z =  (  rr7  * cosf (pcl::deg2rad(phi7[j]))   ) ;
   basic_point7.r= 128;
   basic_point7.g= 128;
   basic_point7.b= 0;
   cloud->points.push_back(basic_point7);
     }
delete[] phi7;
delete[] theta7;






      cloud->width = (int) cloud->points.size ();
      cloud->height = 1;


  std::cout<< "Number of points in the input cloud is:" << cloud->points.size ()<< std::endl;

  // write point cloud to disk
pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/SpherMultiple.pcd", *cloud);

pcl::PLYWriter writePLY;
writePLY.write ("/Path/TO/ArtificialPointClouds/SpherMultiple.ply", *cloud,  false);

// Show the cloud
pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/SpherMultiple.pcd", *cloud);

  pcl::visualization::CloudViewer viewer(" Spher");
  viewer.showCloud(cloud);
  while (!viewer.wasStopped ())
  {}

  return 0;

}

#endif


#ifdef SPHER_BW
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


 //  for (float z(-1.0); z <= 1.0; z += 0.3)       // Sphere Radius 1
  for (float z(-2.0); z <= 2.0; z += 0.3)       // Sphere Radius 2
  {
    for (float theta(0.0); theta <= 360.0; theta += 5.0)
    {
        for (float phi(0.0); phi <= 180.0; phi += 5.0)
        {
      pcl::PointXYZ basic_point;

      basic_point.x = 1* cosf (pcl::deg2rad(theta))* sinf (pcl::deg2rad(phi));
      basic_point.y =1 * sinf (pcl::deg2rad(theta))* sinf (pcl::deg2rad(phi));
      basic_point.z = 1 * cosf (pcl::deg2rad(phi));
      cloud->points.push_back(basic_point);
        }
      }
    }

  cloud->width = (int) cloud->points.size ();
  cloud->height = 1;

  // write point cloud to disk
 //pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/SpherRadius2.pcd", *cloud);
 //Write PLY
 pcl::PLYWriter writePLY;
 //writePLY.write ("/Path/TO/ArtificialPointClouds/SpherRadius2ply.ply", *cloud,  false);

   // Show the cloud
 //pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/CylinderBW.pcd", *cloud);
 pcl::visualization::CloudViewer viewer("Spher");
 viewer.showCloud(cloud);
while (!viewer.wasStopped ())
{}

  return 0;

}


#endif

#ifdef ELLIPSE_AND_Cylinder

#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

int
main (int argc, char** argv)
{


  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "Genarating example point clouds.\n\n";
  // We're going to make an Ellipse and Cylinder extruded along the z-axis. The colour for
  // the XYZRGB cloud will gradually go from red to green to blue.
  uint8_t r(255), g(15), b(15);
  for (float z(-2.0); z <= 2.0; z += 0.0009)
  {
    for (float angle(0.0); angle <= 360.0; angle += 4.0)
    {
      pcl::PointXYZ basic_point;
     // basic_point.x = 0.5 * 2 * cosf (pcl::deg2rad(angle));
      basic_point.x = 2 * cosf (pcl::deg2rad(angle));
      basic_point.y = 2 * sinf (pcl::deg2rad(angle));
      basic_point.z = z;
      basic_cloud_ptr->points.push_back(basic_point);

      pcl::PointXYZRGB point;
      point.x = basic_point.x;
      point.y = basic_point.y;
      point.z = basic_point.z;
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
    }
    if (z < 0.0)
    {
      r -= 12;
      g += 12;
    }
    else
    {
      g -= 12;
      b += 12;
    }
  }
  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;
  point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
  point_cloud_ptr->height = 1;

  // write point cloud to disk
  //pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/Ellipse.pcd", *point_cloud_ptr);
 pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/Cylinder.pcd", *point_cloud_ptr);
   // Show the cloud
 //pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/Ellipse.pcd", *point_cloud_ptr);
 pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/Cylinder.pcd", *point_cloud_ptr);
 //pcl::visualization::CloudViewer viewer("Ellipse");
 pcl::visualization::CloudViewer viewer("Cylinder");
viewer.showCloud(point_cloud_ptr);
while (!viewer.wasStopped ())
{}

  return 0;

}

#endif

#ifdef CYLINDER_B_W

#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  for (float z(-2.0); z <= 2.0; z += 0.09)
  {
    for (float angle(0.0); angle <= 360.0; angle += 1.0)
    {
      pcl::PointXYZ basic_point;
     basic_point.x = 0.5 * 2 * cosf (pcl::deg2rad(angle));
     // basic_point.x = 2 * cosf (pcl::deg2rad(angle));
      basic_point.y = 2 * sinf (pcl::deg2rad(angle));
      basic_point.z = z;
      cloud->points.push_back(basic_point);
    }
  }

  cloud->width = (int) cloud->points.size ();
  cloud->height = 1;

  // write point cloud to disk
 //pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/Cylinder2.pcd", *cloud);
 pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/Elipsoid2.pcd", *cloud);
 //Write PLY
 pcl::PLYWriter writePLY;
 //writePLY.write ("/Path/TO/ArtificialPointClouds/Cylinder2ply.ply", *cloud,  false);
 writePLY.write ("/Path/TO/ArtificialPointClouds/Elipsoid2ply.ply", *cloud,  false);

   // Show the cloud
 //pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/CylinderBW.pcd", *cloud);
 pcl::visualization::CloudViewer viewer("Cylinder");
 viewer.showCloud(cloud);
while (!viewer.wasStopped ())
{}

  return 0;

}

#endif

#ifdef CUBE_POINT_CLOUD

#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

int
main (int argc, char** argv)
{


  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "Genarating example point clouds.\n\n";
  // We're going to make an Ellipse and Cylinder extruded along the z-axis. The colour for
  // the XYZRGB cloud will gradually go from red to green to blue.
  uint8_t r(255), g(15), b(15);
  for (float z(-2.0); z <= 2.0; z += 0.0009)
  {
	  pcl::PointCloud<pcl::PointXYZ> basic_point;
      basic_point.width  = 640;       //307200;
      basic_point.height = 480;          // 1;
      basic_point.is_dense = false;
      basic_point.points.resize (basic_point.width * basic_point.height);
      basic_cloud_ptr->resize(basic_point.points.size());

      for (size_t i = 0; i < basic_point.points.size (); ++i)
        {
      basic_point.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
      basic_point.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
      basic_point.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
      basic_cloud_ptr->points.push_back(basic_point);

      pcl::PointCloud<pcl::PointXYZRGB> point;
      point.points[i].x = basic_point.points[i].x;
      point.points[i].y = basic_point.points[i].y;
      point.points[i].z = basic_point.points[i].z;
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
        }
    if (z < 0.0)
    {
      r -= 12;
      g += 12;
    }
    else
    {
      g -= 12;
      b += 12;
    }
  }
  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;
  point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
  point_cloud_ptr->height = 1;

  // write point cloud to disk
 pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/cube.pcd", *point_cloud_ptr);
   // Read the cloud
 pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/cube.pcd", *point_cloud_ptr);
 // Show the cloud
 pcl::visualization::CloudViewer viewer("cube");
viewer.showCloud(point_cloud_ptr);
while (!viewer.wasStopped ())
{}

  return 0;

}

#endif

#ifdef UNIFORM_SAMPLING_SPHERE

#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

int main (int argc, char** argv)
   {
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/UniformSpher.pcd", *cloud); //download

//chang the color of cloud
    for (size_t i = 0; i < cloud->size (); i++)
        {
    	cloud->points[i].r = 255;
    	cloud->points[i].g = 255;
        cloud->points[i].b = 255;
        }
    // write red point cloud to disk
pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/uniform_sphere.pcd", *cloud);
     // Show the cloud
pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/uniform_sphere.pcd", *cloud);

pcl::visualization::CloudViewer viewer("Cloud Viewer");
viewer.showCloud(cloud);
while (!viewer.wasStopped ())
{}

return 0;
}

#endif

#ifdef UNIFORM_SAMPLING_ELLIPSOID

#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
int main (int argc, char** argv)
   {
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/Ellipsoid.pcd", *cloud);  //download


for (size_t i = 0; i < cloud->size (); i++)
    {
	cloud->points[i].r = 255;
	cloud->points[i].g = 255;
    cloud->points[i].b = 255;
    }

// write red point cloud to disk
pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/uniform_Ellipsoid.pcd", *cloud);
 // Show the cloud
pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/uniform_Ellipsoid.pcd", *cloud);

pcl::visualization::CloudViewer viewer("Cloud Viewer");
viewer.showCloud(cloud);
while (!viewer.wasStopped ())
{}

return 0;
}

#endif

#ifdef CUBE_POINT_CLOUD_SIMPLE
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

int main ()
{
	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::PointXYZ o;

	  cloud-> width  = 640;       //307200;
	  cloud-> height = 480;          // 1;
	  cloud->is_dense = false;
	  cloud->points.resize (cloud->width * cloud-> height);


	 for (size_t i = 0; i < cloud->points.size (); ++i)
	   {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
	   }

	 for (size_t i = 0; i < cloud->size (); i++)
	     {
	 	cloud->points[i].r = 255;
	 	cloud->points[i].g = 0;
	     cloud->points[i].b = 0;
	     }

    // write red point cloud to disk
pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/cube.pcd", *cloud);
     // Show the cloud
pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/cube.pcd", *cloud);
pcl::visualization::CloudViewer viewer(" Cube ");
viewer.showCloud(cloud);
while (!viewer.wasStopped ())
{}
                            return (0);
                           }
#endif

#ifdef SADDLE_HOURSE_FIRST
   // Z= 1- X^2 + Y^2

#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  //for (float z(-2.0); z <= 2.0; z += 0.3)
  //{
    for (float theta(0.0); theta <= 360.0; theta += 5.0)
    {
      pcl::PointXYZ basic_point;

      basic_point.x = cosf (pcl::deg2rad(theta));
      basic_point.y = sinf (pcl::deg2rad(theta));
        basic_point.z = 1- cosf (2*(pcl::deg2rad(theta)));
      cloud->points.push_back(basic_point);

      //}
    }

  cloud->width = (int) cloud->points.size ();
  cloud->height = 1;

  // write point cloud to disk
 pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/Saddlepher2.pcd", *cloud);
 //Write PLY
 pcl::PLYWriter writePLY;
 writePLY.write ("/Path/TO/ArtificialPointClouds/Saddleply.ply", *cloud,  false);

   // Show the cloud
 //pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/CylinderBW.pcd", *cloud);
 pcl::visualization::CloudViewer viewer("Saddle");
 viewer.showCloud(cloud);
while (!viewer.wasStopped ())
{}

  return 0;

}

#endif

#ifdef SADDLE_HOURSE_SECOND
   // Z= 1- X^2 + Y^2

#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  for (float zz(0.00); zz <= 2.00 ; zz += 0.25)
    {
 // for (float theta(-45.00); theta <= 45.00; theta += 2.00)
	  for (float theta(-45.00); theta <= 45.00; theta += 3.00)
  {
 //    for (float phi(-45.00); phi <= 45.00; phi += 2.00)
		  for (float phi(-45.00); phi <= 45.00; phi += 3.00)
    {
      pcl::PointXYZ basic_point;

      basic_point.x = 2* (sinf (pcl::deg2rad(theta)) / cosf (pcl::deg2rad(theta))) * ( 1/ cosf (pcl::deg2rad(phi)));
      basic_point.y = sqrt (2) * ( 1/ cosf (pcl::deg2rad(theta))) * ( 1/ cosf (pcl::deg2rad(phi)));
      basic_point.z = 3 * (sinf (pcl::deg2rad(phi)) / cosf (pcl::deg2rad(phi))) ;
      cloud->points.push_back(basic_point);
        }
      }
    }

//  for (float zz(-3.00); zz <= 0.00; zz += 0.09)
//    {
// for (float theta(-45.00); theta <= 45.00; theta += 1.00)
//  {
//    for (float phi(-45.00); phi <= 45.00; phi += 1.00)
//    {
//      pcl::PointXYZ basic_point2;
//
//      basic_point2.x = 2* (sinf (pcl::deg2rad(theta)) / cosf (pcl::deg2rad(theta))) * ( 1/ cosf (pcl::deg2rad(phi)));
//      basic_point2.y = - sqrt (2) * ( 1/ cosf (pcl::deg2rad(theta))) * ( 1/ cosf (pcl::deg2rad(phi)));
//      basic_point2.z = zz * (sinf (pcl::deg2rad(phi)) / cosf (pcl::deg2rad(phi))) ;
//      cloud->points.push_back(basic_point2);
//        }
//      }
//    }

  cloud->width = (int) cloud->points.size ();
  cloud->height = 1;

  std::cout << "Number of points in the input cloud is:"<< cloud->points.size() << std::endl;

//  // write point cloud to disk
 pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/Saddle180.pcd", *cloud);
 //Write PLY
 pcl::PLYWriter writePLY;
 writePLY.write ("/Path/TO/ArtificialPointClouds/Saddle180.ply", *cloud,  false);

   // Show the cloud
 //pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/CylinderBW.pcd", *cloud);
 pcl::visualization::CloudViewer viewer("Saddle");
 viewer.showCloud(cloud);
while (!viewer.wasStopped ())
{}

  return 0;

}

#endif

#ifdef HYPERBOLIC_PARABOLOID
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  float r = 1.50;
  float a = 2.00;
  float b =4.50;
  //for (float zz(0.00); zz <= 2.00 ; zz += 0.25) {
 // for (float theta(-45.00); theta <= 45.00; theta += 2.00)
	  for (float theta(0.00); theta <= 180.00; theta += 1.00)
  {
 //    for (float phi(-45.00); phi <= 45.00; phi += 2.00)
		  for (float phi(0.00); phi <= 360.00; phi += 1.00)
    {
      pcl::PointXYZ basic_point;

      basic_point.x = r *  ( sinf (pcl::deg2rad(theta))* cosf (pcl::deg2rad(phi)));
      basic_point.y = r * (sinf (pcl::deg2rad(theta))* sinf (pcl::deg2rad(phi)));
      //basic_point.z =  (1/1)* (- 2 * (sinf (pcl::deg2rad(theta))) * (sinf (pcl::deg2rad(theta))) *  cosf (pcl::deg2rad(phi)));
    //  basic_point.z = r * r * sinf (pcl::deg2rad(theta)) * sinf (pcl::deg2rad(theta)) * ( ( (1/ (b*b))* (sinf (pcl::deg2rad(phi))) * (sinf (pcl::deg2rad(phi)))  )   -   (1/ (a*a)) * (cosf (pcl::deg2rad(phi)) )  * ( cosf (pcl::deg2rad(phi)))  );
      basic_point.z  = 1+  r * r * sinf (pcl::deg2rad(theta)) * sinf (pcl::deg2rad(theta))  * (- cosf (pcl::deg2rad(2* phi))  ) ;
      cloud->points.push_back(basic_point);
        //}
      }
    }



  cloud->width = (int) cloud->points.size ();
  cloud->height = 1;

  std::cout << "Number of points in the input cloud is:"<< cloud->points.size() << std::endl;

//  // write point cloud to disk
 pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/HyperbolicParaboloid2.pcd", *cloud);
 //Write PLY
 pcl::PLYWriter writePLY;
 writePLY.write ("/Path/TO/ArtificialPointClouds/HyperbolicParaboloid2.ply", *cloud,  false);

   // Show the cloud
 //pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/CylinderBW.pcd", *cloud);
 pcl::visualization::CloudViewer viewer("HyperbolicParaboloid");
 viewer.showCloud(cloud);
while (!viewer.wasStopped ())
{}

  return 0;

}
#endif

#ifdef SPHER_VTK

#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

int main(int, char *[])
{
  // Create a sphere
  vtkSmartPointer<vtkSphereSource> sphereSource =
    vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetCenter(0.0, 0.0, 0.0);
  sphereSource->SetRadius(5.0);

  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(sphereSource->GetOutputPort());

  vtkSmartPointer<vtkActor> actor =
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderer->AddActor(actor);
  renderer->SetBackground(.3, .6, .3); // Background color green

  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}


#endif


#ifdef SHARP_EDGE

#include <iostream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include "time.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char*argv[])
{
//	float* XX = new float [20];
//	float* YY = new float [20];
//	float* ZZ = new float [20];

	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	  srand ( time(NULL) );
	  for (int i=0 ; i <= 20; i++ )
{
		  for (int j=0 ; j <= 20; j++)
  {
    pcl::PointXYZRGBA basic_point;
//    XX[i] = float (rand() )* 1 / float (RAND_MAX);
//    YY[j] = float (rand() )* 1 / float (RAND_MAX);
    basic_point.x = float (rand() )* 1 / float (RAND_MAX + 1.0f);
    		basic_point.y = float (rand() )* 1 / float (RAND_MAX + 1.0f);
    				basic_point.z =0;
    						 cloud->points.push_back(basic_point);
		  }
  }

		  srand ( time(NULL) );
			  for (int j=0 ; j <= 20; j++)
	  {
				  for (int k=0 ; k <= 20; k++)
			  {
	    pcl::PointXYZRGBA basic_point2;
//	    YY[j] = float (rand() )* 1 / float (RAND_MAX);
//	    ZZ[k] = float (rand() )* 1 / float (RAND_MAX);
	    basic_point2.x = 0;
	    		basic_point2.y = float (rand() )* 1 / float (RAND_MAX + 1.0f);
	    				basic_point2.z =float (rand() )* 1 / float (RAND_MAX + 1.0f);
	    						 cloud->points.push_back(basic_point2);
			  }
	  }

			  cloud->width = (int) cloud->points.size ();
			  cloud->height = 1;

			  for (size_t i = 0; i < cloud ->points.size (); ++i) {
			    		cloud->points[i].r = 255;
			    		cloud->points[i].g = 255;
			    		cloud->points[i].b = 255;
			  }

			  //  // write point cloud to disk
			   pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/SharpEdge.pcd", *cloud);
			   //Write PLY
			   pcl::PLYWriter writePLY;
			   writePLY.write ("/Path/TO/ArtificialPointClouds/SharpEdge.ply", *cloud,  false);

			     // Show the cloud
			   pcl::visualization::CloudViewer viewer("Sharp Edge");
			   viewer.showCloud(cloud);
			  while (!viewer.wasStopped ())
			  {}


	  return 0;
}

#endif

#ifdef CUBE_FOUR_EGDE
#include <iostream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include "time.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char*argv[])
{
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	  // random number between two number (High and low)
	  // Random number time
//	     srand (time(NULL));
//	     randNum = (rand()%(hi-lo))+ lo;
	  // rand() % (max - min) + min;

	  srand ( time(NULL) );
	  for (int i=0 ; i <= 70; i++ )    // Cube 10K , 40 ------> For more density CUbe 30K , i<= 120     // Or We Can Make the size of the cube 3 times less , the size of the cube is between 0 and 0.3
{
		  for (int j=0 ; j <= 70; j++)
  {
    pcl::PointXYZRGBA basic_point;
    basic_point.x = float (rand() )* 0.3 / float (RAND_MAX + 0.3f);
    		basic_point.y = float (rand() )* 0.3 / float (RAND_MAX + 0.3f);
    				basic_point.z =0;
    						 cloud->points.push_back(basic_point);
		  }
  }

		  srand ( time(NULL) );
			  for (int j=0 ; j <= 70; j++)
	  {
				  for (int k=0 ; k <= 70; k++)
			  {
	    pcl::PointXYZRGBA basic_point2;
	    basic_point2.x = 0;
	    		basic_point2.y = float (rand() )* 0.3 / float (RAND_MAX + 0.3f);
	    				basic_point2.z =float (rand() )* 0.3 / float (RAND_MAX + 0.3f);
	    						 cloud->points.push_back(basic_point2);
			  }
	  }
			  srand ( time(NULL) );
				  for (int j=0 ; j <= 70; j++)
		  {
					  for (int k=0 ; k <= 70; k++)
				  {
		    pcl::PointXYZRGBA basic_point2;
		    basic_point2.x = float (rand() )* 0.3 / float (RAND_MAX + 0.3f);
		    		basic_point2.y = 0;
		    				basic_point2.z =float (rand() )* 0.3 / float (RAND_MAX + 0.3f);
		    						 cloud->points.push_back(basic_point2);
				  }
		  }
				  srand ( time(NULL) );
				  for (int i=0 ; i <= 70; i++ )
			{
					  for (int j=0 ; j <= 70; j++)
			  {
			    pcl::PointXYZRGBA basic_point;
			    basic_point.x = float (rand() )* 0.3 / float (RAND_MAX + 0.3f);
			    		basic_point.y = float (rand() )* 0.3 / float (RAND_MAX + 0.3f);
			    				basic_point.z =0.3;
			    						 cloud->points.push_back(basic_point);
					  }
			  }

					  srand ( time(NULL) );
						  for (int j=0 ; j <= 70; j++)
				  {
							  for (int k=0 ; k <= 70; k++)
						  {
				    pcl::PointXYZRGBA basic_point2;
				    basic_point2.x = 0.3;
				    		basic_point2.y = float (rand() )* 0.3 / float (RAND_MAX + 0.3f);
				    				basic_point2.z =float (rand() )* 0.3 / float (RAND_MAX + 0.3f);
				    						 cloud->points.push_back(basic_point2);
						  }
				  }
						  srand ( time(NULL) );
							  for (int j=0 ; j <= 120; j++)
					  {
								  for (int k=0 ; k <= 120; k++)
							  {
					    pcl::PointXYZRGBA basic_point2;
					    basic_point2.x = float (rand() )* 0.3 / float (RAND_MAX + 0.3f);
					    		basic_point2.y = 0.3;
					    				basic_point2.z =float (rand() )* 0.3 / float (RAND_MAX + 0.3f);
					    						 cloud->points.push_back(basic_point2);
							  }
					  }







			  cloud->width = (int) cloud->points.size ();
			  cloud->height = 1;

			  for (size_t i = 0; i < cloud ->points.size (); ++i) {
			    		cloud->points[i].r = 255;
			    		cloud->points[i].g = 255;
			    		cloud->points[i].b = 255;
			  }

			  //  // write point cloud to disk
			   pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/Cube30KSharpEdgeSMALL.pcd", *cloud);
			   //Write PLY
			   pcl::PLYWriter writePLY;
			   writePLY.write ("/Path/TO/ArtificialPointClouds/Cube30KSharpEdgeSMALL.ply", *cloud,  false);
				std::cout << "Number of points in the input cloud is:"<< cloud->points.size() << std::endl;

			     // Show the cloud
			   pcl::visualization::CloudViewer viewer("Sharp Edge");
			   viewer.showCloud(cloud);
			  while (!viewer.wasStopped ())
			  {}


	  return 0;
}

#endif


#ifdef CUBE_CYLINDER

#include <iostream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include "time.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char*argv[])
{
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	  srand ( time(NULL) );
	  for (int i=0 ; i <= 40; i++ )
{
		  for (int j=0 ; j <= 40; j++)
  {
    pcl::PointXYZRGBA basic_point;
    basic_point.x = float (rand() )* 0.5 / float (RAND_MAX + 0.5f);
    		basic_point.y = float (rand() )* 0.5 / float (RAND_MAX + 0.5f);
    				basic_point.z =0;
    						 cloud->points.push_back(basic_point);
		  }
  }

		  srand ( time(NULL) );
			  for (int j=0 ; j <= 40; j++)
	  {
				  for (int k=0 ; k <= 40; k++)
			  {
	    pcl::PointXYZRGBA basic_point2;
	    basic_point2.x = 0;
	    		basic_point2.y = float (rand() )* 0.5 / float (RAND_MAX + 0.5f);
	    				basic_point2.z =float (rand() )* 0.5 / float (RAND_MAX + 0.5f);
	    						 cloud->points.push_back(basic_point2);
			  }
	  }
			  srand ( time(NULL) );
				  for (int j=0 ; j <= 40; j++)
		  {
					  for (int k=0 ; k <= 40; k++)
				  {
		    pcl::PointXYZRGBA basic_point2;
		    basic_point2.x = float (rand() )* 0.5 / float (RAND_MAX + 0.5f);
		    		basic_point2.y = 0;
		    				basic_point2.z =float (rand() )* 0.5 / float (RAND_MAX + 0.5f);
		    						 cloud->points.push_back(basic_point2);
				  }
		  }
				  srand ( time(NULL) );
				  for (int i=0 ; i <= 40; i++ )
			{
					  for (int j=0 ; j <= 40; j++)
			  {
			    pcl::PointXYZRGBA basic_point;
			    basic_point.x = float (rand() )* 0.5 / float (RAND_MAX + 0.5f);
			    		basic_point.y = float (rand() )* 0.5 / float (RAND_MAX + 0.5f);
			    				basic_point.z =0.5;
			    						 cloud->points.push_back(basic_point);
					  }
			  }

					  srand ( time(NULL) );
						  for (int j=0 ; j <= 40; j++)
				  {
							  for (int k=0 ; k <= 40; k++)
						  {
				    pcl::PointXYZRGBA basic_point2;
				    basic_point2.x = 0.5;
				    		basic_point2.y = float (rand() )* 0.5 / float (RAND_MAX + 0.5f);
				    				basic_point2.z =float (rand() )* 0.5 / float (RAND_MAX + 0.5f);
				    						 cloud->points.push_back(basic_point2);
						  }
				  }
						  srand ( time(NULL) );
							  for (int j=0 ; j <= 40; j++)
					  {
								  for (int k=0 ; k <= 40; k++)
							  {
					    pcl::PointXYZRGBA basic_point2;
					    basic_point2.x = float (rand() )* 0.5 / float (RAND_MAX + 0.5f);
					    		basic_point2.y = 0.5;
					    				basic_point2.z =float (rand() )* 0.5 / float (RAND_MAX + 0.5f);
					    						 cloud->points.push_back(basic_point2);
							  }
					  }

							  cloud->width = (int) cloud->points.size ();
							  cloud->height = 1;

// Creating Cylinder at that Coordinat inside of the cube

							  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudCylinder (new pcl::PointCloud<pcl::PointXYZRGBA>);
								float* zz  = new float [30]; // 100
							  float* phi = new float [360];  //360
							 // for (float z(0.0); z <= 1.0; z += 0.09)
							  for (int i=0 ; i < 30 ; i++)
							   {
								      zz [i] = float (rand() )* 0.5 / float (RAND_MAX + 0.5f);;
							    //for (float angle(0.0); angle <= 360.0; angle += 1.0)
									 for (int j=0 ; j < 360; j++)
							    {
								   phi[j] = float (rand() )* 360 / float (RAND_MAX);
							      pcl::PointXYZRGBA basic_point;
							      basic_point.x = 0.25 + 0.25 * cosf (pcl::deg2rad(phi[j]));
							      basic_point.y = 0.25 + 0.25 * sinf (pcl::deg2rad(phi[j]));
							      basic_point.z = 0.0 +   zz [i];
							      cloudCylinder->points.push_back(basic_point);
							    }
							  }

							  cloudCylinder->width = (int) cloud->points.size ();
							  cloudCylinder->height = 1;

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudCubeCylinder (new pcl::PointCloud<pcl::PointXYZRGBA>);
cloudCubeCylinder ->resize( ( cloud->points.size() ) + ( cloudCylinder->points.size() ) );

			  for (size_t i = 0; i < cloud ->points.size (); ++i) {
				  cloudCubeCylinder->points[i].x = cloud->points[i].x ;
				  cloudCubeCylinder->points[i].y = cloud->points[i].y ;
				  cloudCubeCylinder->points[i].z = cloud->points[i].z ;
				  cloudCubeCylinder->points[i].r = 255;
				  cloudCubeCylinder->points[i].g = 255;
				  cloudCubeCylinder->points[i].b = 255;
			  }
			  for (size_t i = 0; i < cloudCylinder ->points.size (); ++i) {
				  cloudCubeCylinder->points[i+( cloud->points.size() ) ].x = cloudCylinder->points[i].x ;
				  cloudCubeCylinder->points[i+( cloud->points.size() ) ].y = cloudCylinder->points[i].y ;
				  cloudCubeCylinder->points[i+( cloud->points.size() ) ].z = cloudCylinder->points[i].z ;
				  cloudCubeCylinder->points[i+( cloud->points.size() ) ].r = 255;
				  cloudCubeCylinder->points[i+( cloud->points.size() ) ].g = 255;
				  cloudCubeCylinder->points[i+( cloud->points.size() ) ].b = 255;
			  }




			  //  // write point cloud to disk
			  // pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/CubeCylinder2.pcd", *cloudCubeCylinder);
			   //Write PLY
			   pcl::PLYWriter writePLY;
			   writePLY.write ("/Path/TO/ArtificialPointClouds/CubesmallCylinderSmall.ply", *cloudCubeCylinder,  false);

			   std::cout << "Number of points for the created cloud is:"<< cloudCubeCylinder->points.size() << std::endl;
			     // Show the cloud
			   pcl::visualization::CloudViewer viewer("Cube-Cylinder");
			   viewer.showCloud(cloudCubeCylinder);
			  while (!viewer.wasStopped ())
			  {}


	  return 0;
}


#endif



#ifdef CUBE_WITHOUT_CYLINDER

#include <iostream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include "time.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char*argv[])
{
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	  srand ( time(NULL) );
	  for (int i=0 ; i <= 40; i++ )
{
		  for (int j=0 ; j <= 40; j++)
  {
    pcl::PointXYZRGBA basic_point;
    basic_point.x = float (rand() )* 1 / float (RAND_MAX + 1.0f);
    		basic_point.y = float (rand() )* 1 / float (RAND_MAX + 1.0f);
    				basic_point.z =0;
    						 cloud->points.push_back(basic_point);
		  }
  }

		  srand ( time(NULL) );
			  for (int j=0 ; j <= 40; j++)
	  {
				  for (int k=0 ; k <= 40; k++)
			  {
	    pcl::PointXYZRGBA basic_point2;
	    basic_point2.x = 0;
	    		basic_point2.y = float (rand() )* 1 / float (RAND_MAX + 1.0f);
	    				basic_point2.z =float (rand() )* 1 / float (RAND_MAX + 1.0f);
	    						 cloud->points.push_back(basic_point2);
			  }
	  }
			  srand ( time(NULL) );
				  for (int j=0 ; j <= 40; j++)
		  {
					  for (int k=0 ; k <= 40; k++)
				  {
		    pcl::PointXYZRGBA basic_point2;
		    basic_point2.x = float (rand() )* 1 / float (RAND_MAX + 1.0f);
		    		basic_point2.y = 0;
		    				basic_point2.z =float (rand() )* 1 / float (RAND_MAX + 1.0f);
		    						 cloud->points.push_back(basic_point2);
				  }
		  }
				  srand ( time(NULL) );
				  for (int i=0 ; i <= 40; i++ )
			{
					  for (int j=0 ; j <= 40; j++)
			  {
			    pcl::PointXYZRGBA basic_point;
			    basic_point.x = float (rand() )* 1 / float (RAND_MAX + 1.0f);
			    		basic_point.y = float (rand() )* 1 / float (RAND_MAX + 1.0f);
			    				basic_point.z =1;
			    						 cloud->points.push_back(basic_point);
					  }
			  }

					  srand ( time(NULL) );
						  for (int j=0 ; j <= 40; j++)
				  {
							  for (int k=0 ; k <= 40; k++)
						  {
				    pcl::PointXYZRGBA basic_point2;
				    basic_point2.x = 1;
				    		basic_point2.y = float (rand() )* 1 / float (RAND_MAX + 1.0f);
				    				basic_point2.z =float (rand() )* 1 / float (RAND_MAX + 1.0f);
				    						 cloud->points.push_back(basic_point2);
						  }
				  }
						  srand ( time(NULL) );
							  for (int j=0 ; j <= 40; j++)
					  {
								  for (int k=0 ; k <= 40; k++)
							  {
					    pcl::PointXYZRGBA basic_point2;
					    basic_point2.x = float (rand() )* 1 / float (RAND_MAX + 1.0f);
					    		basic_point2.y = 1;
					    				basic_point2.z =float (rand() )* 1 / float (RAND_MAX + 1.0f);
					    						 cloud->points.push_back(basic_point2);
							  }
					  }

							  cloud->width = (int) cloud->points.size ();
							  cloud->height = 1;

// Creating Cylinder at that Coordinat inside of the cube

							  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudCylinder (new pcl::PointCloud<pcl::PointXYZRGBA>);
								float* zz  = new float [100]; // 100
							  float* phi = new float [360];  //360
							 // for (float z(0.0); z <= 1.0; z += 0.09)
							  for (int i=0 ; i < 100 ; i++)
							   {
								      zz [i] = (rand()% 100 - 0)/(100.00);
							    //for (float angle(0.0); angle <= 360.0; angle += 1.0)
									 for (int j=0 ; j < 360; j++)
							    {
								   phi[j] = float (rand() )* 360 / float (RAND_MAX);
							      pcl::PointXYZRGBA basic_point;
							      basic_point.x = 1.0 + 0.43 * cosf (pcl::deg2rad(phi[j]));
							      basic_point.y = 1.0 + 0.43 * sinf (pcl::deg2rad(phi[j]));
							      basic_point.z = 0.0 +   zz [i];
							      cloudCylinder->points.push_back(basic_point);
							    }
							  }

							  cloudCylinder->width = (int) cloud->points.size ();
							  cloudCylinder->height = 1;

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudCubeCylinder (new pcl::PointCloud<pcl::PointXYZRGBA>);
cloudCubeCylinder ->resize( ( cloud->points.size() ) + ( cloudCylinder->points.size() ) );

			  for (size_t i = 0; i < cloud ->points.size (); ++i) {
				  cloudCubeCylinder->points[i].x = cloud->points[i].x ;
				  cloudCubeCylinder->points[i].y = cloud->points[i].y ;
				  cloudCubeCylinder->points[i].z = cloud->points[i].z ;
				  cloudCubeCylinder->points[i].r = 255;
				  cloudCubeCylinder->points[i].g = 255;
				  cloudCubeCylinder->points[i].b = 255;
			  }
			  for (size_t i = 0; i < cloudCylinder ->points.size (); ++i) {
				  cloudCubeCylinder->points[i+( cloud->points.size() ) ].x = cloudCylinder->points[i].x ;
				  cloudCubeCylinder->points[i+( cloud->points.size() ) ].y = cloudCylinder->points[i].y ;
				  cloudCubeCylinder->points[i+( cloud->points.size() ) ].z = cloudCylinder->points[i].z ;
				  cloudCubeCylinder->points[i+( cloud->points.size() ) ].r = 255;
				  cloudCubeCylinder->points[i+( cloud->points.size() ) ].g = 255;
				  cloudCubeCylinder->points[i+( cloud->points.size() ) ].b = 255;
			  }




			  //  // write point cloud to disk
			  // pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/CubeCylinder2.pcd", *cloudCubeCylinder);
			   //Write PLY
			   pcl::PLYWriter writePLY;
			   writePLY.write ("/Path/TO/ArtificialPointClouds/CubeSinCylinder2.ply", *cloudCubeCylinder,  false);

			     // Show the cloud
			   pcl::visualization::CloudViewer viewer("Cube-Cylinder");
			   viewer.showCloud(cloudCubeCylinder);
			  while (!viewer.wasStopped ())
			  {}


	  return 0;
}


#endif

#ifdef WEDGE_SHARP

#include <iostream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include "time.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char*argv[])
{
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);


	  float thetaS =10.00 ;
	  float thetaE = 30.00;

	  srand ( time(NULL) );

		  for (int j=0 ; j < 1750; j++)
  {
    pcl::PointXYZRGBA basic_point;
    float alpha = ( float (rand() )* 1.0f/ float (RAND_MAX + 1.0f) ) ;
    		float betha = ( float (rand() )* 1.0f/ float (RAND_MAX + 1.0f) ) ;
                     float theta = thetaS + ( (thetaE - thetaS) * (alpha ) );
    basic_point.x = ( betha ) * (cosf (pcl::deg2rad(theta))) ;
    		basic_point.y = ( betha ) * (sinf (pcl::deg2rad(theta)));
    				basic_point.z = alpha ;
    						 cloud->points.push_back(basic_point);
		  }

		  srand ( time(NULL) );

			  for (int j=0 ; j < 1750; j++)
	  {
	    pcl::PointXYZRGBA basic_point;
	    float alpha = ( float (rand() )* 1.0f/ float (RAND_MAX + 1.0f) ) ;
	    		float betha = ( float (rand() )* 1.0f/ float (RAND_MAX + 1.0f) ) ;
	                    //  float theta = thetaS + ( (thetaE - thetaS) * (alpha ) );
	    		float theta = 0 ;
	    basic_point.x = ( betha ) * (cosf (pcl::deg2rad(theta))) ;
	    		basic_point.y = - ( betha ) * (sinf (pcl::deg2rad(theta)));
	    				basic_point.z = alpha ;
	    						 cloud->points.push_back(basic_point);
			  }


	  cloud->width = (int) cloud->points.size ();
	  cloud->height = 1;

	  std::cout << "Number of points in the input cloud is:"<< cloud->points.size() << std::endl;

	  for (size_t i = 0; i < cloud ->points.size (); ++i) {
		  cloud->points[i].r = 255;
		  cloud->points[i].g = 255;
		  cloud->points[i].b = 255;
	  }

	  //  // write point cloud to disk
	  // pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/CubeCylinder2.pcd", *cloudCubeCylinder);
	   //Write PLY
	   pcl::PLYWriter writePLY;
	   writePLY.write ("/Path/TO/ArtificialPointClouds/Wedge.ply", *cloud,  false);

	     // Show the cloud
	   pcl::visualization::CloudViewer viewer("Wedge");
	   viewer.showCloud(cloud);
	  while (!viewer.wasStopped ())
	  {}


return 0;
}

#endif

#ifdef WEDGE_CYLINDER_NONSYM

#include <iostream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include "time.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char*argv[])
{
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);


	  float thetaS =10.00 ;
	  float thetaE = 30.00;
	  float Scale = 0.5 ;

	  srand ( time(NULL) );

		  for (int j=0 ; j < 1750; j++)
  {
    pcl::PointXYZRGBA basic_point;
    float alpha = ( float (rand() )* 1.0f/ float (RAND_MAX + 1.0f) ) ;
               float theta = thetaS + ( (thetaE - thetaS) * (alpha ) );
    		          float betha = ( float (rand() )* 1.0f/ float (RAND_MAX + 1.0f) ) ;
    		          if ( betha >   Scale) {
    basic_point.x = ( betha ) * (cosf (pcl::deg2rad(theta))) ;
    		basic_point.y = ( betha ) * (sinf (pcl::deg2rad(theta)));
    				basic_point.z = alpha ;
    						 cloud->points.push_back(basic_point);
    		          } // if betha larger than scale
    		          else {
float Radius = Scale * tan ( pcl::deg2rad(theta/2) ) ;
float Xc = Scale;
		float Yc =  Scale * tan ( pcl::deg2rad(theta/2) ) ;
        float gamma = ( float (rand() )* 1.0f/ float (RAND_MAX + 1.0f) ) ;
        if ( gamma < (3.141592 * ((180 -  theta) / 360 ) * tan ( pcl::deg2rad(theta/2) ) )  ) {
            basic_point.x = ( Scale) * (1 - tan( pcl::deg2rad(theta/2) ) *  sinf (pcl::deg2rad(  (90 - theta/2) * (1+ betha/ Scale )         ))) ;
            		basic_point.y = ( Scale ) *   tan( pcl::deg2rad(theta/2) ) * (  1-  (cosf (pcl::deg2rad((90 - theta/2) * (1+ betha/ Scale )))));
            				basic_point.z = alpha ;
            						 cloud->points.push_back(basic_point);
// other part
            				            basic_point.x = ( Scale) * (1 - tan( pcl::deg2rad(theta/2) ) *  sinf (pcl::deg2rad(  (90 - theta/2) * ( betha/ Scale )         ))) ;
            				            		basic_point.y = ( Scale ) *   tan( pcl::deg2rad(theta/2) ) * (  1-  (cosf (pcl::deg2rad((90 - theta/2) * ( betha/ Scale )))));
            				            				basic_point.z = alpha ;
            				            						 cloud->points.push_back(basic_point);

        } // if Gamma

    		          } // if betha smaller than scale


  } // For


		  srand ( time(NULL) );

			  for (int j=0 ; j < 1750; j++)
	  {
	    pcl::PointXYZRGBA basic_point;
	    float alpha = ( float (rand() )* 1.0f/ float (RAND_MAX + 1.0f) ) ;
	               float theta = thetaS + ( (thetaE - thetaS) * (alpha ) );
	    		          float betha = ( float (rand() )* 1.0f/ float (RAND_MAX + 1.0f) ) ;
	    		          if ( betha >   Scale) {
	    basic_point.x = ( betha ) * (cosf (pcl::deg2rad(theta))) ;
	    		basic_point.y = - ( betha ) * (sinf (pcl::deg2rad(theta)));
	    				basic_point.z = alpha ;
	    						 cloud->points.push_back(basic_point);
	    		          } // if betha larger than scale
	    		          else {
	float Radius = Scale * tan ( pcl::deg2rad(theta/2) ) ;
	float Xc = Scale;
			float Yc =  Scale * tan ( pcl::deg2rad(theta/2) ) ;
	        float gamma = ( float (rand() )* 1.0f/ float (RAND_MAX + 1.0f) ) ;
	        if ( gamma < (3.141592 * ((180 -  theta) / 360 ) * tan ( pcl::deg2rad(theta/2) ) )  ) {
	            basic_point.x = ( Scale) * (1 - tan( pcl::deg2rad(theta/2) ) *  sinf (pcl::deg2rad(  (90 - theta/2) * (1+ betha/ Scale )         ))) ;
	            		basic_point.y = - ( Scale ) *   tan( pcl::deg2rad(theta/2) ) * (  1-  (cosf (pcl::deg2rad((90 - theta/2) * (1+ betha/ Scale )))));
	            				basic_point.z = alpha ;
	            						 cloud->points.push_back(basic_point);
	// other part
	            				            basic_point.x = ( Scale) * (1 - tan( pcl::deg2rad(theta/2) ) *  sinf (pcl::deg2rad(  (90 - theta/2) * ( betha/ Scale )         ))) ;
	            				            		basic_point.y = - ( Scale ) *   tan( pcl::deg2rad(theta/2) ) * (  1-  (cosf (pcl::deg2rad((90 - theta/2) * ( betha/ Scale )))));
	            				            				basic_point.z = alpha ;
	            				            						 cloud->points.push_back(basic_point);

	        } // if Gamma

	    		          } // if betha smaller than scale


	  } // For






	  cloud->width = (int) cloud->points.size ();
	  cloud->height = 1;

	  std::cout << "Number of points in the input cloud is:"<< cloud->points.size() << std::endl;

	  for (size_t i = 0; i < cloud ->points.size (); ++i) {
		  cloud->points[i].r = 255;
		  cloud->points[i].g = 255;
		  cloud->points[i].b = 255;
	  }

	  //  // write point cloud to disk
	  // pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/CubeCylinder2.pcd", *cloudCubeCylinder);
	   //Write PLY
	   pcl::PLYWriter writePLY;
	   writePLY.write ("/Path/TO/ArtificialPointClouds/WedgeCylinder.ply", *cloud,  false);

	     // Show the cloud
	   pcl::visualization::CloudViewer viewer("Wedge");
	   viewer.showCloud(cloud);
	  while (!viewer.wasStopped ())
	  {}


return 0;
}




#endif


#ifdef WEDGE_SYM


#include <iostream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include "time.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char*argv[])
{
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	  float thetaS = 22.5 ;        // 5.00 ;
	  float thetaE = 22.5;                 // 60.00;
	  float Scale =  0.00;                   // 0.80 ;

	  srand ( time(NULL) );
		  for (int j=0 ; j < 3000; j++)
  {
    pcl::PointXYZRGBA basic_point;
    pcl::PointXYZRGBA ground_truth;
    float alpha = ( float (rand() )* 1.0f/ float (RAND_MAX + 1.0f) ) ;
               float theta = thetaS + ( (thetaE - thetaS) * (alpha ) );
    		          float beta = ( float (rand() )* 1.0f/ float (RAND_MAX + 1.0f) ) ;
    		          if ( beta >   Scale) {
    basic_point.x = ( beta ) * (cosf (pcl::deg2rad(theta))) ;
    		basic_point.y = ( beta ) * (sinf (pcl::deg2rad(theta)));
    				basic_point.z = alpha ;
    				    basic_point.r = 255 ;
    				    basic_point.g = 255 ;
    				    basic_point.b = 255 ;
    						 cloud->points.push_back(basic_point);
    		          } // if betha larger than scale
    		          else {
float Radius = Scale * tan ( pcl::deg2rad(theta) ) ;
   float Xc = Scale/ (cosf (pcl::deg2rad(theta))) ;
		float Yc =  0 ;
		float phi = ( beta/ Scale ) * (90 -  theta) ;
        float gamma = ( float (rand() )* 1.0f/ float (RAND_MAX + 1.0f) ) ;
        if ( gamma < ( (( 3.141592 ) * (90 -  theta) * (tan ( pcl::deg2rad(theta))) ) / 180  ) ) {
            basic_point.x = Xc - ( Radius *  (cosf (pcl::deg2rad(phi))) ) ;
            		basic_point.y = ( Radius *  (sinf (pcl::deg2rad(phi))) );
            				basic_point.z = alpha ;
            						 cloud->points.push_back(basic_point);
             } // if Gamma
    		          } // if betha smaller than scale
                            } // For
		  srand ( time(NULL) );
			  for (int j=0 ; j < 3000; j++)
	  {
	    pcl::PointXYZRGBA basic_point;
	    pcl::PointXYZRGBA ground_truth;
	    float alpha = ( float (rand() )* 1.0f/ float (RAND_MAX + 1.0f) ) ;
	               float theta = thetaS + ( (thetaE - thetaS) * (alpha ) );
	    		          float beta = ( float (rand() )* 1.0f/ float (RAND_MAX + 1.0f) ) ;
	    		          if ( beta >   Scale) {
	    basic_point.x = ( beta ) * (cosf (pcl::deg2rad(theta))) ;
	    		basic_point.y = - ( beta ) * (sinf (pcl::deg2rad(theta)));
	    				basic_point.z = alpha ;
	    				 basic_point.r = 255 ;
	    				 basic_point.g = 255 ;
	    				 basic_point.b = 255 ;
	    						 cloud->points.push_back(basic_point);
	    		          } // if betha larger than scale
	    		          else {
	float Radius = Scale * tan ( pcl::deg2rad(theta) ) ;
	        float Xc = Scale/ (cosf (pcl::deg2rad(theta))) ;
			float Yc =  0 ;
			float phi = ( beta/ Scale ) * (90 -  theta) ;
	 float gamma = ( float (rand() )* 1.0f/ float (RAND_MAX + 1.0f) ) ;
	         if ( gamma < ( (( 3.141592 ) * (90 -  theta) * (tan ( pcl::deg2rad(theta))) ) / 180  ) ) {
	            basic_point.x = Xc - ( Radius *  (cosf (pcl::deg2rad(phi))) ) ;
	           	basic_point.y = - ( Radius *  (sinf (pcl::deg2rad(phi))) );
	            				basic_point.z = alpha ;
	            						 cloud->points.push_back(basic_point);
	        } // if Gamma
	    		          } // if betha smaller than scale
	                                 } // For

cloud->width = (int) cloud->points.size ();
cloud->height = 1;

for (size_t i = 0; i < cloud ->points.size (); ++i) {
	  cloud->points[i].r = 255;
	  cloud->points[i].g = 255;
	  cloud->points[i].b = 255;
}

std::cout << "Number of points in the input cloud is:"<< cloud->points.size() << std::endl;

//  // write point cloud to disk
  // pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/Wedge/W0.8S5T60.pcd", *cloud);
 //Write PLY
 pcl::PLYWriter writePLY;
    // writePLY.write ("/Path/TO/ArtificialPointClouds/Wedge/W0.8S5T60.ply", *cloud,  false);

   // Show the cloud
 pcl::visualization::CloudViewer viewer("Wedge");
 viewer.showCloud(cloud);
while (!viewer.wasStopped ())
{}


return 0;
}

#endif


#ifdef STAIRS_THREE_CUBE
#include <iostream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include "time.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char*argv[])
{
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	  // random number between two number (High and low)
	  // Random number time
//	     srand (time(NULL));
//	     randNum = (rand()%(hi-lo))+ lo;
	  // rand() % (max - min) + min;
	 //   (((((float) rand()) / (float) RAND_MAX) * (hi-lo)) + lo) ;

	  //First Cube X,Y,Z  between 0 and 0.75
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 300; i++ )    // Cube 10K , 40 ------> For more density CUbe 30K , i<= 120     // Or We Can Make the size of the cube 3 times less , the size of the cube is between 0 and 0.3
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.75- 0.00)) + 0.00) ;
    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.75- 0.00)) + 0.00) ;
    				basic_point.z =0.00;
    						 cloud->points.push_back(basic_point);
  }

		  srand ( time(NULL) );
			  for (int j=0 ; j <= 300; j++)
	  {
	    pcl::PointXYZRGBA basic_point;
	    basic_point.x = 0.00;
	    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.75- 0.00)) + 0.00) ;
	    				basic_point.z = (((((float) rand()) / (float) RAND_MAX) * (0.75- 0.00)) + 0.00) ;
	    						 cloud->points.push_back(basic_point);
	  }
			  srand ( time(NULL) );
				  for (int j=0 ; j <= 300; j++)
		  {
		    pcl::PointXYZRGBA basic_point;
		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.75- 0.00)) + 0.00) ;
		    		basic_point.y = 0.00;
		    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.75- 0.00)) + 0.00) ;
		    						 cloud->points.push_back(basic_point);
		  }
				  srand ( time(NULL) );
				  for (int i=0 ; i <= 300; i++ )
			{
			    pcl::PointXYZRGBA basic_point;
			    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.75- 0.00)) + 0.00) ;
			    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.75- 0.00)) + 0.00) ;
			    				basic_point.z =0.75;
			    						 cloud->points.push_back(basic_point);
			  }

					  srand ( time(NULL) );
						  for (int j=0 ; j <= 300; j++)
				  {
				    pcl::PointXYZRGBA basic_point;
				    basic_point.x = 0.75;
				    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.75- 0.00)) + 0.00) ;
				    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.75- 0.00)) + 0.00) ;
				    						 cloud->points.push_back(basic_point);
				  }
						  srand ( time(NULL) );
							  for (int j=0 ; j <= 300; j++)
					  {
					    pcl::PointXYZRGBA basic_point;
					    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.75- 0.00)) + 0.00) ;
					    		basic_point.y = 0.75;
					    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.75- 0.00)) + 0.00) ;
					    						 cloud->points.push_back(basic_point);
					  }
			// Second Cube X, z between 0,5 and Z between 0.75 and 1,25
							  srand ( time(NULL) );
							  for (int i=0 ; i <= 200; i++ )    // Cube 10K , 40 ------> For more density CUbe 30K , i<= 120     // Or We Can Make the size of the cube 3 times less , the size of the cube is between 0 and 0.3
						{
						    pcl::PointXYZRGBA basic_point;
						    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.50 - 0.00)) + 0.00) ;
						    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.25- 0.75)) + 0.75) ;
						    				basic_point.z =0.00;
						    						 cloud->points.push_back(basic_point);
						  }

								  srand ( time(NULL) );
									  for (int j=0 ; j <= 200; j++)
							  {
							    pcl::PointXYZRGBA basic_point;
							    basic_point.x = 0.00;
							    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.25- 0.75)) + 0.75) ;
							    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
							    						 cloud->points.push_back(basic_point);
							  }
									  srand ( time(NULL) );
										  for (int j=0 ; j <= 200; j++)
								  {
								    pcl::PointXYZRGBA basic_point;
								    basic_point.x =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
								    		basic_point.y = 0.75;
								    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
								    						 cloud->points.push_back(basic_point);
								  }
										  srand ( time(NULL) );
										  for (int i=0 ; i <= 200; i++ )
									{
									    pcl::PointXYZRGBA basic_point;
									    basic_point.x =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
									    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.25- 0.75)) + 0.75) ;
									    				basic_point.z =0.50;
									    						 cloud->points.push_back(basic_point);
									  }

											  srand ( time(NULL) );
												  for (int j=0 ; j <= 200; j++)
										  {
										    pcl::PointXYZRGBA basic_point;
										    basic_point.x = 0.50;
										    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.25- 0.75)) + 0.75) ;
										    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
										    						 cloud->points.push_back(basic_point);
										  }
												  srand ( time(NULL) );
													  for (int j=0 ; j <= 200; j++)
											  {
											    pcl::PointXYZRGBA basic_point;
											    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
											    		basic_point.y = 1.25;
											    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
											    						 cloud->points.push_back(basic_point);
											  }

				// Third Cube a, z between 0 and 0,25 and Y between 1,25 and 1, 50
																		  srand ( time(NULL) );
																		  for (int i=0 ; i <= 100; i++ )    // Cube 10K , 40 ------> For more density CUbe 30K , i<= 120     // Or We Can Make the size of the cube 3 times less , the size of the cube is between 0 and 0.3
																	{
																	    pcl::PointXYZRGBA basic_point;
																	    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.25- 0.00)) + 0.00) ;
																	    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.50 - 1.25)) + 1.25) ;
																	    				basic_point.z =0.00;
																	    						 cloud->points.push_back(basic_point);
																	  }

																			  srand ( time(NULL) );
																				  for (int j=0 ; j <= 100; j++)
																		  {
																		    pcl::PointXYZRGBA basic_point;
																		    basic_point.x = 0.00;
																		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.50 - 1.25)) + 1.25) ;
																		    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.25- 0.00)) + 0.00) ;
																		    						 cloud->points.push_back(basic_point);
																		  }
																				  srand ( time(NULL) );
																					  for (int j=0 ; j <= 100; j++)
																			  {
																			    pcl::PointXYZRGBA basic_point;
																			    basic_point.x =(((((float) rand()) / (float) RAND_MAX) * (0.25- 0.00)) + 0.00) ;
																			    		basic_point.y = 1.25;
																			    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.25- 0.00)) + 0.00) ;
																			    						 cloud->points.push_back(basic_point);
																			  }
																					  srand ( time(NULL) );
																					  for (int i=0 ; i <= 100; i++ )
																				{
																				    pcl::PointXYZRGBA basic_point;
																				    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.25- 0.00)) + 0.00) ;
																				    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.50 - 1.25)) + 1.25) ;
																				    				basic_point.z =0.25;
																				    						 cloud->points.push_back(basic_point);
																				  }

																						  srand ( time(NULL) );
																							  for (int j=0 ; j <= 100; j++)
																					  {
																					    pcl::PointXYZRGBA basic_point;
																					    basic_point.x = 0.25;
																					    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.50 - 1.25)) + 1.25) ;
																					    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.25- 0.00)) + 0.00) ;
																					    						 cloud->points.push_back(basic_point);
																					  }
																							  srand ( time(NULL) );
																								  for (int j=0 ; j <= 100; j++)
																						  {
																						    pcl::PointXYZRGBA basic_point;
																						    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.25- 0.00)) + 0.00) ;
																						    		basic_point.y = 1.50;
																						    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.25- 0.00)) + 0.00) ;
																						    						 cloud->points.push_back(basic_point);
																						  }




			  cloud->width = (int) cloud->points.size ();
			  cloud->height = 1;

			  for (size_t i = 0; i < cloud ->points.size (); ++i) {
			    		cloud->points[i].r = 255;
			    		cloud->points[i].g = 255;
			    		cloud->points[i].b = 255;
			  }

			  //  // write point cloud to disk
			   pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/StairsCube.pcd", *cloud);
			   //Write PLY
			   pcl::PLYWriter writePLY;
			   writePLY.write ("/Path/TO/ArtificialPointClouds/StairsCube.ply", *cloud,  false);
				std::cout << "Number of points in the input cloud is:"<< cloud->points.size() << std::endl;

			     // Show the cloud
			   pcl::visualization::CloudViewer viewer("Sharp Edge");
			   viewer.showCloud(cloud);
			  while (!viewer.wasStopped ())
			  {}


	  return 0;
}

#endif

#ifdef FRACTAL_SEAT

#include <iostream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include "time.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char*argv[])
{
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )    // Cube 10K , 40 ------> For more density CUbe 30K , i<= 120     // Or We Can Make the size of the cube 3 times less , the size of the cube is between 0 and 0.3
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    				basic_point.z =0.00;
    				basic_point.r = 255;
    				basic_point.g = 255;
    				basic_point.b= 255;
    						 cloud->points.push_back(basic_point);
  }
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    		basic_point.y = 0.00;
    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    				basic_point.r = 255;
    				basic_point.g = 255;
    				basic_point.b= 255;
    						 cloud->points.push_back(basic_point);
  }
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = 0.00;
    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    				basic_point.r = 255;
    				basic_point.g = 255;
    				basic_point.b= 255;
    						 cloud->points.push_back(basic_point);
  }
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.25- 0.00)) + 0.00) ;
    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    				basic_point.z =0.50;
    				basic_point.r = 255;
    				basic_point.g = 255;
    				basic_point.b= 255;
    						 cloud->points.push_back(basic_point);
  }

	  srand ( time(NULL) );
	  for (int i=0 ; i <= 400; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.25)) + 0.25) ;
    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.25- 0.00)) + 0.00) ;
    				basic_point.z =0.50;
    				basic_point.r = 255;
    				basic_point.g = 255;
    				basic_point.b= 255;
    						 cloud->points.push_back(basic_point);
  }





	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    		basic_point.y = 0.50;
    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.25- 0.00)) + 0.00) ;
    				basic_point.r = 255;
    				basic_point.g = 255;
    				basic_point.b= 255;
    						 cloud->points.push_back(basic_point);
  }
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 400; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.25- 0.00)) + 0.00) ;
    		basic_point.y = 0.50;
    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.25)) + 0.25) ;
    				basic_point.r = 255;
    				basic_point.g = 255;
    				basic_point.b= 255;
    						 cloud->points.push_back(basic_point);
  }





	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = 0.50;
    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.25- 0.00)) + 0.00) ;
    				basic_point.r = 255;
    				basic_point.g = 255;
    				basic_point.b= 255;
    						 cloud->points.push_back(basic_point);
  }
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 400; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = 0.50;
    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.25- 0.00)) + 0.00) ;
    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.25)) + 0.25) ;
    				basic_point.r = 255;
    				basic_point.g = 255;
    				basic_point.b= 255;
    						 cloud->points.push_back(basic_point);
  }
// Second Small Cube Inside the big cube
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 400; i++ )    // Cube 10K , 40 ------> For more density CUbe 30K , i<= 120     // Or We Can Make the size of the cube 3 times less , the size of the cube is between 0 and 0.3
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.25)) + 0.25) ;
    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.25)) + 0.25) ;
    				basic_point.z =0.25;
    				basic_point.r = 255;
    				basic_point.g = 255;
    				basic_point.b= 255;
    						 cloud->points.push_back(basic_point);
  }
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 400; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.25)) + 0.25) ;
    		basic_point.y = 0.25;
    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.25)) + 0.25) ;
    				basic_point.r = 255;
    				basic_point.g = 255;
    				basic_point.b= 255;
    						 cloud->points.push_back(basic_point);
  }
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 400; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = 0.25;
    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.25)) + 0.25) ;
    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.25)) + 0.25) ;
    				basic_point.r = 255;
    				basic_point.g = 255;
    				basic_point.b= 255;
    						 cloud->points.push_back(basic_point);
  }
//	  srand ( time(NULL) );
//	  for (int i=0 ; i <= 400; i++ )
//{
//    pcl::PointXYZRGBA basic_point;
//    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.50)) + 0.50) ;
//    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.50)) + 0.50) ;
//    				basic_point.z =1.00;
//    				basic_point.r = 255;
//    				basic_point.g = 0;
//    				basic_point.b= 0;
//    						 cloud->points.push_back(basic_point);
//  }
//	  srand ( time(NULL) );
//	  for (int i=0 ; i <= 400; i++ )
//{
//    pcl::PointXYZRGBA basic_point;
//    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.50)) + 0.50) ;
//    		basic_point.y = 1.00;
//    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.50)) + 0.50) ;
//    				basic_point.r = 255;
//    				basic_point.g = 0;
//    				basic_point.b= 0;
//    						 cloud->points.push_back(basic_point);
//  }
//	  srand ( time(NULL) );
//	  for (int i=0 ; i <= 400; i++ )
//{
//    pcl::PointXYZRGBA basic_point;
//    basic_point.x = 1.00;
//    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.50)) + 0.50) ;
//    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.50)) + 0.50) ;
//    				basic_point.r = 255;
//    				basic_point.g = 0;
//    				basic_point.b= 0;
//    						 cloud->points.push_back(basic_point);
//  }


	  cloud->width = (int) cloud->points.size ();
	  cloud->height = 1;

	  // remove 3 surface of inside pertain to small cube
//	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Secondcloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
//	  for (size_t i = 0; i < cloud ->points.size (); ++i) {
//		 if (   )
//
//	  }


//  // write point cloud to disk
 pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/CubeFractal2.pcd", *cloud);
 //Write PLY
 pcl::PLYWriter writePLY;
 writePLY.write ("/Path/TO/ArtificialPointClouds/CubeFractal2.ply", *cloud,  false);
	std::cout << "Number of points in the input cloud is:"<< cloud->points.size() << std::endl;

   // Show the cloud
 pcl::visualization::CloudViewer viewer("Sharp Edge");
 viewer.showCloud(cloud);
while (!viewer.wasStopped ())
{}


return 0;
}
#endif

#ifdef FRACTAL_THREE_CUBE

#include <iostream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include "time.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char*argv[])
{
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	  //1. First Cube
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = 0.00;
    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.50)) + 0.50) ;
    						 cloud->points.push_back(basic_point);
  }
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    		basic_point.y = 0.00;
    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.50)) + 0.50) ;
    						 cloud->points.push_back(basic_point);
  }

	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    				basic_point.z =0.50;
    						 cloud->points.push_back(basic_point);
  }

	  // 2. First Cube
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = 0.50;
    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.50)) + 0.50) ;
    						 cloud->points.push_back(basic_point);
  }
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    		basic_point.y = 0.50;
    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.50)) + 0.50) ;
    						 cloud->points.push_back(basic_point);
  }
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    				basic_point.z =1.00;
    						 cloud->points.push_back(basic_point);
  }
	  //1. Second Cube
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = 0.50;
    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    						 cloud->points.push_back(basic_point);
  }
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.50)) + 0.50) ;
    		basic_point.y = 0.00;
    				basic_point.z = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    						 cloud->points.push_back(basic_point);
  }

	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.50)) + 0.50) ;
    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    				basic_point.z =0.00;
    						 cloud->points.push_back(basic_point);
  }

	  // 2. Second Cube
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = 1.00;
    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    						 cloud->points.push_back(basic_point);
  }
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.50)) + 0.50) ;
    		basic_point.y = 0.50;
    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    						 cloud->points.push_back(basic_point);
  }
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.50)) + 0.50) ;
    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    				basic_point.z =0.50;
    						 cloud->points.push_back(basic_point);
  }
	  //1. Third Cube
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = 0.00;
    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.50)) + 0.50) ;
    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    						 cloud->points.push_back(basic_point);
  }
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    		basic_point.y = 0.50;
    				basic_point.z = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    						 cloud->points.push_back(basic_point);
  }

	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.50)) + 0.50) ;
    				basic_point.z =0.00;
    						 cloud->points.push_back(basic_point);
  }

	  // 2. Third Cube
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = 0.50;
    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.50)) + 0.50) ;
    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    						 cloud->points.push_back(basic_point);
  }
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    		basic_point.y = 1.00;
    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    						 cloud->points.push_back(basic_point);
  }
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ )
{
    pcl::PointXYZRGBA basic_point;
    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.50)) + 0.50) ;
    				basic_point.z =0.50;
    						 cloud->points.push_back(basic_point);
  }



cloud->width = (int) cloud->points.size ();
cloud->height = 1;
std::cout << "Number of points in the input cloud is:"<< cloud->points.size() << std::endl;


for (size_t i = 0; i < cloud ->points.size (); ++i) {
  		cloud->points[i].r = 255;
  		cloud->points[i].g = 255;
  		cloud->points[i].b = 255;
}
//  // write point cloud to disk
 pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/ThreeCubeFractal3.pcd", *cloud);
 //Write PLY
 pcl::PLYWriter writePLY;
 writePLY.write ("/Path/TO/ArtificialPointClouds/ThreeCubeFractal3.ply", *cloud,  false);


   // Show the cloud
 pcl::visualization::CloudViewer viewer("Sharp Edge");
 viewer.showCloud(cloud);
while (!viewer.wasStopped ())
{}


return 0;
}
#endif


#ifdef FRACTAL_FOUR_HOLE

#include <iostream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include "time.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char*argv[])
{
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	  //1. First Cube
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ ) {
		  pcl::PointXYZRGBA basic_point;
		    basic_point.x = 0.00;
		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
		    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
		    						 cloud->points.push_back(basic_point);}
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ ) {
		  pcl::PointXYZRGBA basic_point;
		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
		    		basic_point.y = 0.00;
		    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
		    						 cloud->points.push_back(basic_point);}
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ ) {
		  pcl::PointXYZRGBA basic_point;
		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
		    				basic_point.z =0.00;
		    						 cloud->points.push_back(basic_point);}
// 2.  First Cube
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ ) {
		  pcl::PointXYZRGBA basic_point;
		    basic_point.x = 0.50;
		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
		    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
		    						 cloud->points.push_back(basic_point);}
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ ) {
		  pcl::PointXYZRGBA basic_point;
		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
		    		basic_point.y = 0.50;
		    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
		    						 cloud->points.push_back(basic_point);}
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 800; i++ ) {
		  pcl::PointXYZRGBA basic_point;
		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
		    				basic_point.z =0.50;
		    						 cloud->points.push_back(basic_point);}

	  //1. Second Cube
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 400; i++ ) {
		  pcl::PointXYZRGBA basic_point;
		    basic_point.x = 0.167;
		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.334- 0.167)) + 0.167) ;
		    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (00.334- 0.167)) + 0.167) ;
		    						 cloud->points.push_back(basic_point);}
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 400; i++ ) {
		  pcl::PointXYZRGBA basic_point;
		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.334- 0.167)) + 0.167) ;
		    		basic_point.y = 0.167;
		    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (00.334- 0.167)) + 0.167) ;
		    						 cloud->points.push_back(basic_point);}
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 400; i++ ) {
		  pcl::PointXYZRGBA basic_point;
		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.334- 0.167)) + 0.167) ;
		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.334- 0.167)) + 0.167) ;
		    				basic_point.z =0.167;
		    						 cloud->points.push_back(basic_point);}
	  //2. Second Cube
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 400; i++ ) {
		  pcl::PointXYZRGBA basic_point;
		    basic_point.x = 0.334;
		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.334- 0.167)) + 0.167) ;
		    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (00.334- 0.167)) + 0.167) ;
		    						 cloud->points.push_back(basic_point);}
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 400; i++ ) {
		  pcl::PointXYZRGBA basic_point;
		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.334- 0.167)) + 0.167) ;
		    		basic_point.y = 0.334;
		    				basic_point.z =(((((float) rand()) / (float) RAND_MAX) * (00.334- 0.167)) + 0.167) ;
		    						 cloud->points.push_back(basic_point);}
	  srand ( time(NULL) );
	  for (int i=0 ; i <= 400; i++ ) {
		  pcl::PointXYZRGBA basic_point;
		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (0.334- 0.167)) + 0.167) ;
		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.334- 0.167)) + 0.167) ;
		    				basic_point.z =0.334 ;
		    						 cloud->points.push_back(basic_point);}






cloud->width = (int) cloud->points.size ();
cloud->height = 1;
std::cout << "Number of points in the input cloud is:"<< cloud->points.size() << std::endl;


for (size_t i = 0; i < cloud ->points.size (); ++i) {
  		cloud->points[i].r = 255;
  		cloud->points[i].g = 255;
  		cloud->points[i].b = 255;
}
//  // write point cloud to disk
 pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/FourCubeFractalHole.pcd", *cloud);
 //Write PLY
 pcl::PLYWriter writePLY;
 writePLY.write ("/Path/TO/ArtificialPointClouds/FourCubeFractalHole.ply", *cloud,  false);


   // Show the cloud
 pcl::visualization::CloudViewer viewer("Sharp Edge");
 viewer.showCloud(cloud);
while (!viewer.wasStopped ())
{}


return 0;
}
#endif


#ifdef TETRAHEDRON

#include <iostream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include "time.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char*argv[])
{
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	  // defining an empty 2D vector (matrix) for points of lines of tetrahedron
	   	 int height=100;
	   	 int width=3;
// LineXY
	  std::vector<std::vector <double> > LinXY;
	  LinXY.resize(height); // to defining numbers of row in 2D vector
 	    for(int jj=0; jj< height; ++jj)
 	    	{LinXY[jj].resize(width);}   // to defining numbers of column for each row in 2d vector
 	   // LineXZ
 		  std::vector<std::vector <double> > LinXZ;
 		  LinXZ.resize(height); // to defining numbers of row in 2D vector
 	 	    for(int jj=0; jj< height; ++jj)
 	 	    	{LinXZ[jj].resize(width);}   // to defining numbers of column for each row in 2d vector
	  // LineYZ
		  std::vector<std::vector <double> > LinYZ;
		  LinYZ.resize(height); // to defining numbers of row in 2D vector
	 	    for(int jj=0; jj< height; ++jj)
	 	    	{LinYZ[jj].resize(width);}   // to defining numbers of column for each row in 2d vector


//1. line XY
	  srand ( time(NULL) );
	  for (int i=0 ; i < height; i++ ) {
		  double x = (((((double) rand()) / (double) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
		  double y = - (x) +1 ;
		  double z = 0.00 ;
		    				LinXY[i][0] = x; LinXY[i][1] = y;  LinXY[i][2] = z; }
//2.  line XZ
	  srand ( time(NULL) );
	  for (int i=0 ; i < height; i++ ) {
		  double x = (((((double) rand()) / (double) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
		  double y = 0.00 ;
		  double z = - (x ) +1 ;
		    				LinXZ[i][0] = x; LinXZ[i][1] = y;  LinXZ[i][2] = z; }

//3.  line YZ
	  srand ( time(NULL) );
	  for (int i=0 ; i < height; i++ ) {
		  double x = 0.00;
		  double y = (((((double) rand()) / (double) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
		  double z = - (y) +1 ;
		    				LinYZ[i][0] = x; LinYZ[i][1] = y;  LinYZ[i][2] = z; }


	  std::cout << "lines are done" << std::endl;


// First Triangle Between  p1 (0,0,0)  and p2 (1,0,0) and p3 (0,1,0) XY line
// (0.999 - 0.001)) + 0.001
	  	  srand ( time(NULL) );
	  	  for (int i=0 ; i <= 200; i++ ) {

	  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
	  		float y1 = 0.00 ;
	  		float x2 = 0.00 ;
	  		float y2 = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;;

	  		float m = ( ( y2 - y1) / (x2- x1) ) ;
	  		float k = y2 - (m * x2 ) ;

	  for (int j =0 ; j <= 20; j++ ) {
	  		  pcl::PointXYZRGBA basic_point;
	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 0.00)) + 0.00) ;
	  		    		basic_point.y = (m * (basic_point.x )) + k;
	  		    				basic_point.z = 0.00 ;
	  		    						 cloud->points.push_back(basic_point);
	  	          } // For Line between two points
	  	  } //for 2 points

	  	  srand ( time(NULL) );
	  	  for (int i=0 ; i <= 20; i++ ) {

	  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.85)) + 0.85) ;
	  		float y1 = 0.00 ;
	  		float x2 = 0.00 ;
	  		float y2 =  x1 ;      // (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.85)) + 0.85) ;;

	  		float m = ( ( y2 - y1) / (x2- x1) ) ;
	  		float k = y2 - (m * x2 ) ;

	  for (int j =0 ; j <= 20; j++ ) {
	  		  pcl::PointXYZRGBA basic_point;
	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 0.00)) + 0.00) ;
	  		    		basic_point.y = (m * (basic_point.x )) + k;
	  		    				basic_point.z = 0.00 ;
	  		    						 cloud->points.push_back(basic_point);
	  	          } // For Line between two points
	  	  } //for 2 points





// Second Triangle Between p1 (0,0,0)  and p2 (1,0,0) and p3 (0,0,1) 	 XZ Line
	  	  srand ( time(NULL) );
	  	  for (int i=0 ; i <= 200; i++ ) {

	  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
	  		float z1 = 0.00 ;
	  		float x2 = 0.00 ;
	  		float z2 = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;;

	  		float m = ( ( z2 - z1) / (x2- x1) ) ;
	  		float k = z2 - (m * x2 ) ;

	  for (int j =0 ; j <= 20; j++ ) {
	  		  pcl::PointXYZRGBA basic_point;
	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 0.00)) + 0.00) ;
	  		    		basic_point.y = 0.00;
	  		    				basic_point.z = (m * (basic_point.x )) + k;
	  		    						 cloud->points.push_back(basic_point);
	  	          } // For Line between two points
	  	  } //for 2 points

	  	  srand ( time(NULL) );
	  	  for (int i=0 ; i <= 20; i++ ) {

	  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.85)) + 0.85) ;
	  		float z1 = 0.00 ;
	  		float x2 = 0.00 ;
	  		float z2 = x1; ///(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.85)) + 0.85) ;;

	  		float m = ( ( z2 - z1) / (x2- x1) ) ;
	  		float k = z2 - (m * x2 ) ;

	  for (int j =0 ; j <= 20; j++ ) {
	  		  pcl::PointXYZRGBA basic_point;
	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 0.00)) + 0.00) ;
	  		    		basic_point.y = 0.00;
	  		    				basic_point.z = (m * (basic_point.x )) + k;
	  		    						 cloud->points.push_back(basic_point);
	  	          } // For Line between two points
	  	  } //for 2 points

// Third Triangle Between p1 (0,0,0)  and p2 (0,1,0) and p3 (0,0,1) 	YZ Line
	  	  srand ( time(NULL) );
	  	  for (int i=0 ; i <= 200; i++ ) {

	  	    float y1 =(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
	  		float z1 = 0.00 ;
	  		float y2 = 0.00 ;
	  		float z2 = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;;

	  		float m = ( ( z2 - z1) / (y2- y1) ) ;
	  		float k = z2 - (m * y2 ) ;

	  for (int j =0 ; j <= 20; j++ ) {
	  		  pcl::PointXYZRGBA basic_point;
	  		    basic_point.x = 0.00 ;
	  		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (y1- 0.00)) + 0.00) ;
	  		    				basic_point.z = (m * (basic_point.y )) + k;
	  		    						 cloud->points.push_back(basic_point);
	  	          } // For Line between two points
	  	  } //for 2 points

	  	  srand ( time(NULL) );
	  	  for (int i=0 ; i <= 20; i++ ) {

	  	    float y1 =(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.85)) + 0.85) ;
	  		float z1 = 0.00 ;
	  		float y2 = 0.00 ;
	  		float z2 = y1;  // (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.85)) + 0.85) ;;

	  		float m = ( ( z2 - z1) / (y2- y1) ) ;
	  		float k = z2 - (m * y2 ) ;

	  for (int j =0 ; j <= 20; j++ ) {
	  		  pcl::PointXYZRGBA basic_point;
	  		    basic_point.x = 0.00 ;
	  		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (y1- 0.00)) + 0.00) ;
	  		    				basic_point.z = (m * (basic_point.y )) + k;
	  		    						 cloud->points.push_back(basic_point);
	  	          } // For Line between two points
	  	  } //for 2 points

// Fourth Triangle Between p1 (1,0,0)  and p2 (0,1,0) and p3 (0,0,1) 	  XYZ Line
      	// random_shuffle(begin(LinXZ), end(LinXZ));
	  	  // Equation of the plane X+Y+Z+1 = 0
//	  for (int j =0 ; j < 500; j++ ) {
//	  int i = 	rand() % height	  ;
//	  // line XZ
//	  float 	x1 = LinXZ[i][0];
//	  float	y1 = LinXZ[i][1];
//	  float	z1 = LinXZ[i][2] ;
//// Line YZ
//	  float x2 = LinYZ[i][0] ;
//	  float y2 = LinYZ[i][1] ;
//	  float z2 = LinYZ[i][2] ;




		 for (int j =0 ; j <= 2300; j++ ) {
		  		  pcl::PointXYZRGBA basic_point;
//		  		    basic_point.x = x1 ;
//		  		    		basic_point.y = y2 ;
//		  		    	basic_point.z = std::abs (z2 - z1 );

			  		      basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (1.00 - 0.00)) + 0.00) ;
			  		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
		  		    		//if ( z2 > z1) { basic_point.z = (((((float) rand()) / (float) RAND_MAX) * (z2- z1)) + z1) ;}
		  		    		//else {basic_point.z = (((((float) rand()) / (float) RAND_MAX) * (z1- z2)) + z2) ; }
float ZZ = (-(basic_point.x) - (basic_point.y )) + 1 ;
if ( ZZ >= 0.00 && ZZ <= 1.00 )
{	basic_point.z = ZZ ;

		  		    						 cloud->points.push_back(basic_point); }
		  	        //  } // For Line between two points
	  	} // for select a coordinate of line


cloud->width = (int) cloud->points.size ();
cloud->height = 1;
std::cout << "Number of points in the input cloud is:"<< cloud->points.size() << std::endl;


for (size_t i = 0; i < cloud ->points.size (); ++i) {
  		cloud->points[i].r = 255;
  		cloud->points[i].g = 255;
  		cloud->points[i].b = 255;
}
//  // write point cloud to disk
 pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/Tetrahedron.pcd", *cloud);
 //Write PLY
 pcl::PLYWriter writePLY;
 writePLY.write ("/Path/TO/ArtificialPointClouds/Tetrahedron.ply", *cloud,  false);


   // Show the cloud
 pcl::visualization::CloudViewer viewer("Sharp Edge");
 viewer.showCloud(cloud);
while (!viewer.wasStopped ())
{}


return 0;
}



#endif


#ifdef TETRAHEDRON_MULTIPLE

#include <iostream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include "time.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char*argv[])
{
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	  // defining an empty 2D vector (matrix) for points of lines of tetrahedron
	   	 int height=100;
	   	 int width=3;
// LineXY
	  std::vector<std::vector <double> > LinXY;
	  LinXY.resize(height); // to defining numbers of row in 2D vector
 	    for(int jj=0; jj< height; ++jj)
 	    	{LinXY[jj].resize(width);}   // to defining numbers of column for each row in 2d vector
 	   // LineXZ
 		  std::vector<std::vector <double> > LinXZ;
 		  LinXZ.resize(height); // to defining numbers of row in 2D vector
 	 	    for(int jj=0; jj< height; ++jj)
 	 	    	{LinXZ[jj].resize(width);}   // to defining numbers of column for each row in 2d vector
	  // LineYZ
		  std::vector<std::vector <double> > LinYZ;
		  LinYZ.resize(height); // to defining numbers of row in 2D vector
	 	    for(int jj=0; jj< height; ++jj)
	 	    	{LinYZ[jj].resize(width);}   // to defining numbers of column for each row in 2d vector

// *******************************************************************************************************
// ************************************FIRST THETRAHEDRON*********************************************
 // *******************************************************************************************************

//1. line XY
	  srand ( time(NULL) );
	  for (int i=0 ; i < height; i++ ) {
		  double x = (((((double) rand()) / (double) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
		  double y = - (x) +1 ;
		  double z = 0.00 ;
		    				LinXY[i][0] = x; LinXY[i][1] = y;  LinXY[i][2] = z; }
//2.  line XZ
	  srand ( time(NULL) );
	  for (int i=0 ; i < height; i++ ) {
		  double x = (((((double) rand()) / (double) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
		  double y = 0.00 ;
		  double z = - (x ) +1 ;
		    				LinXZ[i][0] = x; LinXZ[i][1] = y;  LinXZ[i][2] = z; }

//3.  line YZ
	  srand ( time(NULL) );
	  for (int i=0 ; i < height; i++ ) {
		  double x = 0.00;
		  double y = (((((double) rand()) / (double) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
		  double z = - (y) +1 ;
		    				LinYZ[i][0] = x; LinYZ[i][1] = y;  LinYZ[i][2] = z; }


	  std::cout << "lines are done" << std::endl;


// First Triangle Between  p1 (0,0,0)  and p2 (1,0,0) and p3 (0,1,0) XY line
// (0.999 - 0.001)) + 0.001
	  	  srand ( time(NULL) );
	  	  for (int i=0 ; i <= 200; i++ ) {

	  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
	  		float y1 = 0.00 ;
	  		float x2 = 0.00 ;
	  		float y2 = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;;

	  		float m = ( ( y2 - y1) / (x2- x1) ) ;
	  		float k = y2 - (m * x2 ) ;

	  for (int j =0 ; j <= 20; j++ ) {
	  		  pcl::PointXYZRGBA basic_point;
	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 0.00)) + 0.00) ;
	  		    		basic_point.y = (m * (basic_point.x )) + k;
	  		    				basic_point.z = 0.00 ;
	  		    						 cloud->points.push_back(basic_point);
	  	          } // For Line between two points
	  	  } //for 2 points

	  	  srand ( time(NULL) );
	  	  for (int i=0 ; i <= 20; i++ ) {

	  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.85)) + 0.85) ;
	  		float y1 = 0.00 ;
	  		float x2 = 0.00 ;
	  		float y2 =  x1 ;      // (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.85)) + 0.85) ;;

	  		float m = ( ( y2 - y1) / (x2- x1) ) ;
	  		float k = y2 - (m * x2 ) ;

	  for (int j =0 ; j <= 20; j++ ) {
	  		  pcl::PointXYZRGBA basic_point;
	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 0.00)) + 0.00) ;
	  		    		basic_point.y = (m * (basic_point.x )) + k;
	  		    				basic_point.z = 0.00 ;
	  		    						 cloud->points.push_back(basic_point);
	  	          } // For Line between two points
	  	  } //for 2 points





// Second Triangle Between p1 (0,0,0)  and p2 (1,0,0) and p3 (0,0,1) 	 XZ Line
	  	  srand ( time(NULL) );
	  	  for (int i=0 ; i <= 200; i++ ) {

	  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
	  		float z1 = 0.00 ;
	  		float x2 = 0.00 ;
	  		float z2 = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;;

	  		float m = ( ( z2 - z1) / (x2- x1) ) ;
	  		float k = z2 - (m * x2 ) ;

	  for (int j =0 ; j <= 20; j++ ) {
	  		  pcl::PointXYZRGBA basic_point;
	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 0.00)) + 0.00) ;
	  		    		basic_point.y = 0.00;
	  		    				basic_point.z = (m * (basic_point.x )) + k;
	  		    						 cloud->points.push_back(basic_point);
	  	          } // For Line between two points
	  	  } //for 2 points

	  	  srand ( time(NULL) );
	  	  for (int i=0 ; i <= 20; i++ ) {

	  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.85)) + 0.85) ;
	  		float z1 = 0.00 ;
	  		float x2 = 0.00 ;
	  		float z2 = x1; ///(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.85)) + 0.85) ;;

	  		float m = ( ( z2 - z1) / (x2- x1) ) ;
	  		float k = z2 - (m * x2 ) ;

	  for (int j =0 ; j <= 20; j++ ) {
	  		  pcl::PointXYZRGBA basic_point;
	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 0.00)) + 0.00) ;
	  		    		basic_point.y = 0.00;
	  		    				basic_point.z = (m * (basic_point.x )) + k;
	  		    						 cloud->points.push_back(basic_point);
	  	          } // For Line between two points
	  	  } //for 2 points

// Third Triangle Between p1 (0,0,0)  and p2 (0,1,0) and p3 (0,0,1) 	YZ Line
	  	  srand ( time(NULL) );
	  	  for (int i=0 ; i <= 200; i++ ) {

	  	    float y1 =(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
	  		float z1 = 0.00 ;
	  		float y2 = 0.00 ;
	  		float z2 = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;;

	  		float m = ( ( z2 - z1) / (y2- y1) ) ;
	  		float k = z2 - (m * y2 ) ;

	  for (int j =0 ; j <= 20; j++ ) {
	  		  pcl::PointXYZRGBA basic_point;
	  		    basic_point.x = 0.00 ;
	  		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (y1- 0.00)) + 0.00) ;
	  		    				basic_point.z = (m * (basic_point.y )) + k;
	  		    						 cloud->points.push_back(basic_point);
	  	          } // For Line between two points
	  	  } //for 2 points

	  	  srand ( time(NULL) );
	  	  for (int i=0 ; i <= 20; i++ ) {

	  	    float y1 =(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.85)) + 0.85) ;
	  		float z1 = 0.00 ;
	  		float y2 = 0.00 ;
	  		float z2 = y1;  // (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.85)) + 0.85) ;;

	  		float m = ( ( z2 - z1) / (y2- y1) ) ;
	  		float k = z2 - (m * y2 ) ;

	  for (int j =0 ; j <= 20; j++ ) {
	  		  pcl::PointXYZRGBA basic_point;
	  		    basic_point.x = 0.00 ;
	  		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (y1- 0.00)) + 0.00) ;
	  		    				basic_point.z = (m * (basic_point.y )) + k;
	  		    						 cloud->points.push_back(basic_point);
	  	          } // For Line between two points
	  	  } //for 2 points

// Fourth Triangle Between p1 (1,0,0)  and p2 (0,1,0) and p3 (0,0,1) 	  XYZ Line
      	// random_shuffle(begin(LinXZ), end(LinXZ));
	  	  // Equation of the plane X+Y+Z+1 = 0
//	  for (int j =0 ; j < 500; j++ ) {
//	  int i = 	rand() % height	  ;
//	  // line XZ
//	  float 	x1 = LinXZ[i][0];
//	  float	y1 = LinXZ[i][1];
//	  float	z1 = LinXZ[i][2] ;
//// Line YZ
//	  float x2 = LinYZ[i][0] ;
//	  float y2 = LinYZ[i][1] ;
//	  float z2 = LinYZ[i][2] ;




		 for (int j =0 ; j <= 2300; j++ ) {
		  		  pcl::PointXYZRGBA basic_point;
//		  		    basic_point.x = x1 ;
//		  		    		basic_point.y = y2 ;
//		  		    	basic_point.z = std::abs (z2 - z1 );

			  		      basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (1.00 - 0.00)) + 0.00) ;
			  		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
		  		    		//if ( z2 > z1) { basic_point.z = (((((float) rand()) / (float) RAND_MAX) * (z2- z1)) + z1) ;}
		  		    		//else {basic_point.z = (((((float) rand()) / (float) RAND_MAX) * (z1- z2)) + z2) ; }
float ZZ = (-(basic_point.x) - (basic_point.y )) + 1 ;
if ( ZZ >= 0.00 && ZZ <= 1.00 )
{	basic_point.z = ZZ ;

		  		    						 cloud->points.push_back(basic_point); }
		  	        //  } // For Line between two points
	  	} // for select a coordinate of line


		 // *******************************************************************************************************
		 // ************************************SecondTHETRAHEDRON*********************************************
		  // *******************************************************************************************************

		 //1. line XY
		 	  srand ( time(NULL) );
		 	  for (int i=0 ; i < height; i++ ) {
		 		  double x = (((((double) rand()) / (double) RAND_MAX) * (1.50- 1.00)) + 1.00) ;
		 		  double y = - (x) +0.5 ;
		 		  double z = 0.00 ;
		 		    				LinXY[i][0] = x; LinXY[i][1] = y;  LinXY[i][2] = z; }
		 //2.  line XZ
		 	  srand ( time(NULL) );
		 	  for (int i=0 ; i < height; i++ ) {
		 		  double x = (((((double) rand()) / (double) RAND_MAX) * (1.50- 1.00)) + 1.00) ;
		 		  double y = 0.00 ;
		 		  double z = - (x ) +0.5 ;
		 		    				LinXZ[i][0] = x; LinXZ[i][1] = y;  LinXZ[i][2] = z; }

		 //3.  line YZ
		 	  srand ( time(NULL) );
		 	  for (int i=0 ; i < height; i++ ) {
		 		  double x = 1.00;
		 		  double y = (((((double) rand()) / (double) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
		 		  double z = - (y) +0.5 ;
		 		    				LinYZ[i][0] = x; LinYZ[i][1] = y;  LinYZ[i][2] = z; }


		 	  std::cout << "lines are done" << std::endl;


		 // First Triangle Between  p1 (0,0,0)  and p2 (1,0,0) and p3 (0,1,0) XY line
		 // (0.999 - 0.001)) + 0.001
		 	  	  srand ( time(NULL) );
		 	  	  for (int i=0 ; i <= 200; i++ ) {

		 	  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (1.50- 1.00)) + 1.00) ;
		 	  		float y1 = 0.00 ;
		 	  		float x2 = 1.00 ;
		 	  		float y2 = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;;

		 	  		float m = ( ( y2 - y1) / (x2- x1) ) ;
		 	  		float k = y2 - (m * x2 ) ;

		 	  for (int j =0 ; j <= 20; j++ ) {
		 	  		  pcl::PointXYZRGBA basic_point;
		 	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 1.00)) + 1.00) ;
		 	  		    		basic_point.y = (m * (basic_point.x )) + k;
		 	  		    				basic_point.z = 0.00 ;
		 	  		    						 cloud->points.push_back(basic_point);
		 	  	          } // For Line between two points
		 	  	  } //for 2 points

		 	  	  srand ( time(NULL) );
		 	  	  for (int i=0 ; i <= 20; i++ ) {

		 	  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (1.50- 1.35)) + 1.35) ;
		 	  		float y1 = 0.00 ;
		 	  		float x2 = 1.00 ;
		 	  		float y2 = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.35)) + 0.35) ;

		 	  		float m = ( ( y2 - y1) / (x2- x1) ) ;
		 	  		float k = y2 - (m * x2 ) ;

		 	  for (int j =0 ; j <= 20; j++ ) {
		 	  		  pcl::PointXYZRGBA basic_point;
		 	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 1.00)) + 1.00) ;
		 	  		    		basic_point.y = (m * (basic_point.x )) + k;
		 	  		    				basic_point.z = 0.00 ;
		 	  		    						 cloud->points.push_back(basic_point);
		 	  	          } // For Line between two points
		 	  	  } //for 2 points




		 // Second Triangle Between p1 (0,0,0)  and p2 (1,0,0) and p3 (0,0,1) 	 XZ Line
		 	  	  srand ( time(NULL) );
		 	  	  for (int i=0 ; i <= 200; i++ ) {

		 	  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (1.50- 1.00)) + 1.00) ;
		 	  		float z1 = 0.00 ;
		 	  		float x2 = 1.00 ;
		 	  		float z2 = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;;

		 	  		float m = ( ( z2 - z1) / (x2- x1) ) ;
		 	  		float k = z2 - (m * x2 ) ;

		 	  for (int j =0 ; j <= 20; j++ ) {
		 	  		  pcl::PointXYZRGBA basic_point;
		 	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 1.00)) + 1.00) ;
		 	  		    		basic_point.y = 0.00;
		 	  		    				basic_point.z = (m * (basic_point.x )) + k;
		 	  		    						 cloud->points.push_back(basic_point);
		 	  	          } // For Line between two points
		 	  	  } //for 2 points

		 	  	  srand ( time(NULL) );
		 	  	  for (int i=0 ; i <= 20; i++ ) {

		 	  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (1.50- 1.35)) + 1.35) ;
		 	  		float z1 = 0.00 ;
		 	  		float x2 = 1.00 ;
		 	  		float z2 = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.35)) + 0.35) ;

		 	  		float m = ( ( z2 - z1) / (x2- x1) ) ;
		 	  		float k = z2 - (m * x2 ) ;

		 	  for (int j =0 ; j <= 20; j++ ) {
		 	  		  pcl::PointXYZRGBA basic_point;
		 	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 1.00)) + 1.00) ;
		 	  		    		basic_point.y = 0.00;
		 	  		    				basic_point.z = (m * (basic_point.x )) + k;
		 	  		    						 cloud->points.push_back(basic_point);
		 	  	          } // For Line between two points
		 	  	  } //for 2 points


// Third Triangle Between p1 (0,0,0)  and p2 (0,1,0) and p3 (0,0,1) 	YZ Line
		 	  	  srand ( time(NULL) );
		 	  	  for (int i=0 ; i <= 200; i++ ) {

		 	  	    float y1 =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
		 	  		float z1 = 0.00 ;
		 	  		float y2 = 0.00 ;
		 	  		float z2 = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) +0.00) ;;

		 	  		float m = ( ( z2 - z1) / (y2- y1) ) ;
		 	  		float k = z2 - (m * y2 ) ;

		 	  for (int j =0 ; j <= 20; j++ ) {
		 	  		  pcl::PointXYZRGBA basic_point;
		 	  		    basic_point.x = 1.00 ;
		 	  		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (y1- 0.00)) + 0.00) ;
		 	  		    				basic_point.z = (m * (basic_point.y )) + k;
		 	  		    						 cloud->points.push_back(basic_point);
		 	  	          } // For Line between two points
		 	  	  } //for 2 points

		 	  	  srand ( time(NULL) );
		 	  	  for (int i=0 ; i <= 20; i++ ) {

		 	  	    float y1 =(((((float) rand()) / (float) RAND_MAX) * (0.50- 0.35)) + 0.35) ;
		 	  		float z1 = 0.00 ;
		 	  		float y2 = 0.00 ;
		 	  		float z2 = y1;  // (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.85)) + 0.85) ;;

		 	  		float m = ( ( z2 - z1) / (y2- y1) ) ;
		 	  		float k = z2 - (m * y2 ) ;

		 	  for (int j =0 ; j <= 20; j++ ) {
		 	  		  pcl::PointXYZRGBA basic_point;
		 	  		    basic_point.x = 1.00 ;
		 	  		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (y1- 0.00)) + 0.00) ;
		 	  		    				basic_point.z = (m * (basic_point.y )) + k;
		 	  		    						 cloud->points.push_back(basic_point);
		 	  	          } // For Line between two points
		 	  	  } //for 2 points


// Fourth Triangle
		 		 for (int j =0 ; j <= 2300; j++ ) {
		 		  		  pcl::PointXYZRGBA basic_point;
		 			  		      basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (1.50 - 1.00)) + 1.00) ;
		 			  		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (0.50- 0.00)) + 0.00) ;
		 float ZZ = (-(basic_point.x) - (basic_point.y )) + 0.5 ;
		 if ( ZZ >= 0.00 && ZZ <= 0.50 )
		 {	basic_point.z = ZZ ;

		 		  		    						 cloud->points.push_back(basic_point); }
		 		  	        //  } // For Line between two points
		 	  	} // for select a coordinate of line


				 // *******************************************************************************************************
				 // ************************************ThirdTHETRAHEDRON*********************************************
				  // *******************************************************************************************************


				 // First Triangle Between  p1 (0,0,0)  and p2 (1,0,0) and p3 (0,1,0) XY line
				 // (0.999 - 0.001)) + 0.001
				 	  	  srand ( time(NULL) );
				 	  	  for (int i=0 ; i <= 300; i++ ) {

				 	  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (1.50- 0.00)) + 0.00) ;
				 	  		float y1 = 0.00 ;
				 	  		float x2 = 0.00 ;
				 	  		float y2 = (((((float) rand()) / (float) RAND_MAX) * (1.50- 0.00)) + 0.00) ;;

				 	  		float m = ( ( y2 - y1) / (x2- x1) ) ;
				 	  		float k = y2 - (m * x2 ) ;

				 	  for (int j =0 ; j <= 50; j++ ) {
				 	  		  pcl::PointXYZRGBA basic_point;
				 	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 0.00)) + 0.00) ;
				 	  		    		basic_point.y = (m * (basic_point.x )) + k;
				 	  		    				basic_point.z = 1.00 ;
				 	  		    						 cloud->points.push_back(basic_point);
				 	  	          } // For Line between two points
				 	  	  } //for 2 points

				 	  	  srand ( time(NULL) );
				 	  	  for (int i=0 ; i <= 50; i++ ) {

				 	  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (1.50- 1.15)) + 1.15) ;
				 	  		float y1 = 0.00 ;
				 	  		float x2 = 0.00 ;
				 	  		float y2 = (((((float) rand()) / (float) RAND_MAX) * (1.50- 1.15)) + 1.15) ;

				 	  		float m = ( ( y2 - y1) / (x2- x1) ) ;
				 	  		float k = y2 - (m * x2 ) ;

				 	  for (int j =0 ; j <= 20; j++ ) {
				 	  		  pcl::PointXYZRGBA basic_point;
				 	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 0.00)) + 0.00) ;
				 	  		    		basic_point.y = (m * (basic_point.x )) + k;
				 	  		    				basic_point.z = 1.00 ;
				 	  		    						 cloud->points.push_back(basic_point);
				 	  	          } // For Line between two points
				 	  	  } //for 2 points




				 // Second Triangle Between p1 (0,0,0)  and p2 (1,0,0) and p3 (0,0,1) 	 XZ Line
				 	  	  srand ( time(NULL) );
				 	  	  for (int i=0 ; i <= 300; i++ ) {

				 	  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (1.50- 0.00)) + 0.00) ;
				 	  		float z1 = 1.00 ;
				 	  		float x2 = 0.00 ;
				 	  		float z2 = (((((float) rand()) / (float) RAND_MAX) * (2.50- 1.00)) + 1.00) ;;

				 	  		float m = ( ( z2 - z1) / (x2- x1) ) ;
				 	  		float k = z2 - (m * x2 ) ;

				 	  for (int j =0 ; j <= 50; j++ ) {
				 	  		  pcl::PointXYZRGBA basic_point;
				 	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 0.00)) + 0.00) ;
				 	  		    		basic_point.y = 0.00;
				 	  		    				basic_point.z = (m * (basic_point.x )) + k;
				 	  		    						 cloud->points.push_back(basic_point);
				 	  	          } // For Line between two points
				 	  	  } //for 2 points

				 	  	  srand ( time(NULL) );
				 	  	  for (int i=0 ; i <= 50; i++ ) {

				 	  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (1.50- 1.15)) + 1.15) ;
				 	  		float z1 = 1.00 ;
				 	  		float x2 = 0.00 ;
				 	  		float z2 = (((((float) rand()) / (float) RAND_MAX) * (2.50- 2.15)) + 2.15) ;

				 	  		float m = ( ( z2 - z1) / (x2- x1) ) ;
				 	  		float k = z2 - (m * x2 ) ;

				 	  for (int j =0 ; j <= 20; j++ ) {
				 	  		  pcl::PointXYZRGBA basic_point;
				 	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 0.00)) + 0.00) ;
				 	  		    		basic_point.y = 0.00;
				 	  		    				basic_point.z = (m * (basic_point.x )) + k;
				 	  		    						 cloud->points.push_back(basic_point);
				 	  	          } // For Line between two points
				 	  	  } //for 2 points


		// Third Triangle Between p1 (0,0,0)  and p2 (0,1,0) and p3 (0,0,1) 	YZ Line
				 	  	  srand ( time(NULL) );
				 	  	  for (int i=0 ; i <= 300; i++ ) {

				 	  	    float y1 =(((((float) rand()) / (float) RAND_MAX) * (1.50- 0.00)) + 0.00) ;
				 	  		float z1 = 1.00 ;
				 	  		float y2 = 0.00 ;
				 	  		float z2 = (((((float) rand()) / (float) RAND_MAX) * (2.50- 1.00)) +1.00) ;;

				 	  		float m = ( ( z2 - z1) / (y2- y1) ) ;
				 	  		float k = z2 - (m * y2 ) ;

				 	  for (int j =0 ; j <= 50; j++ ) {
				 	  		  pcl::PointXYZRGBA basic_point;
				 	  		    basic_point.x = 0.00 ;
				 	  		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (y1- 0.00)) + 0.00) ;
				 	  		    				basic_point.z = (m * (basic_point.y )) + k;
				 	  		    						 cloud->points.push_back(basic_point);
				 	  	          } // For Line between two points
				 	  	  } //for 2 points

				 	  	  srand ( time(NULL) );
				 	  	  for (int i=0 ; i <= 50; i++ ) {

				 	  	    float y1 =(((((float) rand()) / (float) RAND_MAX) * (1.50- 1.15)) + 1.15) ;
				 	  		float z1 = 1.00 ;
				 	  		float y2 = 0.00 ;
				 	  		float z2 = (((((float) rand()) / (float) RAND_MAX) * (2.50- 2.15)) + 2.15) ;;

				 	  		float m = ( ( z2 - z1) / (y2- y1) ) ;
				 	  		float k = z2 - (m * y2 ) ;

				 	  for (int j =0 ; j <= 20; j++ ) {
				 	  		  pcl::PointXYZRGBA basic_point;
				 	  		    basic_point.x = 0.00 ;
				 	  		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (y1- 0.00)) + 0.00) ;
				 	  		    				basic_point.z = (m * (basic_point.y )) + k;
				 	  		    						 cloud->points.push_back(basic_point);
				 	  	          } // For Line between two points
				 	  	  } //for 2 points


		// Fourth Triangle
				 		 for (int j =0 ; j <= 2300; j++ ) {
				 		  		  pcl::PointXYZRGBA basic_point;
				 			  		      basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (1.50 - 0.00)) + 0.00) ;
				 			  		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.50- 0.00)) + 0.00) ;
				 float ZZ = (-(basic_point.x) - (basic_point.y )) + 1.5 ;
				 if ( ZZ >= 1.00 && ZZ <= 2.50 )
				 {	basic_point.z = ZZ ;

				 		  		    						 cloud->points.push_back(basic_point); }
				 		  	        //  } // For Line between two points
				 	  	} // for select a coordinate of line


						 // *******************************************************************************************************
						 // ************************************FourthTHETRAHEDRON*********************************************
						  // *******************************************************************************************************


						 // First Triangle Between  p1 (0,0,0)  and p2 (1,0,0) and p3 (0,1,0) XY line
						 // (0.999 - 0.001)) + 0.001
						 	  	  srand ( time(NULL) );
						 	  	  for (int i=0 ; i <= 400; i++ ) {

						 	  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (2.00- 0.00)) + 0.00) ;
						 	  		float y1 = 3.00 ;
						 	  		float x2 = 0.00 ;
						 	  		float y2 = (((((float) rand()) / (float) RAND_MAX) * (3.00- 0.00)) + 1.00) ;;

						 	  		float m = ( ( y2 - y1) / (x2- x1) ) ;
						 	  		float k = y2 - (m * x2 ) ;

						 	  for (int j =0 ; j <= 80; j++ ) {
						 	  		  pcl::PointXYZRGBA basic_point;
						 	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 0.00)) + 0.00) ;
						 	  		    if (  ((m * (basic_point.x )) + k) >=1 &&   ((m * (basic_point.x )) + k) <=3){
						 	  		    		basic_point.y = (m * (basic_point.x )) + k;
						 	  		    				basic_point.z = 0.00 ;
						 	  		    						 cloud->points.push_back(basic_point);}
						 	  	          } // For Line between two points
						 	  	  } //for 2 points

						 	  	  srand ( time(NULL) );
						 	  	  for (int i=0 ; i <= 80; i++ ) {

						 	  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (2.00- 1.65)) + 1.65) ;
						 	  		float y1 = 3.00 ;
						 	  		float x2 = 0.00 ;
						 	  		float y2 = (((((float) rand()) / (float) RAND_MAX) * (3.00- 2.65)) + 2.65) ;

						 	  		float m = ( ( y2 - y1) / (x2- x1) ) ;
						 	  		float k = y2 - (m * x2 ) ;

						 	  for (int j =0 ; j <= 20; j++ ) {
						 	  		  pcl::PointXYZRGBA basic_point;
						 	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 1.65)) + 1.65) ;
						 	  		    if (  ((m * (basic_point.x )) + k) >=1.00 &&   ((m * (basic_point.x )) + k) <=3.00){
						 	  		    		basic_point.y = (m * (basic_point.x )) + k;
						 	  		    				basic_point.z = 0.00 ;
						 	  		    						 cloud->points.push_back(basic_point);}
						 	  	          } // For Line between two points
						 	  	  } //for 2 points




						 // Second Triangle Between p1 (0,0,0)  and p2 (1,0,0) and p3 (0,0,1) 	 XZ Line
						 	  	  srand ( time(NULL) );
						 	  	  for (int i=0 ; i <= 400; i++ ) {

						 	  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (2.00- 0.00)) + 0.00) ;
						 	  		float z1 = 0.00 ;
						 	  		float x2 = 0.00 ;
						 	  		float z2 = (((((float) rand()) / (float) RAND_MAX) * (2.00- 0.00)) + 0.00) ;;

						 	  		float m = ( ( z2 - z1) / (x2- x1) ) ;
						 	  		float k = z2 - (m * x2 ) ;

						 	  for (int j =0 ; j <= 80; j++ ) {
						 	  		  pcl::PointXYZRGBA basic_point;
						 	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 0.00)) + 0.00) ;
						 	  		    		basic_point.y = 3.00;
						 	  		    		if ( ((m * (basic_point.x )) + k)>=0.00 && ((m * (basic_point.x )) + k)<=2.00 ){
						 	  		    				basic_point.z = (m * (basic_point.x )) + k;
						 	  		    						 cloud->points.push_back(basic_point);}
						 	  	          } // For Line between two points
						 	  	  } //for 2 points

						 	  	  srand ( time(NULL) );
						 	  	  for (int i=0 ; i <= 80; i++ ) {

						 	  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (2.00- 1.65)) + 1.65) ;
						 	  		float z1 = 0.00 ;
						 	  		float x2 = 0.00 ;
						 	  		float z2 = (((((float) rand()) / (float) RAND_MAX) * (2.00- 1.65)) + 1.65) ;

						 	  		float m = ( ( z2 - z1) / (x2- x1) ) ;
						 	  		float k = z2 - (m * x2 ) ;

						 	  for (int j =0 ; j <= 20; j++ ) {
						 	  		  pcl::PointXYZRGBA basic_point;
						 	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 1.65)) + 1.65) ;
						 	  		    		basic_point.y = 3.00;
						 	  		    		if ( ((m * (basic_point.x )) + k)>=0.00 && ((m * (basic_point.x )) + k)<=2.00 ){
						 	  		    				basic_point.z = (m * (basic_point.x )) + k;
						 	  		    						 cloud->points.push_back(basic_point);}
						 	  	          } // For Line between two points
						 	  	  } //for 2 points




				// Third Triangle Between p1 (0,0,0)  and p2 (0,1,0) and p3 (0,0,1) 	YZ Line
						 	  	  srand ( time(NULL) );
						 	  	  for (int i=0 ; i <= 400; i++ ) {

						 	  	    float y1 =(((((float) rand()) / (float) RAND_MAX) * (3.00- 1.00)) + 1.00) ;
						 	  		float z1 = 0.00 ;
						 	  		float y2 = 3.00 ;
						 	  		float z2 = (((((float) rand()) / (float) RAND_MAX) * (2.00- 0.00)) +0.00) ;

						 	  		float m = ( ( z2 - z1) / (y2- y1) ) ;
						 	  		float k = z2 - (m * y2 ) ;

						 	  for (int j =0 ; j <= 80; j++ ) {
						 	  		  pcl::PointXYZRGBA basic_point;
						 	  		    basic_point.x = 0.00 ;
						 	  		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (3.00- y1)) + y1) ;
						 	  		    		//std::cout << "basic_point.y is"<< basic_point.y << std::endl;
						 	  		    		//std::cout << "z is"<< (m * (basic_point.y )) + k << std::endl;
						 	  		    		if ( ( ((m * (basic_point.y )) + k)>= 0.00) && (((m * (basic_point.y )) + k)<= 2.00)) {
						 	  		    			  basic_point.z =  ((m * (basic_point.y )) + k);
						 	  		    						 cloud->points.push_back(basic_point);}
						 	  	          } // For Line between two points
						 	  	  } //for 2 points

						 	  	  srand ( time(NULL) );
						 	  	  for (int i=0 ; i <= 80; i++ ) {

						 	  	    float y1 =(((((float) rand()) / (float) RAND_MAX) * (3.00- 2.65)) + 2.65) ;
						 	  		float z1 = 0.00 ;
						 	  		float y2 = 3.00 ;
						 	  		float z2 = (((((float) rand()) / (float) RAND_MAX) * (2.00- 1.65)) + 1.65) ;;

						 	  		float m = ( ( z2 - z1) / (y2- y1) ) ;
						 	  		float k = z2 - (m * y2 ) ;

						 	  for (int j =0 ; j <= 20; j++ ) {
						 	  		  pcl::PointXYZRGBA basic_point;
						 	  		    basic_point.x = 0.00 ;
						 	  		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (3.00- y1)) + y1) ;
						 	  		    		if ( ( (m * (basic_point.y )) + k)>= 0.00 && ((m * (basic_point.y )) + k)<= 2.00  ){
						 	  		    			// std::cout << "there is Z"<< std::endl;
						 	  		    				basic_point.z = (m * (basic_point.y )) + k;
						 	  		    						 cloud->points.push_back(basic_point);}
						 	  	          } // For Line between two points
						 	  	  } //for 2 points





				// Fourth Triangle
						 		 for (int j =0 ; j <= 2300; j++ ) {
						 		  		  pcl::PointXYZRGBA basic_point;
						 			  		      basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (2.00 - 0.00)) + 0.00) ;
						 			  		    		basic_point.z = (((((float) rand()) / (float) RAND_MAX) * (2.00- 0.00)) + 0.00) ;
						 float YY = (-(basic_point.x) - (basic_point.z )) + 2.00 ;
						 if ( YY >= 1.00 && YY <= 3.00 )
						 {	basic_point.z = YY ;

						 		  		    						 cloud->points.push_back(basic_point); }
						 		  	        //  } // For Line between two points
						 	  	} // for select a coordinate of line






cloud->width = (int) cloud->points.size ();
cloud->height = 1;
std::cout << "Number of points in the input cloud is:"<< cloud->points.size() << std::endl;


for (size_t i = 0; i < cloud ->points.size (); ++i) {
  		cloud->points[i].r = 255;
  		cloud->points[i].g = 255;
  		cloud->points[i].b = 255;
}
//  // write point cloud to disk
 pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/TetrahedronMultiple.pcd", *cloud);
 //Write PLY
 pcl::PLYWriter writePLY;
 writePLY.write ("/Path/TO/ArtificialPointClouds/TetrahedronMultiple.ply", *cloud,  false);


   // Show the cloud
 pcl::visualization::CloudViewer viewer("Sharp Edge");
 viewer.showCloud(cloud);
while (!viewer.wasStopped ())
{}


return 0;
}


#endif


#ifdef TETRAHEDRON_PARTITE

#include <iostream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include "time.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char*argv[])
{
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);


  	  for (int i=0 ; i <= 200; i++ ) {

  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
  		float z1 = 0.00 ;
  		float x2 = 0.00 ;
  		float z2 = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;;

  		float m = ( ( z2 - z1) / (x2- x1) ) ;
  		float k = z2 - (m * x2 ) ;

  for (int j =0 ; j <= 20; j++ ) {
  		  pcl::PointXYZRGBA basic_point;
  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 0.00)) + 0.00) ;
  		    		basic_point.y = 0.00;
  		    				basic_point.z = (m * (basic_point.x )) + k;
  		    						 cloud->points.push_back(basic_point);
  	          } // For Line between two points
  	  } //for 2 points

  	  srand ( time(NULL) );
  	  for (int i=0 ; i <= 20; i++ ) {

  	    float x1 =(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.85)) + 0.85) ;
  		float z1 = 0.00 ;
  		float x2 = 0.00 ;
  		float z2 = x1; ///(((((float) rand()) / (float) RAND_MAX) * (1.00- 0.85)) + 0.85) ;;

  		float m = ( ( z2 - z1) / (x2- x1) ) ;
  		float k = z2 - (m * x2 ) ;

  for (int j =0 ; j <= 20; j++ ) {
  		  pcl::PointXYZRGBA basic_point;
  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (x1- 0.00)) + 0.00) ;
  		    		basic_point.y = 0.00;
  		    				basic_point.z = (m * (basic_point.x )) + k;
  		    						 cloud->points.push_back(basic_point);
  	          } // For Line between two points
  	  } //for 2 points



		 for (int j =0 ; j <= 2300; j++ ) {
		  		  pcl::PointXYZRGBA basic_point;
//		  		    basic_point.x = x1 ;
//		  		    		basic_point.y = y2 ;
//		  		    	basic_point.z = std::abs (z2 - z1 );

			  		      basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (1.00 - 0.00)) + 0.00) ;
			  		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
		  		    		//if ( z2 > z1) { basic_point.z = (((((float) rand()) / (float) RAND_MAX) * (z2- z1)) + z1) ;}
		  		    		//else {basic_point.z = (((((float) rand()) / (float) RAND_MAX) * (z1- z2)) + z2) ; }
float ZZ = (-(basic_point.x) - (basic_point.y )) + 1 ;
if ( ZZ >= 0.00 && ZZ <= 1.00 )
{	basic_point.z = ZZ ;

		  		    						 cloud->points.push_back(basic_point); }
		  	        //  } // For Line between two points
	  	} // for select a coordinate of line


cloud->width = (int) cloud->points.size ();
cloud->height = 1;
std::cout << "Number of points in the input cloud is:"<< cloud->points.size() << std::endl;


for (size_t i = 0; i < cloud ->points.size (); ++i) {
  		cloud->points[i].r = 255;
  		cloud->points[i].g = 255;
  		cloud->points[i].b = 255;
}
//  // write point cloud to disk
 pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/TetrahedronPartite.pcd", *cloud);
 //Write PLY
 pcl::PLYWriter writePLY;
 writePLY.write ("/Path/TO/ArtificialPointClouds/TetrahedronPartite.ply", *cloud,  false);


   // Show the cloud
 pcl::visualization::CloudViewer viewer("Sharp Edge");
 viewer.showCloud(cloud);
while (!viewer.wasStopped ())
{}


return 0;
}

#endif

#ifdef TWO_PLANE

#include <iostream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include "time.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char*argv[])
{
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

for (int j =0 ; j < 30000; j++ ) {
		  pcl::PointXYZRGBA basic_point;
		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
		    				basic_point.z = 0.00;
		    						 cloud->points.push_back(basic_point);
	  } //for 2 points

for (int j =0 ; j < 30000; j++ ) {
		  pcl::PointXYZRGBA basic_point;
		    basic_point.x = 0.00;
			 basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ; // 90
		    // basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
		    				 // basic_point.z = 0.5 * (basic_point.x) ;    // 22.5
		    				// basic_point.z = 1.00 * (basic_point.x) ;  	// 45
		    				// basic_point.z = 0.75 * (basic_point.x) ; // 67.5
		    				basic_point.z = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;  	//90
		    						 cloud->points.push_back(basic_point);
	  } //for 2 points


cloud->width = (int) cloud->points.size ();
cloud->height = 1;
std::cout << "Number of points in the input cloud is:"<< cloud->points.size() << std::endl;


for (size_t i = 0; i < cloud ->points.size (); ++i) {
  		cloud->points[i].r = 255;
  		cloud->points[i].g = 255;
  		cloud->points[i].b = 255;
}
// write red point cloud to disk
pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/TwoPlaneLarge90.pcd", *cloud);
 // Show the cloud
pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/TwoPlaneLarge90.pcd", *cloud);
pcl::visualization::CloudViewer viewer(" TwoPlane ");
viewer.showCloud(cloud);
while (!viewer.wasStopped ())
{}
                        return (0);
                       }
#endif

#ifdef INTERSECTION_TWOPLANE
// Three plane

#include <iostream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include "time.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>


int
main (int argc, char*argv[])
{
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	  for (int j =0 ; j < 2000; j++ ) {
	  		  pcl::PointXYZRGBA basic_point;
	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
	  		    		basic_point.y =  0.00;
	  		    				basic_point.z = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
	  		    						 cloud->points.push_back(basic_point);
	  	  } //for XZ Plane X and Z between 0 and 1 and y equal to 0

	  for (int j =0 ; j < 2000; j++ ) {
	  		  pcl::PointXYZRGBA basic_point;
	  		    basic_point.x = 0.50;
	  			 basic_point.y = ((((float) rand()) / (float) RAND_MAX) * (0.50- (-0.50)) + (-0.50)) ; // 90
	  		    basic_point.z = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;  	//90
	  		    						 cloud->points.push_back(basic_point);
	  	  } // X is equal to 0.5 and y is between -0.5 and 0.5 and z is between 0 and 1

	  for (int j =0 ; j < 2000; j++ ) {
	  		  pcl::PointXYZRGBA basic_point;
	  		    basic_point.x =  (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
	  			 basic_point.y = ((((float) rand()) / (float) RAND_MAX) * (0.50- (-0.50)) + (-0.50)) ;
	  		    basic_point.z =  0.50;
	  		    						 cloud->points.push_back(basic_point);
	  	  } // X is is between 0 and 1 and y is between -0.5 and 0.5 and z is equal to 0.5



cloud->width = (int) cloud->points.size ();
cloud->height = 1;
std::cout << "Number of points in the input cloud is:"<< cloud->points.size() << std::endl;


for (size_t i = 0; i < cloud ->points.size (); ++i) {
  		cloud->points[i].r = 255;
  		cloud->points[i].g = 255;
  		cloud->points[i].b = 255;
}
// write red point cloud to disk
pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/IntersectionTwoPlanes.pcd", *cloud);
 // Show the cloud
pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/IntersectionTwoPlanes.pcd", *cloud);
pcl::visualization::CloudViewer viewer(" Intersection TwoPlanes ");
viewer.showCloud(cloud);
while (!viewer.wasStopped ())
{}
                        return (0);
                       }
#endif


#ifdef INTERSECTION_THREEPLANE_DIFFERENTSIZES
// Three plane

#include <iostream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include "time.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>


int
main (int argc, char*argv[])
{
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	  for (int j =0 ; j < 2000; j++ ) {
	  		  pcl::PointXYZRGBA basic_point;
	  		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
	  		    		basic_point.y =  0.00;
	  		    				basic_point.z = (((((float) rand()) / (float) RAND_MAX) * (0.70- 0.00)) + 0.00) ;
	  		    						 cloud->points.push_back(basic_point);
	  	  } //for XZ Plane X and Z between 0 and 1 and y equal to 0

	  for (int j =0 ; j < 2000; j++ ) {
	  		  pcl::PointXYZRGBA basic_point;
	  		    basic_point.x = 0.50;
	  			 basic_point.y = ((((float) rand()) / (float) RAND_MAX) * (0.50- (-0.50)) + (-0.50)) ; // 90
	  		    basic_point.z = (((((float) rand()) / (float) RAND_MAX) * (0.80- 0.00)) + 0.00) ;  	//90
	  		    						 cloud->points.push_back(basic_point);
	  	  } // X is equal to 0.5 and y is between -0.5 and 0.5 and z is between 0 and 1

	  for (int j =0 ; j < 2000; j++ ) {
	  		  pcl::PointXYZRGBA basic_point;
	  		    basic_point.x =  (((((float) rand()) / (float) RAND_MAX) * (0.95- 0.00)) + 0.00) ;
	  			 basic_point.y = ((((float) rand()) / (float) RAND_MAX) * (0.50- (-0.50)) + (-0.50)) ;
	  		    basic_point.z =  0.50;
	  		    						 cloud->points.push_back(basic_point);
	  	  } // X is is between 0 and 1 and y is between -0.5 and 0.5 and z is equal to 0.5



cloud->width = (int) cloud->points.size ();
cloud->height = 1;
std::cout << "Number of points in the input cloud is:"<< cloud->points.size() << std::endl;


for (size_t i = 0; i < cloud ->points.size (); ++i) {
  		cloud->points[i].r = 255;
  		cloud->points[i].g = 255;
  		cloud->points[i].b = 255;
}
// write red point cloud to disk
pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/IntersectionThreePlanesDiffernt.pcd", *cloud);
 // Show the cloud
pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/IntersectionThreePlanesDiffernt.pcd", *cloud);
pcl::visualization::CloudViewer viewer(" Intersection TwoPlanes ");
viewer.showCloud(cloud);
while (!viewer.wasStopped ())
{}
                        return (0);
                       }
#endif





# ifdef ONE_PLANE

#include <iostream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include "time.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char*argv[])
{
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

for (int j =0 ; j <= 3000; j++ ) {
		  pcl::PointXYZRGBA basic_point;
		    basic_point.x = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
		    		basic_point.y = (((((float) rand()) / (float) RAND_MAX) * (1.00- 0.00)) + 0.00) ;
		    				basic_point.z = 0.00;
		    						 cloud->points.push_back(basic_point);
	  } //for 2 points


cloud->width = (int) cloud->points.size ();
cloud->height = 1;
std::cout << "Number of points in the input cloud is:"<< cloud->points.size() << std::endl;

for (size_t i = 0; i < cloud ->points.size (); ++i) {
  		cloud->points[i].r = 255;
  		cloud->points[i].g = 255;
  		cloud->points[i].b = 255;
}
// write red point cloud to disk
pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/OnePlane.pcd", *cloud);
 // Show the cloud
pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/OnePlane.pcd", *cloud);
pcl::PLYWriter writePLY;
writePLY.write ("/Path/TO/ArtificialPointClouds/AddingNoise/OnePlane.ply", *cloud,  false);
pcl::visualization::CloudViewer viewer(" TwoPlane ");
viewer.showCloud(cloud);
while (!viewer.wasStopped ())
{}
                        return (0);
                       }
#endif

#ifdef ADD_NOISE
#include <iostream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include "time.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

int
main (int argc, char*argv[])
{
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Noisycloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

      // pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/OnePlane.pcd", *cloud);
      // pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/CubeSharpEdge.pcd", *cloud);
      pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/CubeFractal2.pcd", *cloud);

      std::cout << "Number of points in the input cloud is:"<< cloud->points.size() << std::endl;

 	 Noisycloud->resize(cloud ->points.size () );

      // creat kdtree
     		 pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
     		 kdtree.setInputCloud (cloud);
     		 pcl::PointXYZRGBA searchPoint;
     		 // K nearest neighbor search
     		 int NumbersNeighbor = 12; // numbers of neighbors 7
     		 std::vector<int> NeighborsKNSearch(NumbersNeighbor);
     		 std::vector<float> NeighborsKNSquaredDistance(NumbersNeighbor);

     		 double* StndDevX = new  double [cloud->points.size() ];
     		 double* StndDevY = new  double [cloud->points.size() ];
     		 double* StndDevZ = new  double [cloud->points.size() ];

     		 //All the Points of the cloud
     		 for (size_t i = 0; i < cloud ->points.size (); ++i) {
     		 searchPoint.x =   cloud->points[i].x;
     		 searchPoint.y =   cloud->points[i].y;
     		 searchPoint.z =   cloud->points[i].z;

     		 if ( kdtree.nearestKSearch (searchPoint, NumbersNeighbor, NeighborsKNSearch, NeighborsKNSquaredDistance) > 0 ) {
     		 NumbersNeighbor = NeighborsKNSearch.size (); }
     		   else { NumbersNeighbor = 0; }

   // computing VAriance
     		 double sumX = 0.00 ;  double sumY = 0.00 ;  double sumZ = 0.00 ;
    		 for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii) {
    			 sumX  += (cloud->points[ NeighborsKNSearch[ii] ].x);
    			 sumY  += (cloud->points[ NeighborsKNSearch[ii] ].y);
    			 sumZ  += (cloud->points[ NeighborsKNSearch[ii] ].z);
     		  } // For each neighbor of the query point
    	double AvgX = sumX / NeighborsKNSearch.size () ;	  double AvgY = sumY / NeighborsKNSearch.size () ;	   double AvgZ= sumZ / NeighborsKNSearch.size () ;
    	  sumX = 0.00 ;   sumY = 0.00 ;   sumZ = 0.00 ;
    		 for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii) {
    			 sumX += ( ((cloud->points[ NeighborsKNSearch[ii] ].x) - AvgX) * ((cloud->points[ NeighborsKNSearch[ii] ].x) - AvgX) ) ;
    			 sumY += ( ((cloud->points[ NeighborsKNSearch[ii] ].y) - AvgY) * ((cloud->points[ NeighborsKNSearch[ii] ].y) - AvgY) ) ;
    			 sumZ += ( ((cloud->points[ NeighborsKNSearch[ii] ].z) - AvgZ) * ((cloud->points[ NeighborsKNSearch[ii] ].z) - AvgZ) ) ;
    		           } // For each neighbor of the query point
    		 StndDevX [i] = sqrt(sumX  /  ( NeighborsKNSearch.size () - 1)) ;
    		 StndDevY [i] = sqrt(sumY  /  ( NeighborsKNSearch.size () - 1)) ;
    		 StndDevZ [i] = sqrt(sumZ  /  ( NeighborsKNSearch.size () - 1)) ;
     		 } // For each Point of the Cloud

    // First copy all the points of main cloud to the noisy cloud
      		 for (size_t i = 0; i < Noisycloud ->points.size (); ++i ) {
        		 // Noisy point cloud
      			Noisycloud->points[i].x = cloud->points[i].x;
      			Noisycloud->points[i].y = cloud->points[i].y;
      			Noisycloud->points[i].z = cloud->points[i].z;
      			Noisycloud->points[i].r = 255;
      			Noisycloud->points[i].g = 255;
      			Noisycloud->points[i].b = 255;
      		 }
// Then add noise to each 10 point of the cloud
      		 for (size_t i = 0; i < Noisycloud ->points.size (); i+=10 ) {
        		 // Noisy point cloud
      			Noisycloud->points[i].x = cloud->points[i].x + (7* StndDevX[i]);
      			Noisycloud->points[i].y = cloud->points[i].y + (7* StndDevY[i]);
      			Noisycloud->points[i].z = cloud->points[i].z + (7* StndDevZ[i]);
      			Noisycloud->points[i].r = 255;
      			Noisycloud->points[i].g = 255;
      			Noisycloud->points[i].b = 255;
      		 }



     		std::cout << "Number of points in the Noisy cloud is:"<< Noisycloud->points.size() << std::endl;
// write red point cloud to disk
pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/AddingNoise/Frac2withNoise710.pcd", *Noisycloud);
pcl::PLYWriter writePLY;
writePLY.write ("/Path/TO/ArtificialPointClouds/AddingNoise/Frac2withNoise710.ply", *Noisycloud,  false);
 // Show the cloud
//pcl::visualization::CloudViewer viewer(" ONePlaneWithNoise ");
//viewer.showCloud(Noisycloud);
//while (!viewer.wasStopped ())
//{}
                        return (0);
                       }

#endif



#ifdef SacModel
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/sample_consensus/sac_model_plane.h>
//#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

int
main (int argc, char** argv)
{

	// initialize PointClouds
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	  // populate our PointCloud with points
	  cloud->width    = 500;
	  cloud->height   = 1;
	  cloud->is_dense = false;
	  cloud->points.resize (cloud->width * cloud->height);
	  for (size_t i = 0; i < cloud->points.size (); ++i)
	  {
	    if (pcl::console::find_argument (argc, argv, "-s") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
	    {
	      cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
	      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
	      if (i % 5 == 0)
	        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
	      else if(i % 2 == 0)
	        cloud->points[i].z =  sqrt( 1 - (cloud->points[i].x * cloud->points[i].x)
	                                      - (cloud->points[i].y * cloud->points[i].y));
	      else
	        cloud->points[i].z =  - sqrt( 1 - (cloud->points[i].x * cloud->points[i].x)
	                                        - (cloud->points[i].y * cloud->points[i].y));
	    }
	    else
	    {
	      cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
	      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
	      if( i % 2 == 0)
	        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
	      else
	        cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
	    }
	  }


// write red point cloud to disk
pcl::io::savePCDFile ("/Path/TO/ArtificialPointClouds/SacModel.pcd", *cloud);
 // Show the cloud
pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/SacModel.pcd", *cloud);
pcl::visualization::CloudViewer viewer(" Cube ");
viewer.showCloud(cloud);
while (!viewer.wasStopped ())
{}
                        return (0);
                       }
#endif

