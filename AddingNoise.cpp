/*
 * AddingNoise.cpp
 *
 *  Created on: May 22, 2015
 *      Author: dbazazian
 */


// #define STANDARD_DEVIATION_NEIGHBORS
#define GAUSSIAN_NOISE

#ifdef  STANDARD_DEVIATION_NEIGHBORS
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

      // pcl::io::loadPCDFile ("path/to/OnePlane.pcd", *cloud);
      // pcl::io::loadPCDFile ("path/to/CubeSharpEdge.pcd", *cloud);
      pcl::io::loadPCDFile ("path/to/CubeFractal2.pcd", *cloud);

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
pcl::io::savePCDFile ("path/to//AddingNoise/Frac2withNoise710.pcd", *Noisycloud);
pcl::PLYWriter writePLY;
writePLY.write ("path/to//AddingNoise/Frac2withNoise710.ply", *Noisycloud,  false);
 // Show the cloud
//pcl::visualization::CloudViewer viewer(" ONePlaneWithNoise ");
//viewer.showCloud(Noisycloud);
//while (!viewer.wasStopped ())
//{}
                        return (0);
                       }


#endif



#ifdef GAUSSIAN_NOISE

#include <iostream>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <math.h>
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


#define PI 3.14159265

int
main (int argc, char*argv[])
{
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Noisycloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

      // pcl::io::loadPCDFile ("path/to/OnePlane.pcd", *cloud);
      // pcl::io::loadPCDFile ("path/to/CubeSharpEdge.pcd", *cloud);
       //    pcl::io::loadPCDFile ("path/to/CubeFractal2.pcd", *cloud);
          // pcl::io::loadPCDFile ("path/to/TwoPlane22.pcd", *cloud);
          // pcl::io::loadPCDFile ("path/to/bunny.pcd",*cloud);
          pcl::io::loadPCDFile ("path/to/Tetrahedron.pcd", *cloud);


      std::cout << "Number of points in the input cloud is:"<< cloud->points.size() << std::endl;

 	 Noisycloud->resize(cloud ->points.size () );

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



		 double* GussNosX = new  double [cloud->points.size() ];
		 double* GussNosY = new  double [cloud->points.size() ];
		 double* GussNosZ = new  double [cloud->points.size() ];


		 double variance = 0.3 ;
		 double mean = 0.00;

 	// http://forums.codeguru.com/showthread.php?459963-Adding-noise-to-image

		 //All the Points of the cloud
		 for (size_t i = 0; i < cloud ->points.size (); i+=35) { // 10 , 50
			  double u1 = (((((float) rand()) / (float) RAND_MAX) * (0.003 - 0.00)) + 0.00) ;
			  double u2 = (((((float) rand()) / (float) RAND_MAX) * (0.003- 0.00)) + 0.00) ;
// if (u1 > 0.005 ) u1 = 0.005;
//			  temp = sqrt(-2.0*variance*log(u1));
//			  tempint = p[ix][iy] + (int) (temp * sin(TWO_PI*u2) + mean);
			  double temp = sqrt(-2.0*variance*log(u1));
			  double tempin = (temp * sin(2* PI *u2) + mean);

			  Noisycloud->points[i].x = cloud->points[i].x + tempin;
			  Noisycloud->points[i].y = cloud->points[i].y + tempin;
			  Noisycloud->points[i].z = cloud->points[i].z + tempin;

		 }// For each point


 // Then add noise to each 10 point of the cloud
//       		 for (size_t i = 0; i < Noisycloud ->points.size (); i+=10 ) {
//         		 // Noisy point cloud
//       			Noisycloud->points[i].x = cloud->points[i].x + (7* StndDevX[i]);
//       			Noisycloud->points[i].y = cloud->points[i].y + (7* StndDevY[i]);
//       			Noisycloud->points[i].z = cloud->points[i].z + (7* StndDevZ[i]);
//       			Noisycloud->points[i].r = 255;
//       			Noisycloud->points[i].g = 255;
//       			Noisycloud->points[i].b = 255;
//       		 }



  		std::cout << "Number of points in the Noisy cloud is:"<< Noisycloud->points.size() << std::endl;
// write red point cloud to disk
  		// pcl::io::savePCDFile ("path/to/AddingNoise/Two22plane14Noise.pcd", *Noisycloud);
  		// pcl::io::savePCDFile ("path/to/AddingNoise/Bunny03Noise50.pcd", *Noisycloud);
  		pcl::io::savePCDFile ("path/to/AddingNoise/TetrahedronNoise35.pcd", *Noisycloud);


  		// pcl::io::savePCDFile ("path/to/AddingNoise/Frac2Guass14Noise.pcd", *Noisycloud);
// pcl::io::savePCDFile ("path/to/AddingNoise/Frac2withNoise710.pcd", *Noisycloud);
pcl::PLYWriter writePLY;


writePLY.write ("path/to//AddingNoise/TetrahedronNoise35.ply", *Noisycloud,  false);
// writePLY.write ("path/to//AddingNoise/Bunny03Noise50.ply", *Noisycloud,  false);
// writePLY.write ("path/to/AddingNoise/Frac2Guass14Noise.ply", *Noisycloud,  false);
// writePLY.write ("path/to/AddingNoise/Frac2withNoise710.ply", *Noisycloud,  false);



// Show the cloud
pcl::visualization::CloudViewer viewer(" ONePlaneWithNoise ");
viewer.showCloud(Noisycloud);
while (!viewer.wasStopped ())
{}
                     return (0);
                    }


#endif



