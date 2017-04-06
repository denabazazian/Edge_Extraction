/*
 * EigenvalueCovariance.cpp
 *
 *  Created on: May 6, 2015
 *      Author: dbazazian
 */

     #include <iostream>
     #include <Eigen/Eigenvalues>
     #include <Eigen/Dense>
     #include <vector>
     #include <math.h>
     #include <cmath>
     #include <fstream>
     #include <string>
     #include <vector>
     #include <pcl/io/io.h>
     #include <pcl/io/pcd_io.h>
     #include <pcl/point_types.h>
     #include <pcl/features/integral_image_normal.h>
     #include <pcl/features/normal_3d.h>
     #include <pcl/common/common_headers.h>
     #include <pcl/features/integral_image_normal.h>
     #include <pcl/features/normal_3d.h>
     #include <pcl/visualization/cloud_viewer.h>
     #include <pcl/filters/passthrough.h>
     #include <pcl/ModelCoefficients.h>
     #include <pcl/filters/project_inliers.h>
     #include <pcl/features/shot_omp.h>
     #include "pcl/features/fpfh.h"
     #include <pcl/io/ply_io.h>

using namespace std;
using namespace Eigen;


int
main (int argc, char*argv[])
{
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr RelevantElements(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr SelectedElements(new pcl::PointCloud<pcl::PointXYZRGBA>);


	 // pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/TwoPlane45.pcd", *cloud);
	 // pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/CubeSharpEdge.pcd", *cloud);
	  // pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/OnePlane.pcd", *cloud);
	      // pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/bunny.pcd",*cloud);
	     //  pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/Statue.pcd", *cloud);
	  //  pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/dragon.pcd", *cloud);


	  //   pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/AimAtShape/trim-starC.pcd", *cloud);
	      //	pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/AimAtShape/VaseC.pcd", *cloud);
	       //  pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/AimAtShape/twirlC.pcd", *cloud);
	      	//  pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/AimAtShape/fandiskC.pcd", *cloud);
	      // pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/AimAtShape/sharp_sphereC.pcd", *cloud);
		// pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/CubeFractal2.pcd", *cloud);
		//  pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/AddingNoise/Frac2Guass12Noise.pcd", *cloud);
		  // pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/IntersectionThreePlanes.pcd", *cloud);
		// pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/SpherMultiple.pcd", *cloud);
		 // pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/Tetrahedron.pcd", *cloud);
		 //pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/AddingNoise/TetrahedronNoise35.pcd", *cloud);
		   pcl::io::loadPCDFile ("/Path/TO/ArtificialPointClouds/TetrahedronMultiple.pcd", *cloud);



	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge (new pcl::PointCloud<pcl::PointXYZRGBA>);
	 // pcl::io::loadPCDFile ("/Path/TO/GroundTruth/GroundTruthTwoPlaneX.pcd", *edge);
	  	  // pcl::io::loadPCDFile ("/Path/TO/GroundTruth/GroundTruthTwoPlaneY.pcd", *edge);
	  	// pcl::io::loadPCDFile ("/Path/TO/GroundTruth/ExactEdge.pcd", *edge);
	  	// pcl::io::loadPCDFile (	"/Path/TO/GroundTruth/GroundtruthFractal2.pcd", *edge);
	 // pcl::io::loadPCDFile (	"/Path/TO/GroundTruth/GroundTruthThreeoPlanes.pcd", *edge);
	 // pcl::io::loadPCDFile (	"/Path/TO/GroundTruth/GroundTruthTetrahedron.pcd", *edge);
	  pcl::io::loadPCDFile ("/Path/TO/GroundTruth/GroundTruthTetrahedronMultiple.pcd", *edge);


	   std::cout << "Number of points in the Cube Input cloud is:"<< cloud->points.size() << std::endl;
	   std::cout << "Number of points in the Edge Ground Truth cloud is:"<< edge->points.size() << std::endl;


	   // Compute distance between all the points of the flat cloud and points of the Edge cloud
	   	std::vector<std::vector <double> > Distances ; // 2D vector(Matrix) for each point that has the Angle between lines of normals around barycenter
	   		 int heightD;
	   		 int widthD;
	    // defining an empty 2D vector (matrix)
	   	 heightD= cloud ->points.size ();
	   		 widthD = edge ->points.size ();
	   			 Distances.resize(heightD); // to defining numbers of row in 2D vector
	   				   for(int jj=0; jj< heightD; ++jj)
	   				   	{Distances[jj].resize(widthD);}   // to defining numbers of column for each row in 2d vector

	   for (size_t ii = 0; ii < cloud ->points.size (); ++ii) {
	    for (size_t jj = 0; jj< edge ->points.size (); ++jj) {
	   double distance = sqrt (   	( ( (cloud->points[ii].x)	-  (edge->points[jj].x) ) *  ( (cloud->points[ii].x)	-  (edge->points[jj].x) ) ) +   ( ( (cloud->points[ii].y)	-  (edge->points[jj].y) ) *  ( (cloud->points[ii].y)	-  (edge->points[jj].y) ) ) + ( ( (cloud->points[ii].z)	-  (edge->points[jj].z) ) *  ( (cloud->points[ii].z)	-  (edge->points[jj].z) ) )   );
	   		Distances [ii][jj] = distance;
	   			 }
	   		 }
	   	 // Define a cloud for the relevant elements
	   	 double relevantsize = 0.00 ;
	   		 // find minimum distance for each line
	   		std::vector<double> MinDistances;
	   		 for (size_t ii = 0; ii < cloud ->points.size (); ++ii) {
	   		 double min = 100.00;
	   		for (size_t jj = 0; jj< edge ->points.size (); ++jj) {
	   		 if (Distances [ii][jj] < min ){
	   			 min = Distances [ii][jj] ;}
	   				   }
	   	MinDistances .push_back (min );
	   	 if (min< 0.0075){
	   // Copy the 	coordinates on to  the relevant cloud
	   				pcl::PointXYZRGBA basic_point;
	   				basic_point.x = cloud->points[ii].x;
	   				basic_point.y = cloud->points[ii].y;
	   				basic_point.z = cloud->points[ii].z;
	   				basic_point.r = 255;
	   				basic_point.g = 255;
	   				basic_point.b= 255;
	   				RelevantElements->points.push_back(basic_point);
	   				relevantsize += 1.00 ;
	   				         }// change the color in main cloud
	   		 	 }// if min <0.03
	   // Size of the RelevantElements Cloud
	   		 RelevantElements->width = (int) RelevantElements->points.size ();
	   		 RelevantElements->height = 1;
	   		 std::cout << "Number of points in the relevant location is:"<< relevantsize << std::endl;
	   		 std::cout << "size of the relevant cloud is:"<< RelevantElements->points.size () << std::endl;


// to visualize the relevant points
	   	//  pcl::visualization::CloudViewer viewer1("Relevant points");
	   	//  viewer1.showCloud(RelevantElements);
	   	 //   while (!viewer1.wasStopped ())
	   	  // {}



	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Normals (new pcl::PointCloud<pcl::PointXYZRGBA>);
	  Normals->resize(cloud->size());

	  // K nearest neighbor search
	  int KNumbersNeighbor = 10; // numbers of neighbors 7
	  std::vector<int> NeighborsKNSearch(KNumbersNeighbor);
	  std::vector<float> NeighborsKNSquaredDistance(KNumbersNeighbor);

	  int* NumbersNeighbor = new  int [cloud ->points.size ()];
	  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
	  kdtree.setInputCloud (cloud);
	  pcl::PointXYZRGBA searchPoint;



	  double* SmallestEigen = new  double [cloud->points.size() ];
	  double* MiddleEigen = new  double [cloud->points.size() ];
	  double* LargestEigen = new  double [cloud->points.size() ];

	  double* DLS = new  double [cloud->points.size() ];
	  double* DLM = new  double [cloud->points.size() ];
	  double* DMS = new  double [cloud->points.size() ];
	  double* Sigma = new  double [cloud->points.size() ];

//		std::vector<double> SmallestEigen;
//		std::vector<double> MiddleEigen;
//		std::vector<double> LargestEigen;
//
//		std::vector<double> DLS;
//		std::vector<double> DML;
//		std::vector<double> DMS;

	    //  ************ All the Points of the cloud *******************
	for (size_t i = 0; i < cloud ->points.size (); ++i) {

	searchPoint.x =   cloud->points[i].x;
	searchPoint.y =   cloud->points[i].y;
	searchPoint.z =   cloud->points[i].z;

	if ( kdtree.nearestKSearch (searchPoint, KNumbersNeighbor, NeighborsKNSearch, NeighborsKNSquaredDistance) > 0 ) {
		 NumbersNeighbor[i]= NeighborsKNSearch.size (); }
	    else { NumbersNeighbor[i] = 0; }

	float Xmean; float Ymean; float Zmean;
	float sum= 0.00;
// Computing Covariance Matrix
	for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii){
      sum += cloud->points[ NeighborsKNSearch[ii] ].x; }
	Xmean = sum / NumbersNeighbor[i] ;
	sum= 0.00;
			for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii){
			sum += cloud->points[NeighborsKNSearch[ii] ].y;}
			Ymean = sum / NumbersNeighbor[i] ;
		 sum= 0.00;
			for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii){
			sum += cloud->points[NeighborsKNSearch[ii] ].z;}
			Zmean = sum / NumbersNeighbor[i] ;

			float	CovXX;  float CovXY; float CovXZ; float CovYX; float CovYY; float CovYZ; float CovZX; float CovZY; float CovZZ;

			sum = 0.00 ;
			for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii){
			sum += ( (cloud->points[NeighborsKNSearch[ii] ].x - Xmean ) * ( cloud->points[NeighborsKNSearch[ii] ].x - Xmean )  );}
			CovXX = sum / ( NumbersNeighbor[i]-1) ;

			sum = 0.00 ;
			for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii){
			sum += ( (cloud->points[NeighborsKNSearch[ii] ].x - Xmean ) * ( cloud->points[NeighborsKNSearch[ii] ].y - Ymean )  );}
			CovXY = sum / ( NumbersNeighbor[i]-1) ;

			CovYX = CovXY ;

			sum = 0.00 ;
			for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii){
			sum += ( (cloud->points[NeighborsKNSearch[ii] ].x - Xmean ) * ( cloud->points[NeighborsKNSearch[ii] ].z - Zmean )  );}
			CovXZ= sum / ( NumbersNeighbor[i]-1) ;

			CovZX = CovXZ;

			sum = 0.00 ;
			for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii){
			sum += ( (cloud->points[NeighborsKNSearch[ii] ].y - Ymean ) * ( cloud->points[NeighborsKNSearch[ii] ].y - Ymean )  );}
			CovYY = sum / ( NumbersNeighbor[i]-1) ;

			sum = 0.00 ;
			for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii){
			sum += ( (cloud->points[NeighborsKNSearch[ii] ].y - Ymean ) * ( cloud->points[NeighborsKNSearch[ii] ].z - Zmean )  );}
			CovYZ = sum / ( NumbersNeighbor[i]-1) ;

			CovZY = CovYZ;

			sum = 0.00 ;
			for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii){
			sum += ( (cloud->points[NeighborsKNSearch[ii] ].z - Zmean ) * ( cloud->points[NeighborsKNSearch[ii] ].z - Zmean )  );}
			CovZZ = sum / ( NumbersNeighbor[i]-1) ;

// Computing Eigenvalue and EigenVector
   Matrix3f Cov;
   Cov << CovXX, CovXY, CovXZ, CovYX, CovYY, CovYZ, CovZX, CovZY, CovZZ;

  SelfAdjointEigenSolver<Matrix3f> eigensolver(Cov);
  if (eigensolver.info() != Success) abort();

  double EigenValue1 = eigensolver.eigenvalues()[0];
  double EigenValue2 = eigensolver.eigenvalues()[1];
  double EigenValue3 = eigensolver.eigenvalues()[2];

  double Smallest = 0.00; double Middle = 0.00; double Largest= 0.00;
  if (EigenValue1<  EigenValue2 ) { Smallest =  EigenValue1 ; } else { Smallest = EigenValue2 ; }
  if (EigenValue3<  Smallest ) { Smallest =  EigenValue3 ; }


  if(EigenValue1 <= EigenValue2 && EigenValue1 <= EigenValue3) {
	  Smallest = EigenValue1;
  if(EigenValue2 <= EigenValue3) {Middle = EigenValue2; Largest = EigenValue3;}
  else {Middle = EigenValue3; Largest = EigenValue2;}
  }

  if(EigenValue1 >= EigenValue2 && EigenValue1 >= EigenValue3)
  {
	  Largest = EigenValue1;
  if(EigenValue2 <= EigenValue3) { Smallest = EigenValue2; Middle = EigenValue3; }
  else {Smallest = EigenValue3; Middle = EigenValue2;}
  }

  if ((EigenValue1 >= EigenValue2 && EigenValue1 <= EigenValue3) || (EigenValue1 <= EigenValue2 && EigenValue1 >= EigenValue3))
  {
	  Middle = EigenValue1;
  if(EigenValue2 >= EigenValue3){Largest = EigenValue2; Smallest = EigenValue3;}
  else{Largest = EigenValue3; Smallest = EigenValue2;}
  }

SmallestEigen[i]= Smallest ;
MiddleEigen[i]= Middle;
LargestEigen[i]= Largest;

DLS[i] = std::abs ( LargestEigen[i] -  SmallestEigen[i] ) ;
DLM[i] = std::abs (  LargestEigen[i] - MiddleEigen[i] ) ;
DMS[i] = std::abs ( MiddleEigen[i] -  SmallestEigen[i] ) ;
Sigma[i] = (SmallestEigen[i] ) / ( SmallestEigen[i] + MiddleEigen[i] + LargestEigen[i] ) ;
	} // For each point of the cloud

// Color Map For the difference of the eigen values

	  double MaxD=0.00 ;
	  double MinD= cloud ->points.size ();
	  int Ncolors=256;

	  for (size_t i = 0; i < cloud ->points.size (); ++i) {
	  	if (  Sigma [i] < MinD) MinD= Sigma [i];
	  	if (  Sigma[i] > MaxD) MaxD = Sigma [i];
	  }

	  std::cout<< " Minimum is :" << MinD<< std::endl;
	  std::cout<< " Maximum  is :" << MaxD << std::endl;

	 double ss = 0.00 ;
	  for (size_t i = 0; i < cloud ->points.size (); ++i) {
		  ss += Sigma [i] ;}
	  double avg = ss / cloud ->points.size () ;
	  ss = 0.00 ;
	  for (size_t i = 0; i < cloud ->points.size (); ++i) {
		  ss += (Sigma [i] -  avg ) * (  Sigma [i] -  avg ) ;}
	  double stddvtion =   sqrt (  ss  /  ( cloud ->points.size () - 1 )  ) ;

	  std::cout<< " Standard Deviation is :" << stddvtion << std::endl;

	  MaxD = ( 2 )* stddvtion;

/*
	  // Color table
		double line;
		double code[Ncolors][3];
	   ifstream colorcode ( "/Path/TO/ArtificialPointCloud/JetColorDensity/ColorCodes256.txt" );
	   //store color codes in array
	    int i=0,j=0;
	    while( colorcode>> line ) {
	    code[i][j]=line;
	    j++;
	    if (j == 3)
	    i++;
	}
	    code[1][0] = 0;
	    code[1][1] = 0;
	    code[1][2] = 135.468;

	int level = 0;
	float step = ( ( MaxD -  MinD) / Ncolors ) ;
	    for (size_t i = 0; i < cloud ->points.size (); ++i) {
if (  Sigma [i] < MaxD ) {
	    level = floor( (Sigma [i] - MinD ) /  step ) ;

	    cloud->points[i].r = code[ level ][0];
	    cloud->points[i].g =  code[ level ][1];
	    cloud->points[i].b =  code[ level ][2];
} // if sigma less than MAx
	    }
*/

	  int level = 0;
	  float step = ( ( MaxD -  MinD) / Ncolors ) ;
	 //  level = floor( (Sigma [i] - MinD ) /  step ) ;
for (size_t i = 0; i < cloud ->points.size (); ++i) {
	  if ( Sigma [i] > ( MinD + ( 0.003* step) ) ) {   // for the 3 plane ( Sigma [i] > ( MinD + ( 0.003* step) ) )
	    cloud->points[i].r = 0;
	    cloud->points[i].g =  0 ;
	    cloud->points[i].b =  128;
		pcl::PointXYZRGBA basic_point2;
		basic_point2.x = cloud->points[i].x;
		basic_point2.y = cloud->points[i].y;
		basic_point2.z = cloud->points[i].z;
		basic_point2.r = 255;
		basic_point2.g = 255;
		basic_point2.b= 255;
		SelectedElements->points.push_back(basic_point2);
        }
     }




// Computing

SelectedElements->width = (int) SelectedElements->points.size ();
SelectedElements->height = 1;
std::cout <<"Size of the Edge  cloud by clustering method is:  " << SelectedElements->points.size ()<< std::endl;


// Compute the  True Positive by Comparing the two Clouds of Relevant and Selective
double TruePositive = 0.00 ;

	 for (size_t ii = 0; ii< RelevantElements ->points.size (); ++ii) {
 	 for (size_t jj = 0; jj< SelectedElements ->points.size (); ++jj) {
 		 if ( ((RelevantElements->points[ii].x) == (SelectedElements->points[jj].x)) &&   ((RelevantElements->points[ii].y) == (SelectedElements->points[jj].y)) &&  ((RelevantElements->points[ii].z) == (SelectedElements->points[jj].z)) ) {
			TruePositive += 1.00 ; 	 } // if
		     } // jj
 	 } // ii

std::cout <<"Numbers of True Positive  is:  " << TruePositive << std::endl;
double FalseNegative = (RelevantElements ->points.size () ) - TruePositive ;
std::cout <<"Numbers of False Negative  is:  " << FalseNegative << std::endl;
double FalsePositive = (SelectedElements ->points.size ()) - TruePositive;
std::cout <<"Numbers of False Positive  is:  " << FalsePositive << std::endl;
double TrueNegative = ((cloud ->points.size ()) -  (RelevantElements ->points.size () ) ) -  FalsePositive ;
std::cout <<"Numbers of True Negative  is:  " << TrueNegative << std::endl;

double Precision = (TruePositive / ( SelectedElements ->points.size () ) ) ;
std::cout <<"Numbers of Precision  is:  " << Precision << std::endl;
double Recall = (TruePositive / (RelevantElements ->points.size () ) ) ;
std::cout <<"Numbers of Recall   is:  " << Recall << std::endl;
double FScore = 2* (   ( Precision * Recall  ) / (Precision +  Recall )) ;
std::cout <<"F1 Score of True Positive  is:  " << FScore << std::endl;



  	pcl::PLYWriter writePLY;
  //	writePLY.write ("/Path/TO/CUbeEigenJetColor.ply", *cloud,  false);
	 //   writePLY.write ("/Path/TO/CloudEigeJnetTwirl.ply", *cloud,  false);
	// writePLY.write ("/Path/TO/CloudEigeJnetDragon.ply", *cloud,  false);
  	//  writePLY.write ("/Path/TO/CloudEigeJnetTwoPlane22.ply", *cloud,  false);

  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  viewer.showCloud(cloud);
  while (!viewer.wasStopped ())
  {}

  return 0;
  }