# Edge_Extraction
Fast and robust algorithm to extract edges in unorganized point clouds.

Source code and the dataset for this [paper](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.701.4058&rep=rep1&type=pdf ): <br />
Fast and Robust Edge Extraction in Unorganized Point Clouds (Dena Bazazian, Josep R Casas, Javier Ruiz-Hidalgo) - DICTA2015 <br />
<br />

## Code

* ```Difference_Eigenvalues.cpp``` includes the C++ source code for extracting edges in unorganized point clouds. <br />

* ```F1Score-Eigenvalues.cpp``` is for computing the accuracy of edge extraction. <br />

* We have created some artificial point clouds in order to have a labeled dataset, since we have both the point clouds and ground truths. Hence, in the ``` ArtificialPointClouds``` and ```GroundTruth ``` directories, you can find the artificial point clouds and their correspond ground truth. <br />

* In addition, in the ``` artificial_point_cloud.cpp ``` you can access to the source code that we have generated those artificial point clouds. <br />



# Citation
Please cite this work in your publications if it helps your research: <br />

@InProceedings{Bazazian15, <br />
  author = {Bazazian, Dena and Casas, Josep R and Ruiz-Hidalgo, Javier}, <br />
  title = {Fast and Robust Edge Extraction in Unorganized Point Clouds}, <br />
  booktitle = {Proceeding of International Confere on Digital Image Computing: Techniques and Applications (DICTA)}, <br />
  publisher = {IEEE}, <br />
  pages = {1-8}, <br />
  year = {2015} <br />
}

