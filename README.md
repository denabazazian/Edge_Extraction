# Edge_Extraction
Fast and robust algorithm to extract edges in unorganized point clouds.

Source code and the dataset of this [paper](https://www.researchgate.net/profile/Dena_Bazazian/publication/304288759_Fast_and_Robust_Edge_Extraction_in_Unorganized_Point_Clouds/links/5ec5733f299bf1c09acf9283/Fast-and-Robust-Edge-Extraction-in-Unorganized-Point-Clouds.pdf ): <br />
Fast and Robust Edge Extraction in Unorganized Point Clouds (Dena Bazazian, Josep R Casas, Javier Ruiz-Hidalgo) - DICTA2015 <br />
<br />

## Code

#### Python version
* ```Difference_Eigenvalues.py``` is a source code for extracting the edges of a point cloud based on Python 3 and [pyntcloud](https://github.com/daavoo/pyntcloud) library. <br />
* Installation is based on ```conda install pyntcloud -c conda-forge``` or ```pip install pyntcloud```. <br />


#### C++ version
* ```Difference_Eigenvalues.cpp``` includes the C++ source code for extracting edges in unorganized point clouds. <br />

* ```F1Score-Eigenvalues.cpp``` is for computing the accuracy of edge extraction. <br />


## Dataset
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

