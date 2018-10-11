#include <iostream>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

double computeCloudRMS(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target, pcl::PointCloud<pcl::PointXYZ>::ConstPtr source, double max_range){

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(target);
        //int count = 0;
        double fitness_score = 0.0;

        std::vector<int> nn_indices (1);
        std::vector<float> nn_dists (1);

        // For each point in the source dataset
        int nr = 0;
        for (size_t i = 0; i < source->points.size (); ++i){
                //Avoid NaN points as they crash nn searches
                if(!pcl_isfinite((*source)[i].x)){
                        continue;
                }

                // Find its nearest neighbor in the target
                tree->nearestKSearch (source->points[i], 1, nn_indices, nn_dists);
                //count++;
                // Deal with occlusions (incomplete targets)
                if (nn_dists[0] <= max_range*max_range){
                        // Add to the fitness score
                        fitness_score += nn_dists[0];
                        nr++;
                }
        }
        //std::cout<<"points number to calculate distance"<<count<<std::endl;
        //std::cout<<"number used to calculate the distance is "<< nr<<std::endl;
        if (nr > 0){
                return fitness_score;//sqrt(fitness_score / nr)*1000.0;
        }else{
                return (std::numeric_limits<double>::max ());
        }
} 

int
main (int argc, char** argv)
{
  // Loading first scan of room.
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("room_scan1.pcd", *target_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << target_cloud->size () << " data points from room_scan1.pcd" << std::endl;

  // Loading second scan of room from new perspective.
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("room_scan2.pcd", *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << input_cloud->size () << " data points from room_scan2.pcd" << std::endl;

  // Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.85, 0.85, 0.85);
  approximate_voxel_filter.setInputCloud (input_cloud);
  approximate_voxel_filter.filter (*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size ()
            << " data points from room_scan2.pcd" << std::endl;
  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (0.01);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (0.1);
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (1.795);//default 1.0

  // Setting max number of registration iterations.
  ndt.setMaximumIterations (100);

  // Setting point cloud to be aligned.
  ndt.setInputSource (filtered_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (target_cloud);

  // Set initial alignment estimate found using robot odometry.
  Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

  auto start = std::chrono::system_clock::now();
  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align (*filtered_output_cloud, init_guess);

  std::cout << ndt.getFinalTransformation() << std::endl;
  auto end = std::chrono::system_clock::now();

  std::chrono::duration<double> diff = end-start;
  std::cout << "Time calculate" 
                   << diff.count() << " s\n";
  // Transforming unfiltered, input cloud using found transform.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());//use final Transformation to get output from input
  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            <<"score: "<<computeCloudRMS(target_cloud, output_cloud, 1)<<std::endl;//ignore distance larger than 100 meters.
            //<< " score: " << ndt.getFitnessScore (100) << std::endl;//Obtain the Euclidean fitness score (e.g., sum of squared distances from the source to the target). 
  //std::cout<<"target cloud size"<<target_cloud->size()<<std::endl;
  //std::cout<<"output cloud size"<<output_cloud->size()<<std::endl;
  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr ini_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*input_cloud, *ini_cloud, init_guess);

  // Initializing point cloud visualizer
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);

  // Coloring and visualizing target cloud (red).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  target_color (target_cloud, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");

  // Coloring and visualizing inital trans cloud (blue).
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  ini_color (ini_cloud, 0, 0, 255);
  viewer_final->addPointCloud<pcl::PointXYZ> (ini_cloud, ini_color, "initial guess cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "initial guess cloud");

  // Coloring and visualizing transformed input cloud (green).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  output_color (output_cloud, 0, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");

  // Starting visualizer
  viewer_final->addCoordinateSystem (1.0, "global");
  viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  return (0);
}
