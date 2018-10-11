#include <iostream>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/approximate_voxel_grid.h>

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
                //count++;
                // Find its nearest neighbor in the target
                tree->nearestKSearch (source->points[i], 1, nn_indices, nn_dists);

                // Deal with occlusions (incomplete targets)
                if (nn_dists[0] <= max_range*max_range){
                        // Add to the fitness score
                        fitness_score += nn_dists[0];
                        nr++;
                }
        }
        //std::cout<<"points number to calculate distance"<<count<<std::endl;// all points from the targets are used to calculate the distance
        if (nr > 0){
                return fitness_score;//sqrt(fitness_score / nr)*1000.0;
        }else{
                return (std::numeric_limits<double>::max ());
        }
} 

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("room_scan1.pcd", *target_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << target_cloud->size () << " data points from room_scan1.pcd" << std::endl;

  // Loading second scan of room from new perspective.
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("room_scan2.pcd", *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << input_cloud->size () << " data points from room_scan2.pcd" << std::endl;

    // Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.85, 0.85, 0.85);//the larger of those numbers, the less of the downsample points 
  approximate_voxel_filter.setInputCloud (input_cloud);
  approximate_voxel_filter.filter (*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size ()
            << " data points from room_scan2.pcd" << std::endl;

  // Fill in the CloudIn data
  /*
  cloud_in->width    = 5;
  cloud_in->height   = 1;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {
    cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
      << std::endl;
  for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
      cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
      cloud_in->points[i].z << std::endl;
  *cloud_out = *cloud_in;
  std::cout << "size:" << cloud_out->points.size() << std::endl;
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
    cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
  std::cout << "Transformed " << cloud_in->points.size () << " data points:"
      << std::endl;
  for (size_t i = 0; i < cloud_out->points.size (); ++i)
    std::cout << "    " << cloud_out->points[i].x << " " <<
      cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
  */
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(filtered_cloud);
  icp.setInputTarget(target_cloud);
   
  // Setting scale dependent NDT parameters(criterion 1)
  // Setting minimum transformation difference for termination condition.
  icp.setTransformationEpsilon (1e-8);

  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (0.5);

  // Set the euclidean distance difference epsilon (criterion 3)
  //Set the maximum allowed Euclidean error between two consecutive steps in the ICP loop, before the algorithm is considered to have converged. 
  icp.setEuclideanFitnessEpsilon (1e-5); //criterion 1 smaller than 1e-8 and cirteria 3 smaller than 1e-5 this file can yeild good results

  // Setting max number of registration iterations.
  icp.setMaximumIterations (100);

    // Set initial alignment estimate found using robot odometry.
  Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
  cout<<"init_guess is "<<"\n"
                        <<init_guess<<endl;
  
  auto start = std::chrono::system_clock::now();

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  icp.align(*filtered_output_cloud,init_guess);

  auto end = std::chrono::system_clock::now();

  std::chrono::duration<double> diff = end-start;
  std::cout << "Time calculate" 
                   << diff.count() << " s\n";

  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*input_cloud, *output_cloud, icp.getFinalTransformation ());

  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  computeCloudRMS(target_cloud, output_cloud, 1)<<std::endl;//ignore distance larger than 100 meters.

  //std::cout<<"output cloud size"<<output_cloud->size()<<std::endl;
  //icp.getFitnessScore(100) << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

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
