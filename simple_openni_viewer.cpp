#include "SimpleOpenNIViewer.h"

SimpleOpenNIViewer::SimpleOpenNIViewer():
viewer ("PCL OpenNI Viewer")
{
}

void SimpleOpenNIViewer::cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
        if (!viewer.wasStopped())
        {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
               
                //FILTERING PROCESS TO ELIMINATE EVERYTHING BUT THE SURFACE
                pcl::PassThrough<pcl::PointXYZ> pass;
                pass.setInputCloud (cloud);
                pass.setFilterFieldName ("z");
                pass.setFilterLimits (0, 1.3);
                pass.filter (*cloud_filtered);

                pass.setInputCloud (cloud_filtered);
                pass.setFilterFieldName ("x");
                pass.setFilterLimits (-0.4, 0.4);
                //pass.setFilterLimitsNegative (true);
                pass.filter (*cloud_filtered);

                pass.setInputCloud (cloud_filtered);
                pass.setFilterFieldName ("y");
                pass.setFilterLimits (-0.15, 0.3);
                pass.filter (*cloud_filtered);

                //DOWNSAMPLING RESULTING POINT CLOUD
                pcl::VoxelGrid<pcl::PointXYZ> sor;
                sor.setInputCloud (cloud_filtered);
                sor.setLeafSize (0.01f, 0.01f, 0.01f);
                sor.filter (*cloud_filtered2);
               
                //SEGMENT SURFACE
                pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
                pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
                // Create the segmentation object
                pcl::SACSegmentation<pcl::PointXYZ> seg;
                // Optional
                seg.setOptimizeCoefficients (true);

                // Mandatory
                seg.setModelType (pcl::SACMODEL_PLANE);
                seg.setMethodType (pcl::SAC_RANSAC);
                seg.setDistanceThreshold (0.01);
                seg.setInputCloud (cloud_filtered2->makeShared());
                seg.segment (*inliers, *coefficients);

                if (inliers->indices.size () == 0)
                {
                        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
                  exit(0);
                }
               
                //PAINT SURFACE
             /* for (unsigned int i = 0; i < inliers->indices.size(); i++)
                {
                        int idx = inliers->indices[i];
                        cloud_filtered2->points[idx].r = 255;
                        cloud_filtered2->points[idx].g = 0;
                        cloud_filtered2->points[idx].b = 0;
                }  */

                viewer.showCloud(cloud_filtered2);

                  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                              << coefficients->values[1] << " "
                              << coefficients->values[2] << " "
                              << coefficients->values[3] << std::endl;


                 std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
                 for (size_t i = 0; i < inliers->indices.size (); ++i)
                    std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                                                << cloud->points[inliers->indices[i]].y << " "
                                                                << cloud->points[inliers->indices[i]].z << std::endl;

        }
}

void SimpleOpenNIViewer::run()
{
        pcl::Grabber* interface = new pcl::OpenNIGrabber();

        boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)>    f = 
        boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

        interface->registerCallback (f);

        interface->start ();

        while (!viewer.wasStopped())
        {
            boost::this_thread::sleep (boost::posix_time::seconds (1));
        }

        interface->stop ();
}

int 
main ()
 {
   SimpleOpenNIViewer v;
   v.run ();
   return 0;
 } 