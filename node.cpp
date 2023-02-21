#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PCDReader reader;
//pcl::PCDWriter writer;

int iteraciones = 0;

void simpleVis ()
{
  	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	while(!viewer.wasStopped())
	{
	  viewer.showCloud (visu_pc);
	  boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}

}

void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
{
	// Contamos una iteración
	iteraciones++;

	// Declaramos punteros
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	// Creamos un objeto lector y lo leemos
	// pcl::PCDReader reader;
	// reader.read<pcl::PointXYZRGB>("point_cloud_1.ply", *cloud);

	cout << "Cantidad de puntos capturados: " << cloud->size() << endl;
	cout << "Nube de puntos antes de filtrado: " << *cloud << endl;

	pcl::VoxelGrid<pcl::PointXYZRGB > vGrid;
	vGrid.setInputCloud (cloud);
	vGrid.setLeafSize (0.05f, 0.05f, 0.05f);
	vGrid.filter (*cloud_filtered);

	cout << "Puntos tras filtrado VG: " << cloud_filtered->size() << endl;
	cout << "Nube de puntos tras filtrado VG: " << *cloud_filtered << endl;

	visu_pc = cloud_filtered;

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZRGB>("point_cloud_1_inliers.ply", *cloud_filtered, false);
}

void filterData()
{

}

//void build3DMap

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sub_pcl");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth/points", 1, callback);

	boost::thread t(simpleVis);

	while(ros::ok())
	{
		ros::spinOnce();
	}
	
}
