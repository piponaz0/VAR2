#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <stdio.h>
#include <math.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/correspondence.h>
#include <pcl/console/time.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/fpfh.h>
#include <pcl/registration/icp.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/susan.h>
#include <pcl/keypoints/trajkovic_3d.h>

float radioBusquedaKdTree = 1;


//El parametro radioBusqueda nos permite modificar el radio de busqueda, para buscar el valor optimo en la fase de experimentacion
void calculoNormalesKdTree(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nubePuntos, pcl::PointCloud<pcl::Normal>::Ptr& nubeNormales, pcl::PointCloud<pcl::PointNormal>::Ptr& nubeCompleta, float radioBusqueda) {
    pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> estimacion;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    
    estimacion.setInputCloud(nubePuntos);
    estimacion.setSearchMethod(kdtree);
    estimacion.setRadiusSearch(radioBusqueda);
    estimacion.compute(*nubeNormales);

    pcl::concatenateFields(*nubePuntos, *nubeNormales, *nubeCompleta);
}

void concatenarPuntoNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nubePuntos, pcl::PointCloud<pcl::PointNormal>::Ptr& nubeNormales, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& nubeCompleta) {
    pcl::concatenateFields(*nubePuntos, *nubeNormales, *nubeCompleta);
}

/* Para calcular los Keypoints, existen los siguientes algoritmos en PCL:
    - Harris 3D
    - SIFT
    - ISS
    - Susan
    - Trajkovic
*/

void calculoKeypointsHarris(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nubePuntos, pcl::PointCloud<pcl::Normal>::Ptr& nubeNormales, pcl::PointCloud<pcl::PointXYZI>::Ptr& nubeResultado) {

    float r_keypoint = 0.8;
      
    pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI,pcl::Normal>* harris = new pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI, pcl::Normal>;
    harris->setRadiusSearch(r_keypoint);
    // Establecer el radio de búsqueda del vecino más cercano para la estimación de puntos clave
    harris->setInputCloud(nubePuntos);
    harris->setNormals(nubeNormales);
    harris->setNumberOfThreads(6);
    harris->compute(*nubeResultado);

}

void calculoKeypointsSIFT(pcl::PointCloud<pcl::PointNormal>::Ptr& nubeCompleta, pcl::PointCloud<pcl::PointWithScale>::Ptr& nubeResultado) {
    
    pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
    pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree(new pcl::search::KdTree<pcl::PointNormal> ());

    const float min_scale = std::stof("0.01");                    
	const int n_octaves = std::stof("6");                         
	const int n_scales_per_octave = std::stof("4");
	const float min_contrast = std::stof("0.01"); 
    
    sift.setInputCloud(nubeCompleta);
    sift.setSearchMethod(kdtree);
    sift.setScales(min_scale,n_octaves,n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);

    sift.compute(*nubeResultado);
}


void calculoKeypointsISS(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nubePuntos, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nubeResultado) {
    
    pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>());

	iss.setInputCloud(nubePuntos);
	iss.setSearchMethod(kdtree);
	iss.setSalientRadius(0.007f);// Establecer el radio de la vecindad esférica que se usa para calcular la matriz de covarianza
	iss.setNonMaxRadius(0.005f);// Establecer el radio del algoritmo de aplicación de supresión no máxima
	iss.setThreshold21(0.65); // Establecer el límite superior de la relación entre el segundo y el primer valor propio
	iss.setThreshold32(0.5);  // Establecer el límite superior de la relación del tercer y segundo valor propio
	iss.setMinNeighbors(10); // Al aplicar el algoritmo de supresión no máxima, establezca el número mínimo de vecinos que se deben encontrar
	iss.setNumberOfThreads(6); // Inicialice el planificador y establezca el número de subprocesos que se utilizarán
	iss.compute(*nubeResultado); 
}

void calculoKeypointsSUSAN(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nubePuntos, pcl::PointCloud<pcl::Normal>::Ptr& nubeNormales, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nubeResultado) {

	pcl::SUSANKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr susan(new  pcl::SUSANKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>());

    float r_keypoint = 0.8;

    susan->setNormals(nubeNormales);
    susan->setInputCloud(nubePuntos);
    susan->setSearchMethod(kdtree);
    susan->setRadiusSearch(r_keypoint);
    susan->setNumberOfThreads(6);
    susan->compute(*nubeResultado);
}

void calculoKeypointsTrajkovic(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nubePuntos, pcl::PointCloud<pcl::Normal>::Ptr& nubeNormales, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& nubeResultado) {

    pcl::TrajkovicKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::Normal> trajkovic;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>());

    trajkovic.setInputCloud(nubePuntos);
    trajkovic.setNormals(nubeNormales);
    trajkovic.setSearchMethod(kdtree);
    trajkovic.compute(*nubeResultado);

}


void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg) {

    pcl::PointCloud<pcl::PointNormal>::Ptr nubeNormales (new pcl::PointCloud<pcl::PointNormal>);

    //Transformamos el puntero msg a un objeto PointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr msgP(new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
	
    //Calculamos las normales de la nube de puntos
    calculoNormalesKdTree(msgP, nubeNormales, radioBusquedaKdTree);
}

int main(int argc, char** argv){
}