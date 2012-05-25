#include "stdafx.h"
//----------------------------------------------------------------------
// SSII2012　チュートリアル「 2D&3Dレジストレーション」サンプルコード　
// その３（Sample_3_ICP_Registration）
// 林昌希(mhayashi@aoki-medialab.org)
// 対応PCLバージョン: PCL1.5.1
// 
// （SSII2012の資料中、デモ３）
// OpenNIGrabberクラスを用いてKinectのキャプチャを開始し、
// 最初にキャプチャされたポイントクラウド(cloud_cb_関数の引数cloud)を
// .pcdファイルに保存するプログラムです。
//----------------------------------------------------------------------

#include <iostream>

#include <boost/thread/thread.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include "pcl/visualization/pcl_visualizer.h"

int
 main (int argc, char** argv)
{
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	//レジストレーションを行うソースとターゲットのポイントクラウド
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZRGBA>);
	//ソースとターゲットの読み込み
	reader.read (argv[1], *cloud_source);
	std::cout << "Source PointCloud "<< argv[1] << " has: " << cloud_source->points.size () << " data points." << std::endl; //*
	reader.read (argv[2], *cloud_target);
	std::cout << "Target PointCloud "<< argv[2] << " has: " << cloud_target->points.size () << " data points." << std::endl; //* 

	//ICP用を行うクラスIterativeClosestPoint<SourcePointT,TargetPoint>を作成
	pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
	icp.setInputCloud(cloud_source); //ソースのポイントクラウドを設定
	icp.setInputTarget(cloud_target); //ターゲットのポイントクラウドを設定

	//※ここでは一切設定していないが、ICPは各種パラメータを設定できます

	//ICPの結果を用いて位置合わされたソースのポイントクラウド
	pcl::PointCloud<pcl::PointXYZRGBA> cloud_source_transformed;

	//ICPアルゴリズムを実行
	icp.align(cloud_source_transformed); 
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_final(new pcl::PointCloud<pcl::PointXYZRGBA>);
	cloud_final = (cloud_source_transformed + *cloud_target).makeShared() ; 
	//レジストレーション結果（cloud_final.pcd）を.pcdファイルに保存
	writer.write("cloud_final.pcd",*cloud_final);

	//PCLVaizualizerを準備して、移動後のソースとターゲットを色つきで表示。
	//（カメラの初期位置が中央に偏っていてポイントクラウドが見えない場合はマウスでカメラ視点を調整。）
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Registered Result"));
	viewer->setBackgroundColor (255, 255, 255);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> source_color(cloud_source_transformed.makeShared(), 0, 0, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> target_color(cloud_target, 255, 0, 0);

	viewer->addPointCloud<pcl::PointXYZRGBA> (cloud_source_transformed.makeShared(), source_color, "source cloud transformed");
	viewer->addPointCloud<pcl::PointXYZRGBA> (cloud_target, target_color, "target cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();

	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	return (0);
}
