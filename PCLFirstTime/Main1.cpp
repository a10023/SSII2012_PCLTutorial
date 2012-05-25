#include "stdafx.h"
//----------------------------------------------------------------------
// SSII2012　チュートリアル「 2D&3Dレジストレーション」サンプルコード　
// その1（Sample_1_CloudViewer）
// 林昌希(mhayashi@aoki-medialab.org)
// 対応PCLバージョン: PCL1.5.1
//
// （SSII2012の資料中、デモ３）
// CloudViewerクラスにより実行引数でファイル名を指定した.pcdファイルを
// 読み込むだけの、PCLで１番シンプルなプログラムです。　
//----------------------------------------------------------------------

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <boost/thread.hpp>
using namespace boost::posix_time;

int _tmain(int argc, const char** argv)
{
	//PointCloud<PointT>オブジェクトの確保。
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	//.pcdファイルからのデータ読み込み。
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloud) == -1) 
	{
		PCL_ERROR ("指定したファイルを読み込めません \n");
		return (-1);
	}   
	//CloudViewerオブジェクトの作成 
	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	//PointCloud<PointT>のCloudViewerへの表示
	viewer.showCloud(cloud);


	while (!viewer.wasStopped ())
	{
		//boostライブラリによりこまめに短時間スリープ
		 boost::this_thread::sleep(boost::posix_time::microseconds (100000));
	}
	return 0;
}
