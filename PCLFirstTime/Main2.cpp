#include "stdafx.h"
//----------------------------------------------------------------------
// SSII2012　チュートリアル「 2D&3Dレジストレーション」サンプルコード　
// その２（Sample_2_OpenNICapture）
// 林昌希(mhayashi@aoki-medialab.org)
// 対応PCLバージョン: PCL1.5.1
// 
// （SSII2012の資料中、デモ１）
// OpenNIGrabberクラスを用いてKinectのキャプチャを開始し、
// 最初にキャプチャされたポイントクラウド(cloud_cb_関数の引数cloud)を
// .pcdファイルに保存するプログラムです。
//----------------------------------------------------------------------

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

#ifdef WIN32
# define sleep(x) Sleep((x)*1000) 
#endif
///----------------SimpleOpenNIProcessorクラス/----------------
// OpenNIGrabberでキャプチャしたポイントクラウドを
// .pcdファイル（"output.pcd"）に保存するクラス。
//-------------------------------------------------------------
class SimpleOpenNIProcessor
{
private:	
	//保存のためのキャプチャが終わったかを判定するフラグ変数
	bool captured;
	//Kinectでキャプチャしたデータを表示するCloudViewer
	pcl::visualization::CloudViewer viewer;
	//指定した領域（xが-0.5 - 0.5,zが0.5 - 2.5）のポイントクラウド
	//だけを残すためのPassThroughフィルタ
	pcl::PassThrough<pcl::PointXYZRGBA> passX;
	pcl::PassThrough<pcl::PointXYZRGBA> passZ;
public:

	SimpleOpenNIProcessor(): viewer("PCL OpenNI Viewer")
	{
		captured = false;

	}

	//KinectからRGB画像とデプス画像がPoinCloud<PointT>型オブジェクト（ここではcloud）として取得されるごとに
	//呼び出されるコールバック関数
	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{
		//PathThroughフィルタ処理されたあとのポイントクラウド
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
		//x軸方向のPassThroughフィルターを設定
		passX.setInputCloud (cloud);
		passX.setFilterFieldName ("x");
		passX.setFilterLimits (-0.5, 0.5);
		//フィルタ処理を実行
		passX.filter (*cloud_filtered);

		//z軸方向のPathThroughフィルターを設定
		passZ.setInputCloud (cloud_filtered);
		passZ.setFilterFieldName ("z");
		passZ.setFilterLimits (0.5, 2.5);
		//フィルタ処理を実行
		passZ.filter (*cloud_filtered);

		if (!captured)
		{
			pcl::io::savePCDFile("output.pcd", *cloud_filtered);
			viewer.showCloud (cloud_filtered);
			captured = true;
		}
	}

	void run ()
	{
		//OpenNIデバイス用のOpenNIGrabberオブジェクトを作成 
		pcl::Grabber* interface = new pcl::OpenNIGrabber();

		//コールバック関数にcloud_cb_をバインド 
		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
			boost::bind (&SimpleOpenNIProcessor::cloud_cb_, this, _1);

		//コールバック関数をOpenNIGrabberに接続
		boost::signals2::connection c = interface->registerCallback (f);

		//ポイントクラウドの取得を開始
		interface->start ();

		//Ctrl-Cなどが押されるまでWhileループ
		while (true)
			sleep(1);

		//OpenNIGrabberを停止
		interface->stop ();
	}
};
//main関数
int main ()
{
	SimpleOpenNIProcessor v;
	v.run ();
	return (0);
}
