#include "stdafx.h"
//----------------------------------------------------------------------
// SSII2012�@�`���[�g���A���u 2D&3D���W�X�g���[�V�����v�T���v���R�[�h�@
// ����1�iSample_1_CloudViewer�j
// �я���(mhayashi@aoki-medialab.org)
// �Ή�PCL�o�[�W����: PCL1.5.1
//
// �iSSII2012�̎������A�f���R�j
// CloudViewer�N���X�ɂ����s�����Ńt�@�C�������w�肵��.pcd�t�@�C����
// �ǂݍ��ނ����́APCL�łP�ԃV���v���ȃv���O�����ł��B�@
//----------------------------------------------------------------------

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <boost/thread.hpp>
using namespace boost::posix_time;

int _tmain(int argc, const char** argv)
{
	//PointCloud<PointT>�I�u�W�F�N�g�̊m�ہB
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	//.pcd�t�@�C������̃f�[�^�ǂݍ��݁B
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloud) == -1) 
	{
		PCL_ERROR ("�w�肵���t�@�C����ǂݍ��߂܂��� \n");
		return (-1);
	}   
	//CloudViewer�I�u�W�F�N�g�̍쐬 
	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	//PointCloud<PointT>��CloudViewer�ւ̕\��
	viewer.showCloud(cloud);


	while (!viewer.wasStopped ())
	{
		//boost���C�u�����ɂ�肱�܂߂ɒZ���ԃX���[�v
		 boost::this_thread::sleep(boost::posix_time::microseconds (100000));
	}
	return 0;
}
