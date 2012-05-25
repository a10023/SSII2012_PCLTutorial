#include "stdafx.h"
//----------------------------------------------------------------------
// SSII2012�@�`���[�g���A���u 2D&3D���W�X�g���[�V�����v�T���v���R�[�h�@
// ���̂R�iSample_3_ICP_Registration�j
// �я���(mhayashi@aoki-medialab.org)
// �Ή�PCL�o�[�W����: PCL1.5.1
// 
// �iSSII2012�̎������A�f���R�j
// OpenNIGrabber�N���X��p����Kinect�̃L���v�`�����J�n���A
// �ŏ��ɃL���v�`�����ꂽ�|�C���g�N���E�h(cloud_cb_�֐��̈���cloud)��
// .pcd�t�@�C���ɕۑ�����v���O�����ł��B
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
	//���W�X�g���[�V�������s���\�[�X�ƃ^�[�Q�b�g�̃|�C���g�N���E�h
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZRGBA>);
	//�\�[�X�ƃ^�[�Q�b�g�̓ǂݍ���
	reader.read (argv[1], *cloud_source);
	std::cout << "Source PointCloud "<< argv[1] << " has: " << cloud_source->points.size () << " data points." << std::endl; //*
	reader.read (argv[2], *cloud_target);
	std::cout << "Target PointCloud "<< argv[2] << " has: " << cloud_target->points.size () << " data points." << std::endl; //* 

	//ICP�p���s���N���XIterativeClosestPoint<SourcePointT,TargetPoint>���쐬
	pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
	icp.setInputCloud(cloud_source); //�\�[�X�̃|�C���g�N���E�h��ݒ�
	icp.setInputTarget(cloud_target); //�^�[�Q�b�g�̃|�C���g�N���E�h��ݒ�

	//�������ł͈�ؐݒ肵�Ă��Ȃ����AICP�͊e��p�����[�^��ݒ�ł��܂�

	//ICP�̌��ʂ�p���Ĉʒu���킳�ꂽ�\�[�X�̃|�C���g�N���E�h
	pcl::PointCloud<pcl::PointXYZRGBA> cloud_source_transformed;

	//ICP�A���S���Y�������s
	icp.align(cloud_source_transformed); 
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_final(new pcl::PointCloud<pcl::PointXYZRGBA>);
	cloud_final = (cloud_source_transformed + *cloud_target).makeShared() ; 
	//���W�X�g���[�V�������ʁicloud_final.pcd�j��.pcd�t�@�C���ɕۑ�
	writer.write("cloud_final.pcd",*cloud_final);

	//PCLVaizualizer���������āA�ړ���̃\�[�X�ƃ^�[�Q�b�g��F���ŕ\���B
	//�i�J�����̏����ʒu�������ɕ΂��Ă��ă|�C���g�N���E�h�������Ȃ��ꍇ�̓}�E�X�ŃJ�������_�𒲐��B�j
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
