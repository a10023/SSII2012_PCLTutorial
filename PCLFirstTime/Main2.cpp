#include "stdafx.h"
//----------------------------------------------------------------------
// SSII2012�@�`���[�g���A���u 2D&3D���W�X�g���[�V�����v�T���v���R�[�h�@
// ���̂Q�iSample_2_OpenNICapture�j
// �я���(mhayashi@aoki-medialab.org)
// �Ή�PCL�o�[�W����: PCL1.5.1
// 
// �iSSII2012�̎������A�f���P�j
// OpenNIGrabber�N���X��p����Kinect�̃L���v�`�����J�n���A
// �ŏ��ɃL���v�`�����ꂽ�|�C���g�N���E�h(cloud_cb_�֐��̈���cloud)��
// .pcd�t�@�C���ɕۑ�����v���O�����ł��B
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
///----------------SimpleOpenNIProcessor�N���X/----------------
// OpenNIGrabber�ŃL���v�`�������|�C���g�N���E�h��
// .pcd�t�@�C���i"output.pcd"�j�ɕۑ�����N���X�B
//-------------------------------------------------------------
class SimpleOpenNIProcessor
{
private:	
	//�ۑ��̂��߂̃L���v�`�����I��������𔻒肷��t���O�ϐ�
	bool captured;
	//Kinect�ŃL���v�`�������f�[�^��\������CloudViewer
	pcl::visualization::CloudViewer viewer;
	//�w�肵���̈�ix��-0.5 - 0.5,z��0.5 - 2.5�j�̃|�C���g�N���E�h
	//�������c�����߂�PassThrough�t�B���^
	pcl::PassThrough<pcl::PointXYZRGBA> passX;
	pcl::PassThrough<pcl::PointXYZRGBA> passZ;
public:

	SimpleOpenNIProcessor(): viewer("PCL OpenNI Viewer")
	{
		captured = false;

	}

	//Kinect����RGB�摜�ƃf�v�X�摜��PoinCloud<PointT>�^�I�u�W�F�N�g�i�����ł�cloud�j�Ƃ��Ď擾����邲�Ƃ�
	//�Ăяo�����R�[���o�b�N�֐�
	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{
		//PathThrough�t�B���^�������ꂽ���Ƃ̃|�C���g�N���E�h
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
		//x��������PassThrough�t�B���^�[��ݒ�
		passX.setInputCloud (cloud);
		passX.setFilterFieldName ("x");
		passX.setFilterLimits (-0.5, 0.5);
		//�t�B���^���������s
		passX.filter (*cloud_filtered);

		//z��������PathThrough�t�B���^�[��ݒ�
		passZ.setInputCloud (cloud_filtered);
		passZ.setFilterFieldName ("z");
		passZ.setFilterLimits (0.5, 2.5);
		//�t�B���^���������s
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
		//OpenNI�f�o�C�X�p��OpenNIGrabber�I�u�W�F�N�g���쐬 
		pcl::Grabber* interface = new pcl::OpenNIGrabber();

		//�R�[���o�b�N�֐���cloud_cb_���o�C���h 
		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
			boost::bind (&SimpleOpenNIProcessor::cloud_cb_, this, _1);

		//�R�[���o�b�N�֐���OpenNIGrabber�ɐڑ�
		boost::signals2::connection c = interface->registerCallback (f);

		//�|�C���g�N���E�h�̎擾���J�n
		interface->start ();

		//Ctrl-C�Ȃǂ��������܂�While���[�v
		while (true)
			sleep(1);

		//OpenNIGrabber���~
		interface->stop ();
	}
};
//main�֐�
int main ()
{
	SimpleOpenNIProcessor v;
	v.run ();
	return (0);
}
