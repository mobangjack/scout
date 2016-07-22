#include "cv.h"
#include "highgui.h"
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
using namespace std;
using namespace cv;
#include "tracking.hpp"
#include "iostream"

#define ZOOMIN_FACTOR 0.6

class FD{
	public:
	FD();
	~FD() {};
	CvRect update_mhi(IplImage* img,int diff_threshold);
	cv::Rect detect(Mat& img);
	void init(cv::Mat&);
	
	protected:
	const double MHI_DURATION = 0.5;//0.5sΪ�˶����ٵ�������ʱ��
	const double MAX_TIME_DELTA = 0.5; //���ʱ������Ϊ0.5s
	const double MIN_TIME_DELTA = 0.05; //��Сʱ������0.05s
	const int N = 3;
	int last = 0;
	IplImage **buf = 0;//ָ���ָ��
	IplImage *mhi = 0; // MHI: motion history image �˶���ʷͼ��
	IplImage *pimg;
	CvRect roi;
	IplImage* dst;
	IplImage temp; 
};

FD::FD()
{	
	
	
}
	
cv::Rect FD::detect(Mat& img)
{
	temp=IplImage(img);
	pimg=&temp;
		
	roi=update_mhi(pimg,  60);
	
	cv::Rect rect(roi);
	return rect;
}


void FD::init(cv::Mat& img)
{
	
	temp=IplImage(img);
	
	pimg=&temp;
	
	dst = cvCreateImage(cvSize(pimg->width, pimg->height), 8, 1); 
	cout<<pimg->width<<"   "<<pimg->height<<endl;
	cvZero(dst); 

}
	

// ������
// img - ������Ƶ֡ // dst - �����
CvRect FD::update_mhi(IplImage* img, int diff_threshold)
{
	
	int CONTOUR_MAX_AERA=16;
	double timestamp = clock() / 100.; // get current time in seconds ʱ���
	CvSize size = cvSize(img->width, img->height); // get current frame size���õ���ǰ֡�ĳߴ�
	int i, idx1, idx2;
	IplImage* silh;
	IplImage* pyr = cvCreateImage(cvSize((size.width & -2) / 2, (size.height & -2) / 2), 8, 1);
	
	CvMemStorage *stor;
	CvSeq *cont;

	/*�Ƚ������ݵĳ�ʼ��*/
	if (!mhi || mhi->width != size.width || mhi->height != size.height)
	{
		if (buf == 0) //����û�г�ʼ��������ڴ����
		{
			buf = (IplImage**)malloc(N*sizeof(buf[0]));
			memset(buf, 0, N*sizeof(buf[0]));
		}

		for (i = 0; i < N; i++)
		{
			cvReleaseImage(&buf[i]);
			buf[i] = cvCreateImage(size, IPL_DEPTH_8U, 1);
			cvZero(buf[i]);// clear Buffer Frame at the beginning
		}
		cvReleaseImage(&mhi);
		mhi = cvCreateImage(size, IPL_DEPTH_32F, 1);
		cvZero(mhi); // clear MHI at the beginning
	
	} // end of if(mhi)
	

	/*����ǰҪ�����֡ת��Ϊ�Ҷȷŵ�buffer�����һ֡��*/
	cvCvtColor(img, buf[last], CV_BGR2GRAY); // convert frame to grayscale

	/*�趨֡�����*/
	idx1 = last;
	idx2 = (last + 1) % N; // index of (last - (N-1))th frame
	last = idx2;



	// ��֡��
	silh = buf[idx2];//��ֵ��ָ��idx2
	cvAbsDiff(buf[idx1], buf[idx2], silh); // get difference between frames

	// �Բ�ͼ������ֵ��
	cvThreshold(silh, silh, 50, 255, CV_THRESH_BINARY); //threshold it,��ֵ��

	//ȥ����ʱ��Ӱ���Ը����˶���ʷͼ��
	cvUpdateMotionHistory(silh, mhi, timestamp, MHI_DURATION); // update MHI
	
	cvConvert(mhi, dst);//��mhiת��Ϊdst,dst=mhi
		
	// ��ֵ�˲�������С������
	cvSmooth(dst, dst, CV_MEDIAN, 3, 0, 0, 0);

	cvPyrDown(dst, pyr, CV_GAUSSIAN_5x5);// ���²�����ȥ��������ͼ����ԭͼ����ķ�֮һ
	cvDilate(pyr, pyr, 0, 1); // �����Ͳ���������Ŀ��Ĳ������ն�
	cvPyrUp(pyr, dst, CV_GAUSSIAN_5x5);// ���ϲ������ָ�ͼ��ͼ����ԭͼ����ı�

	// ����ĳ���������ҵ�����
	// Create dynamic structure and sequence.
	stor = cvCreateMemStorage(0);
	cont = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), stor);

	// �ҵ���������
	cvFindContours(dst, stor, &cont, sizeof(CvContour),
		CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));
	CvRect roi;
	// ֱ��ʹ��CONTOUR�еľ�����������
	for (; cont; cont = cont->h_next)
	{
		CvRect r = ((CvContour*)cont)->rect;
		if (r.height * r.width > CONTOUR_MAX_AERA) // ���С�ķ���������
		{
			CONTOUR_MAX_AERA=r.height * r.width;
			roi=r;
			if (roi.x-roi.width*(ZOOMIN_FACTOR*1.0/2)>0)
			roi.x=roi.x-roi.width*(ZOOMIN_FACTOR*1.0/2);
			else roi.x=0;
			if (roi.y-roi.height*(ZOOMIN_FACTOR*1.0/2)>0)
			roi.y=roi.y-roi.height*(ZOOMIN_FACTOR*1.0/2);
			else roi.y=0;
			if (roi.x+roi.width*(ZOOMIN_FACTOR+1)<img->width)
			roi.width=roi.width*(ZOOMIN_FACTOR+1);
			else roi.width=img->width-roi.x;
			if (roi.y+roi.height*(ZOOMIN_FACTOR+1)<img->height)
			roi.height=roi.height*(ZOOMIN_FACTOR+1);
			else roi.height=img->height-roi.y;
		}
	}
	
				
	// free memory
	cvReleaseMemStorage(&stor);
	cvReleaseImage(&pyr);
	if (roi.width*1.0/roi.height > 1.2 && roi.width*1.0/roi.height < 2.5 )
		return roi;
	else
		{	Rect zero; 
			return zero;
		}
}




