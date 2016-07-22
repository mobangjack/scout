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
	const double MHI_DURATION = 0.5;//0.5s为运动跟踪的最大持续时间
	const double MAX_TIME_DELTA = 0.5; //最大时间增量为0.5s
	const double MIN_TIME_DELTA = 0.05; //最小时间增量0.05s
	const int N = 3;
	int last = 0;
	IplImage **buf = 0;//指针的指针
	IplImage *mhi = 0; // MHI: motion history image 运动历史图像
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
	

// 参数：
// img - 输入视频帧 // dst - 检测结果
CvRect FD::update_mhi(IplImage* img, int diff_threshold)
{
	
	int CONTOUR_MAX_AERA=16;
	double timestamp = clock() / 100.; // get current time in seconds 时间戳
	CvSize size = cvSize(img->width, img->height); // get current frame size，得到当前帧的尺寸
	int i, idx1, idx2;
	IplImage* silh;
	IplImage* pyr = cvCreateImage(cvSize((size.width & -2) / 2, (size.height & -2) / 2), 8, 1);
	
	CvMemStorage *stor;
	CvSeq *cont;

	/*先进行数据的初始化*/
	if (!mhi || mhi->width != size.width || mhi->height != size.height)
	{
		if (buf == 0) //若尚没有初始化则分配内存给他
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
	

	/*将当前要处理的帧转化为灰度放到buffer的最后一帧中*/
	cvCvtColor(img, buf[last], CV_BGR2GRAY); // convert frame to grayscale

	/*设定帧的序号*/
	idx1 = last;
	idx2 = (last + 1) % N; // index of (last - (N-1))th frame
	last = idx2;



	// 做帧差
	silh = buf[idx2];//差值的指向idx2
	cvAbsDiff(buf[idx1], buf[idx2], silh); // get difference between frames

	// 对差图像做二值化
	cvThreshold(silh, silh, 50, 255, CV_THRESH_BINARY); //threshold it,二值化

	//去掉超时的影像以更新运动历史图像
	cvUpdateMotionHistory(silh, mhi, timestamp, MHI_DURATION); // update MHI
	
	cvConvert(mhi, dst);//将mhi转化为dst,dst=mhi
		
	// 中值滤波，消除小的噪声
	cvSmooth(dst, dst, CV_MEDIAN, 3, 0, 0, 0);

	cvPyrDown(dst, pyr, CV_GAUSSIAN_5x5);// 向下采样，去掉噪声，图像是原图像的四分之一
	cvDilate(pyr, pyr, 0, 1); // 做膨胀操作，消除目标的不连续空洞
	cvPyrUp(pyr, dst, CV_GAUSSIAN_5x5);// 向上采样，恢复图像，图像是原图像的四倍

	// 下面的程序段用来找到轮廓
	// Create dynamic structure and sequence.
	stor = cvCreateMemStorage(0);
	cont = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), stor);

	// 找到所有轮廓
	cvFindContours(dst, stor, &cont, sizeof(CvContour),
		CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));
	CvRect roi;
	// 直接使用CONTOUR中的矩形来画轮廓
	for (; cont; cont = cont->h_next)
	{
		CvRect r = ((CvContour*)cont)->rect;
		if (r.height * r.width > CONTOUR_MAX_AERA) // 面积小的方形抛弃掉
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




