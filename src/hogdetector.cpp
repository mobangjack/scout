#include <iostream>
#include <opencv2/opencv.hpp>

#include <ios>
#include <fstream>
#include "hogdetector.hpp"

using namespace std;
using namespace cv;
using namespace cv::ml;


void HOGDetector::load(const char* model)
{
	Ptr<SVM> svm = SVM::load<SVM>(model); 
	
	int DescriptorDim = svm->getVarCount();//����������ά������HOG�����ӵ�ά��  
	Mat supportVectorMat = svm ->getSupportVectors();//֧����������  
	int supportVectorNum = supportVectorMat.rows;//֧�������ĸ���  
	cout << "support Vector Num   " << supportVectorNum << endl;

	Mat alphaMat = Mat::zeros(supportVectorNum, DescriptorDim, CV_32FC1);//alpha���������ȵ���֧����������  
	Mat svindex = Mat::zeros(1, supportVectorNum,CV_64F);
	Mat resultMat ;//alpha��������֧����������Ľ��  

	double rho = svm ->getDecisionFunction(0, alphaMat, svindex);

	alphaMat.convertTo(alphaMat, CV_32F);
	//����-(alphaMat * supportVectorMat),����ŵ�resultMat��  
	//gemm(alphaMat, supportVectorMat, -1, 0, 1, resultMat);//��֪��Ϊʲô�Ӹ��ţ�  
	resultMat = -1 * alphaMat * supportVectorMat;

	//�õ����յ�setSVMDetector(const vector<float>& detector)�����п��õļ����  
	
	vector<float> myDetector;
	//��resultMat�е����ݸ��Ƶ�����myDetector��  
	for (int i = 0; i < DescriptorDim; i++)
	{
		myDetector.push_back(resultMat.at<float>(0, i));
	}
	//������ƫ����rho���õ������  
	myDetector.push_back(rho);
	cout << "Detector Size   " << myDetector.size() << endl;
	
	hog.setSVMDetector(myDetector);
}

vector<Rect>  HOGDetector::detect(const Mat& img)
{
	cout << "Runing the multi-scale object detection." << endl;
	vector<Rect> found, found_filtered;
	hog.detectMultiScale(img, found, 0, Size(8, 8), Size(0, 0), 1.05, 2);
	cout << "Rect NUM    " << found.size() << endl;
	for (int i = 0; i < found.size(); i++)
	{
		Rect r = found[i];
		int j = 0;
		for (; j < found.size(); j++)
			if (j != i && (r & found[j]) == r)
				break;
		if (j == found.size()) {
			r.x += cvRound(r.width*0.1);
			r.width = cvRound(r.width*0.8);
			r.y += cvRound(r.height*0.07);
			r.height = cvRound(r.height*0.8);
			found_filtered.push_back(r);
		}
	}
	return found_filtered;
}

