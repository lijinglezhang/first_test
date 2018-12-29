#include <stdlib.h>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#if CV_MAJOR_VERSION < 3
#include <opencv2/contrib/contrib.hpp>
#endif
#include <deque>
#include <string>
#include <cerrno>
#include <iostream>
#include <stdint.h>
#include <ctype.h>
#include <vector>
#include "PicoZense_api.h"
#include <fstream>

#include <thread>
#include <chrono>
#include <pthread.h>


using std::cout;
using std::cin;
using namespace std;
using namespace cv;
pthread_mutex_t _mtx;
#define FRAME_QUEUE_SZ		3
#define Zrange             80

static deque< PsFrame > qFrame;
static pthread_mutex_t gmtx;

void FindDoorSeamAndDetectObstacle(Mat _binaryMat,int &count_Obstacle_frame);

const int direction = 0;


void coordinateLine(int &y, int &angle);
cv::Mat rotate(cv::Mat &img, int angle);


cv::Mat rotate(cv::Mat &img, int angle)
{
	cv::Mat imgCopy = img.clone();
	cv::Mat imgRotate;

	cv::Point2f center = cv::Point2f(static_cast<float>(imgCopy.cols / 2), static_cast<float>(imgCopy.rows / 2));
	cv::Mat affineTrans = getRotationMatrix2D(center, angle, 1.0);

	cv::warpAffine(imgCopy, imgRotate, affineTrans, imgCopy.size(), cv::INTER_CUBIC, cv::BORDER_REPLICATE);

	return imgRotate;
}


void coordinateLine(int &y, int &angle) {

	int	minBlack = 255;

	cv::Mat img, imgXRotate;


	cv::Mat imgX = cv::imread("1.jpg");
	cv::Mat imgXCopy = imgX.clone();


	cv::cvtColor(imgXCopy, imgXCopy, CV_BGR2GRAY);
	cv::Mat imgXDirection = rotate(imgXCopy, direction);
	vector<int> gray_line;
	cv::Scalar mean;


	for (int k = -3; k < 3; k++)
	{
		imgXRotate = rotate(imgXDirection, k);

		for (int i = 0; i < imgXRotate.rows - 1; i++)
		{
			img = imgXRotate(cv::Range(i, i + 1), cv::Range::all());
			mean = cv::mean(img);
			 //cout << "mean" << mean << endl;
			if ((int)mean[0] < minBlack)
			{
				minBlack = mean[0];

				//gray_line.push_back(i) ;
				y = i;
				angle = k;

				//std::cout << y << "1111  " << minBlack << "  " << "angle"<<angle<<"mean" << mean << std::endl;
			}
		}
	}

	cv::Mat imgXR = rotate(imgX, direction + angle);

	cv::line(imgXR, cv::Point(0, y), cv::Point(imgXR.cols, y), cv::Scalar(0, 0, 255), 2);
	cv::imshow("Good Matches & Object detection", imgXR);
	cv::waitKey(50);
}



void FindDoorSeamAndDetectObstacle(Mat _binaryMat,int &count_Obstacle_frame)
{

	int angle = 0;
	int y = 0;


    	Mat gray,gray_out;
	vector<int> gray_point ,all_point;
	gray_point.clear();


	//_binaryMat.convertTo(_binaryMat, CV_8U, 1.0/15);
	//threshold(gray, gray_out, 200, 255, THRESH_BINARY);


	Mat srcImg=imread("1.jpg");
	if(!srcImg.data)
	{

		//if(count_image > 10)
		//{
			cout<<"read image error! and create a image"<<endl;
			imwrite("1.jpg",_binaryMat);
		//}
		//else
		//{
		//	return;
		//}

	}


  	if(y == 0)
   	{
		coordinateLine(y, angle);
		angle = direction + angle;
   	}

	Rect rect_edge_1(0, y, 350,1);

cout << ""<<y<<endl;
	_binaryMat = rotate(_binaryMat, angle);
	//gray_out.convertTo(_binaryMat, CV_8U, 255.0);
	//threshold(gray, gray_out, 200, 255, THRESH_BINARY);

	cv::imshow("coordinateLine", _binaryMat);

	_binaryMat = _binaryMat(rect_edge_1);
	//all_point = gray_out;

	cv::imshow("coor", _binaryMat);
	cout<<_binaryMat<<endl;
#if 0
	for(int i = 0; i < all_point.size(); i++)
	{
		if(all_point.at(i) > 0)
		{
			gray_point.push_back(all_point.at(i));
		}
	}

	//cout << "gray_point.size()"<<gray_point.size()<<endl;
	if(gray_point.size() > 2)
	{

		count_Obstacle_frame = 1;
		cout <<"count_Obstacle_frame" <<count_Obstacle_frame<< endl;
	}

#endif

}


static void Opencv_Depth(bool colorMap, uint32_t slope, PsFrameMode depthFrameMode, uint8_t*pData, cv::Mat& mDispImg,int &count_Obstacle_frame)
{


	Rect rect_edge(180, 180, 350,30);


	uint16_t* pDepthData = (uint16_t*)pData;
	for (int i = 0; i < depthFrameMode.resolutionHeight; i++)
	{
		for (int j = 0; j < depthFrameMode.resolutionWidth;j++)
		{

			if(*(pDepthData + i*depthFrameMode.resolutionWidth + j) > 2250)
			{
				*(pDepthData + i*depthFrameMode.resolutionWidth + j)= 0;
			}
		}
	}




	mDispImg = cv::Mat(depthFrameMode.resolutionHeight, depthFrameMode.resolutionWidth, CV_16UC1, pDepthData);

	mDispImg = mDispImg(rect_edge);



	Point2d pointxy(depthFrameMode.resolutionWidth / 2, depthFrameMode.resolutionHeight / 2);
	int val = mDispImg.at<ushort>(pointxy);
	char text[20];
#ifdef _WIN32
	sprintf_s(text, "%d", val);
#else
	snprintf(text, sizeof(text), "%d", val);
#endif
	mDispImg.convertTo(mDispImg, CV_8U, 255.0 / slope);
	if (colorMap)
		applyColorMap(mDispImg, mDispImg, cv::COLORMAP_RAINBOW);
	int color;
	if (val > 2500)
		color = 0;
	else
		color = 4096;

	//circle(mDispImg, pointxy, 4, Scalar(color, color, color), -1, 8, 0);
	//putText(mDispImg, text, pointxy, FONT_HERSHEY_DUPLEX, 2, Scalar(color, color, color));

}






int main(int argc, char* argv[])
{



	const string depthWindow = "Depth Image";
	Mat mDispImg;
	uint32_t slope = 1450;
	bool colorMap = false, bEnabled = true;
	uint16_t threshold = 20;
	int count_image=0 ,count = 0,count_sm = 0 ,count_Obstacle_frame = 0,count_tmp,count_tmp1,count_tmp2,sum_gateseam_obstacle_frame = 0;

	int32_t deviceIndex = 0;
	uint16_t pulseCount = 600;
	PsReturnStatus status;
	status = PsInitialize();
	if (status != PsReturnStatus::PsRetOK)
    {
            cout << "Initialize failed!" << endl;
            system("pause");
            exit(0);
    }


	uint8_t* pData = NULL;
	int32_t deviceCount = 0;
	status = PsGetDeviceCount(&deviceCount);
	//cout << "Get device count status: " << status << " device count: " << deviceCount << endl;
	status = PsOpenDevice(deviceIndex);
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "OpenDevice failed!" << endl;
		system("pause");
		exit(0);
	}
	status = PsSetDepthRange(deviceIndex, PsFarRange);
	status = PsStartFrame(deviceIndex, PsDepthFrame);
	PsSetFilter(deviceIndex, PsComputeRealDepthFilter,bEnabled);
	PsSetPulseCount(deviceIndex, pulseCount);
	PsSetThreshold(deviceIndex, threshold);



	int dataMode = PsDepth_30;
	PsSetProperty(deviceIndex, PsPropertyDataMode_UInt8, &dataMode, sizeof(uint8_t));
	bool empty;
	PsReturnStatus psRetGetDepth = PsRetOK;
	PsFrameMode depthFrameMode;
	PsGetFrameMode(deviceIndex, PsDepthFrame, &depthFrameMode);
	while(1)
	{
		cout << count_image << endl;
                if(count_image > 10000)
		{
			count_image = 0;

		}
		count_image ++;


        PsReadNextFrame(deviceIndex);
		PsFrame depthFrame = {0};
		status = PsGetFrame(deviceIndex, PsDepthFrame, &depthFrame);
		if (status != psRetGetDepth)
		{
			PsGetFrameMode(deviceIndex, PsDepthFrame, &depthFrameMode);

		}


		pthread_mutex_lock(&gmtx);
		if (qFrame.size() < FRAME_QUEUE_SZ)
		{
			qFrame.push_back(depthFrame);
		}
		pthread_mutex_unlock(&gmtx);


		empty = qFrame.empty();
		if (!empty)
		{
			pthread_mutex_lock(&gmtx);
			PsFrame frm = qFrame.front();
			pthread_mutex_unlock(&gmtx);
			if(frm.pFrameData != NULL)
			{



				Opencv_Depth(colorMap, slope, depthFrameMode, frm.pFrameData, mDispImg,count_Obstacle_frame);
				FindDoorSeamAndDetectObstacle(mDispImg,count_Obstacle_frame);
				sum_gateseam_obstacle_frame = sum_gateseam_obstacle_frame + count_Obstacle_frame;

				if( sum_gateseam_obstacle_frame == 1)
				{
					count_tmp = count_image;
				}
				else if( sum_gateseam_obstacle_frame == 6)
				{
					count_tmp1 = count_image;
				}
				count_tmp2 = count_image;
				cout <<"sum_gateseam_obstacle_frame"<<sum_gateseam_obstacle_frame<<endl;
				if( sum_gateseam_obstacle_frame == 6 && abs(count_tmp1 - count_tmp) <= 6)
				{
				   cout << "很明显有障碍物"<<endl;

					sum_gateseam_obstacle_frame = 0;
				   //break;
				}
				else if(( sum_gateseam_obstacle_frame > 0&& abs(count_tmp - count_tmp2) > 6)||count_Obstacle_frame == 0 )
				{

					sum_gateseam_obstacle_frame = 0;
				}



                   count_Obstacle_frame = 0;

				//pcl::io::savePCDFileASCII("test_pcd1.pcd",cloud_filtered_s);
				cv::imshow(depthWindow, mDispImg);
				waitKey(10);
			}
		}



		pthread_mutex_lock(&gmtx);
        qFrame.pop_front();
        pthread_mutex_unlock(&gmtx);
	//Frame process
	}

	status = PsStopFrame(deviceIndex, PsDepthFrame);
	status = PsStopFrame(deviceIndex, PsIRFrame);
	status = PsCloseDevice(deviceIndex);
	//Shutdown PicoZense SDK
	status = PsShutdown();
	return 0;
}
