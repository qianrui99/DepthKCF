
/*************************************************
- Copyright (C), 2017-2018, XUST
- File name:    main_all.cpp
- Author: Dawn      Version:  V1.0.0      Date: Nov,14,2017
- Description:      目标跟踪         
- Others:        
- Function List:  
1. ....
History:   <author>    <date>   <modification>
1.    		Dawn   		Nov,14,2017		Make
2.    		Dawn   		Nov,14,2017		Modify Main   		
3.    .......
*************************************************/


#include <stdio.h> 
#include <unistd.h> 
#include <stdlib.h> 
#include <opencv2/opencv.hpp>

/*=======================================*/
/*=========[ADD][OPENTLD][2017-6-2]=====================*/
#include <iostream>
/*==============[ADD][KCF][2017-6-3]=====================*/
#include "cv.h"
#include "highgui.h"
#include "kcftracker.hpp"


using namespace cv;
using namespace std;



#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480
#define MIN_WIN 15  	


Rect result(0,0,0,0);
Rect box(0,0,0,0); 
bool drawing_box = false;
bool gotBB = false;


/*=================Declare Function========================*/
void drawBox(Mat& image, Rect box);
void mouseHandler(int event, int x, int y, int flags, void *param);

int main(int argc, char* argv[])
{
/******局部变量定义****/
	bool HOG = true;
	bool FIXEDWINDOW = false;
	bool MULTISCALE = true;
	bool LAB = false;
	double t = 0;
	double Fps = 0;
	
	Point pos = Point(20,20);				//The Position of Text
/*========================*/
//目标统计，检测到的目标数
    int  nframes = 0;


	// Create KCFTracker object ......
      KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);


     VideoCapture capture(1);
	// capture.open(0);
	if (!capture.isOpened())
	{
	  cout << "capture device failed to open!\n" << endl;
	  return 0;
	}



   Mat frame;

/*=========设置摄像头采集图像尺寸=================*/
   capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
   capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
/*=========与上行通行通行，确定目标框==============*/


	while(capture.isOpened())
	{
		capture >> frame;	
		cvNamedWindow("KCF", 1);
		cvSetMouseCallback("KCF", mouseHandler);

		if(!gotBB || min(box.width, box.height) < MIN_WIN)	{
			drawBox(frame,box);
			putText(frame,"Select Target Block With Mouse",pos,FONT_HERSHEY_TRIPLEX,0.8,(10,255,255),2,CV_AA);
			imshow("KCF", frame);
			printf("image width:%d,height:%d\n",frame.cols,frame.rows);
			printf("Initial Bounding Box = x:%d y:%d h:%d w:%d\n,size:%d", box.x, box.y, box.width, box.height,sizeof(box));	
			
		} else {
			// First frame, give the groundtruth to the tracker
			if(nframes<2) {
				nframes++;
				tracker.init(box, frame);
			}
			if(nframes>=2) {
				result = tracker.update(frame);
				printf("target box = x:%d y:%d h:%d w:%d\n",result.x,result.y,result.width,result.height);
				drawBox( frame,result);
				putText(frame," ESC: Exit Program!  r: Reset The Program! ",pos,FONT_HERSHEY_TRIPLEX,0.8,(10,255,255),2,CV_AA);
				imshow("KCF", frame);
				nframes=3;
			}
		}
		char c = (char)waitKey(30);
		if( c == 27 )
			break;
		switch(c) {
			case 'r':
				gotBB = false;
				nframes = 0;
				break;
		}
	}
	return 1;
}



/*==============函数定义===========================*/
void drawBox(Mat& image, Rect box){
	rectangle(image, cvPoint(box.x, box.y), cvPoint(box.x + box.width, box.y + box.height), Scalar(0, 255, 255), 1,8);
}

//bounding box mouse callback
void mouseHandler(int event, int x, int y, int flags, void *param){
	switch (event){
		case CV_EVENT_MOUSEMOVE:
			if (drawing_box){
				box.width = x - box.x;
				box.height = y - box.y;
			}
			break;
		case CV_EVENT_LBUTTONDOWN:
			drawing_box = true;
			box = Rect(x, y, 0, 0);
			break;
		case CV_EVENT_LBUTTONUP:
			drawing_box = false;
			if (box.width < 0){
				box.x += box.width;
				box.width *= -1;
			}
			if (box.height < 0){
				box.y += box.height;
				box.height *= -1;
			}
			gotBB = true;
			break;
	}
}
