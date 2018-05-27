
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
#include <sl_zed/Camera.hpp>

// OpenCV includes
#include <opencv2/highgui.hpp>
/*=======================================*/
/*=========[ADD][OPENTLD][2017-6-2]=====================*/
#include <iostream>
/*==============[ADD][KCF][2017-6-3]=====================*/

#include "kcftracker.hpp"
#include "usart.h"
#include "cvserver.hpp"

using namespace sl;


#define FRAME_WIDTH  320
#define FRAME_HEIGHT 180



SocketSever cvSocketServer;

cv::Rect result(0,0,0,0);
cv::Rect box(10,0,0,0); 
bool drawing_box = false;
bool gotBB = false;

cv::Mat tImage;

/*=================Declare Function========================*/
void drawBox(cv::Mat& image, cv::Rect box);
void mouseHandler(int event, int x, int y, int flags, void *param);
cv::Mat slMat2cvMat(Mat& input);
void printHelp();

void *display(void *ptr);
void *transmit(void *ptr);

int main(int argc, char **argv) {

      

      
      int ssClient=0;
/*
	if(!cvSocketServer.setparam()){
	    std::cout<<"Socket init error"<<std::endl;
	}*/
// 	ssClient = cvSocketServer.connect();
// 	 if(ssClient<=0){
// 	    std::cout<<"Socket connect error"<<std::endl;
// 	}
//     

    int localSocket,remoteSocket,port = 8888;
    struct sockaddr_in localAddr, remoteAddr;
    pthread_t thread_id;
    
    int addrLen = sizeof(struct sockaddr_in);
    localSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(localSocket == -1) {
	std::perror("socket() call failed!");
    }
    
    localAddr.sin_family = AF_INET;
    localAddr.sin_addr.s_addr = INADDR_ANY;
    localAddr.sin_port = htons( port);
    
    if(bind(localSocket,(struct sockaddr *)&localAddr, sizeof(localAddr))<0) {
	std::perror("Can't bind() socket");
	exit(1);
    }
    
    listen(localSocket, 3);
    std::cout<< "Waiting for connections...\n"
	      << "Server Port:"<<port<<std::endl;
    // Loop until 'q' is pressed
    char key = ' ';
    
    while (key != 'q') {
	

/////////////////////////////////socket>> image////////////////////////////////////////////////
	    ///////////////////////////socke<<box///////////////////////////////////////////////
	    remoteSocket = accept(localSocket, (struct sockaddr *)&remoteAddr, (socklen_t*)&addrLen);
	    
	    if(remoteSocket <0) {
		std::perror("accept failed");
		exit(1);
	    }
	    std::cout << "Connection accepted"<< std::endl;

	       pthread_create(&thread_id,NULL,display,&remoteSocket);
	   
	}
    
    return 0;
}



void *display(void *ptr){
	int socket = *(int *)ptr;
  
	pthread_t thread_id;
   
  	double t = 0;
	double Fps = 0;
	/******局部变量定义****/
	bool HOG = true;
	bool FIXEDWINDOW = false;
	bool MULTISCALE = true;
	bool LAB = false;
	
	int certrex,certrey;
	float certrez=0;
	
	cv::Point pos = cv::Point(20,20);				//The Position of Text
/*========================*/
//目标统计，检测到的目标数
      int  nframes = 0;
       char key = ' ';
  /*==========================USART===================================*/
	int fd;                            //文件描述符    
	int len;       
	unsigned char stopData[4]={0}; 
	unsigned char data[4]={0};  //send buffer for usart	
	unsigned char end[2]={0};
			   
	int checkout=0;		//checkout bits
	fd = UART0_Open(fd); //打开串口2，返回文件描述符    
	UART0_Init(fd,115200,0,8,1,'N');
	
	
/*==================================================================*/    
    // Create KCFTracker object ......
	KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

	
    // Create a ZED camera object
    Camera zed;
    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION_HD720;   //RESOLUTION_VGA 100fps;RESOLUTION_HD720 60fps
    init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
    init_params.coordinate_units = UNIT_METER;    
    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS) {
        printf("%s\n", toString(err).c_str());
        zed.close();
         // Quit if an error occurred
    }

    // Display help in console
    printHelp();

    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_STANDARD;  //SENSING_MODE_STANDARD;   SENSING_MODE_FILL

    // Prepare new image size to retrieve half-resolution images
    Resolution image_size = zed.getResolution();
    int new_width = image_size.width / 2;
    int new_height = image_size.height / 2;

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    Mat image_zed(new_width, new_height, MAT_TYPE_8U_C4);
    cv::Mat image_ocv = slMat2cvMat(image_zed);
    Mat depth_image_zed(new_width, new_height, MAT_TYPE_8U_C4);
    cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed);
    Mat point_cloud;

 cv::Mat image, grayImage, blurImage, depth;
    while (true) {
	if (zed.grab(runtime_parameters) == SUCCESS) {
	    t = (double)cv::getTickCount(); 

	    cvNamedWindow("KCF", 1);
	    cvSetMouseCallback("KCF", mouseHandler);


	    // Retrieve the left image, depth image in half-resolution
	    zed.retrieveImage(image_zed, VIEW_LEFT, MEM_CPU, new_width, new_height);
	    zed.retrieveImage(depth_image_zed, VIEW_DEPTH, MEM_CPU, new_width, new_height);

	    // Retrieve the RGBA point cloud in half-resolution
	    // To learn how to manipulate and display point clouds, see Depth Sensing sample
	    zed.retrieveMeasure(point_cloud, MEASURE_XYZRGBA, MEM_CPU, new_width, new_height);

	    //Notice:There has transfer the TYPE_8U_C4 to TYPE_8U_C3
	    cv::cvtColor(image_ocv,image,cv::COLOR_BGRA2BGR);
	    cv::cvtColor(depth_image_ocv,depth,cv::COLOR_BGRA2BGR);


	    cv::imshow("KCF", image);
	    image.copyTo(tImage);

	     pthread_create(&thread_id,NULL,transmit,&socket);
	    if(gotBB || std::min(box.width, box.height) < MIN_WIN)	{
		drawBox(image,box);
		cv::putText(image,"Select Target Block With Mouse",pos,cv::FONT_HERSHEY_TRIPLEX,0.4,(0,255,255),1,CV_AA);
		cv::imshow("KCF", image);
// 		printf("image width:%d,height:%d\n",image.cols,image.rows);
// 		printf("Initial Bounding Box = x:%d y:%d h:%d w:%d\n,size:%d", box.x, box.y, box.width, box.height,sizeof(box));	
		UART0_Send(fd,stopData,4); UART0_Send(fd,end,2);
	    } else {
		// First frame, give the groundtruth to the tracker
		if(nframes<2) {
		    nframes++;
		    tracker.init(box, image);
		}
		if(nframes>=2) {
		    result = tracker.update(image);
		    /*		certrex[0] = result.x  >> 8;
		    certrex[1] = result.x & 0xff;
		    certrey[0] = result.y   >> 8;
		    certrey[1] = result.y & 0xff;	
		    */				
		    certrex = (result.x+result.width/2)/2;
		    certrey = (result.y+result.height/2)/2;
		    data[0] = certrex  >> 8;;
		    data[1] = certrex & 0x000000ff;
		    data[2] = certrey   >> 8;
		    data[3] = certrey & 0x000000ff;
		    //						data[4] = (result.x +result.y)>>8;
		    //						data[5] = (result.x +result.y)& 0x000000ff;	


		    end[0] = 0x0D;
		    end[1] = 0x0A;	
		    UART0_Send(fd,data,4); 
		    UART0_Send(fd,end,2); 
		    sl::float4 point_cloud_value;
		    point_cloud.getValue(certrex, certrey, &point_cloud_value);
		    certrez = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);
		    printf("certrex:%d certrey:%d certrez:%.2f\n",certrex,certrey,certrez);
		    //printf("%x,%x,%x,%x,%x,%x,%x,%x\n",data[0],data[1],data[2],data[3],end[0],end[1]);

		    drawBox( image,result);
		    
		    ///////////////////////////socke>>result///////////////////////////////////////////////
		    cv::putText(image," ESC: Exit Program!  r: Reset The Program! ",pos,cv::FONT_HERSHEY_TRIPLEX,0.4,(0,255,255),1,CV_AA);
		    cv::imshow("KCF", image);
		    nframes=3;
		}
	    }
	    cv::imshow("depth", depth);
	    
	    
	    
	    // Handle key event
	    key = cv::waitKey(10);

	    switch(key) {
	    case 'r':gotBB = false;nframes = 0;break;
	    case 'p':UART0_Send(fd,stopData,4); UART0_Send(fd,end,2);cv::waitKey(5000);break;
	    case 's':zed.close();break;
	    }

	    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
	    Fps = 1.0 / t;
	    //printf("FPS: %.2f\n",Fps);
	}
	
    }
	
		
/*==========================Socket===================================*/

 //cvSocketServer.sendcontral(box);
}




void *transmit(void *ptr){
  int socket = *(int *)ptr;
  int bytes =0;
  size_t imgSize;
  
  cv::Mat img;
  struct recvBuf data;
	if(!tImage.empty()) {
// 		if(!cvSocketServer.transmit(tImage,socket)){
// 		    std::cout<<"transmit ok!"<<std::endl;
// 		}
// 		else{
// 		    std::cout<<"transmit error!"<<std::endl;
// 
// 		}
	 // cv::resize(tImage,img,cv::Size(640,360),0,0,CV_INTER_LINEAR);
	  tImage.copyTo(img);  
		if(img.empty()) {
		      std::cout<<"empty image!"<<std::endl;
		  }
		  
		  if(img.cols != IMG_WIDTH || img.rows != IMG_HEIGHT || img.type()!= CV_8UC3) {
		      std::cout<<"the image mast satisfy : 320*180!"<<std::endl;
		  }
		std::cout<<img.cols<<","<<img.rows<<std::endl;
		
		
		 size_t imgSize = img.cols*img.rows*3;
		  for (int i = 0; i < PACKAGE_NUM; i++)
		  {
		      int num1 = IMG_HEIGHT / PACKAGE_NUM * i;
		      for (int j = 0; j < IMG_HEIGHT / PACKAGE_NUM; j++)
		      {
			      int num2 = j * IMG_WIDTH * 3;
			      uchar* ucdata = img.ptr<uchar>(j + num1);
			      for (int k = 0; k < IMG_WIDTH * 3; k++)
			      {
				      data.buf[num2 + k] = ucdata[k];
			      }
		      }
		      if (i == PACKAGE_NUM -1)
			data.flag = 2;
		      else
			data.flag = 1;
		      
		      if(send(socket, (char*)(&data),sizeof(data),0) < 0){
			printf("send fail\n");
		      } 
			  
		  }
		  
	    cv::waitKey(30);
	}
}



/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}

/**
* This function displays help in console
**/
void printHelp() {
    std::cout << " Press 's' to save Side by side images" << std::endl;
    std::cout << " Press 'p' to save Point Cloud" << std::endl;
    std::cout << " Press 'd' to save Depth image" << std::endl;
    std::cout << " Press 'm' to switch Point Cloud format" << std::endl;
    std::cout << " Press 'n' to switch Depth format" << std::endl;
}

/*==============函数定义===========================*/
void drawBox(cv::Mat& image, cv::Rect box){
	cv::rectangle(image, cv::Point(box.x, box.y), cv::Point(box.x + box.width, box.y + box.height), cv::Scalar(0, 255, 255), 1,8);
}

// //bounding box mouse callback
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
			box = cv::Rect(x, y, 0, 0);
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


