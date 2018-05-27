#ifndef __CVSERVER_HPP__
#define  __CVSERVER_HPP__


#include <iostream>
#include <stdio.h>  
#include <opencv/cv.h>  
#include <opencv2/opencv.hpp>

#include <sys/socket.h> 
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h> 
#include <string.h>



#define PACKAGE_NUM 2  
#define IMG_WIDTH 640 //320  //640
#define IMG_HEIGHT 360 //240 //480 
#define MIN_WIN 15  

#define BLOCKSIZE IMG_WIDTH*IMG_HEIGHT*3/PACKAGE_NUM 

#define OK 0
#define ERROR 1
/*图像数据的接收发送处理*/
struct recvBuf
{
	char buf[BLOCKSIZE];
	int  flag;
};
/*坐标框的接收发送处理*/

union AIMBOX
{
	int intValue[4];
	char charValue[16];

};

/*Socket服务端类*/
class SocketSever
{
public:
	 SocketSever(void);
	~SocketSever(void);


	int setparam(void);
	int connect(void);
	int transmit(cv::Mat& image, int slisten);
	int recvcontral(cv::Rect *box);
	int sendcontral(cv::Rect &box);
	int disconnect(void);

	struct recvBuf data;
	union AIMBOX g_upLoadBoxSever;
private:
	//创建套接字
	int sclient,server;
	int needSend;
	int count;

	struct sockaddr_in remoteAddr;

	
};


#endif