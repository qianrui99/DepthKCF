/**
 * OpenCV video streaming over TCP/IP
 * Server: Captures video from a webcam and send it to a client
 * by Isaac Maia
 */



#include "cvserver.hpp"
/**************************************************
Function: SocketSever,~SocketSever
Description: 构造函数，析构函数
Calls :
Input:
Output:
Return:
Others:
History:
<author>               <time>                <desc>
1)  dawn    Jun,2,2017          
*********************************************************/
SocketSever::SocketSever(void)
{

}

SocketSever::~SocketSever(void)
{

}
/**************************************************
Function: setparam
Description: Socket服务端，参数配置
Input:
Output:
Return:执行结果
Others:
History:
<author>               <time>                <desc>
1)  dawn				Jun,2,2017              修改
*********************************************************/

int SocketSever::setparam(void)
{
  
     server =  socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server == -1)
    {
	    printf("socket error !");
	    return -1;
    }

    //绑定IP和端口
    sockaddr_in sin;
    sin.sin_family = AF_INET;
    sin.sin_port = htons(8888);//端口8888  
    sin.sin_addr.s_addr = INADDR_ANY;
    
    if( bind(server,(struct sockaddr *)&sin , sizeof(sin)) < 0) {
         printf("Can't bind() socket");
         return -1;
    }
    
    //Listening
    listen(server , 3);
    return 1;
}


/**************************************************
Function: connect
Description:连接下位机
Input:
Output:
Return:执行结果
Others:
History:
<author>               <time>                <desc>
1)  dawn				Jun,2,2017             修改
*********************************************************/
int SocketSever::connect(void)
{
    int nAddrlen = sizeof(remoteAddr);

    sclient = accept(server, (struct sockaddr *)&remoteAddr, (socklen_t*)&nAddrlen);
    if(sclient<0){
      return ERROR;
      std::cout<<"Connect error!"<<std::endl;
    }else
      printf("接受到一个连接：%s \r\n", inet_ntoa(remoteAddr.sin_addr));

    return sclient;
}

/**************************************************
Function: socket_receive
Description: 接收下位机上发的数据
Input:
Output:cv::Mat& image 图像指针
Return:执行结果
Others:
History:
<author>               <time>                <desc>
1)  dawn				Jun,2,2017             修改
*********************************************************/

int SocketSever::transmit(cv::Mat& image,int slisten)
{
	cv::Mat img;
	cv::resize(image,img,cv::Size(320,180),0,0,CV_INTER_LINEAR);
	if(img.empty()) {
	    std::cout<<"empty image!"<<std::endl;
	    return ERROR;
	}
	
	if(img.cols != IMG_WIDTH || img.rows != IMG_HEIGHT || img.type()!= CV_8UC3) {
	    std::cout<<"the image mast satisfy : 320*180!"<<std::endl;
	    return ERROR;
	}
      std::cout<<img.cols<<","<<img.rows<<std::endl;
      
      
	size_t imgSize = img.cols*img.rows*3;
	for (int i = 0; i < PACKAGE_NUM; i++)
	{
	    size_t num1 = IMG_HEIGHT / PACKAGE_NUM * i;
	    for (int j = 0; j < IMG_HEIGHT / PACKAGE_NUM; j++)
	    {
		    size_t num2 = j * IMG_WIDTH * 3;
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
	    
	    if(send(slisten, (char*)(&data),sizeof(data),0) < 0){
	      printf("send fail\n");
	      return ERROR;  
	      break;
	    } 
		
	}
	
	std::cout<< sizeof(data)<<","<<slisten<<std::endl;
	
	return OK;
  
}


/**************************************************
Function: sendcontral()
Description: 上行通信，接收指令
Input:
Output:
Return:执行结果
Others:
History:
<author>               <time>                <desc>
1)  dawn				Jun,2,2017            新增
*********************************************************/
int SocketSever::recvcontral(cv::Rect  *box)
{
    int len0;
    union AIMBOX l_uploadBoxClient;


    len0 = recv(sclient, (char *)(&l_uploadBoxClient), 16, 0);

    if (len0<0)
    {
      return ERROR;
    }

    box->x  = l_uploadBoxClient.intValue[0];
    box->y  = l_uploadBoxClient.intValue[1];
    box->width = l_uploadBoxClient.intValue[2];
    box->height = l_uploadBoxClient.intValue[3];
    std::cout<<box->x<<";"<<box->y<<";"<<box->width<<";"<<box->height<<std::endl;
    return OK;
}
/********************************************************
Function: sendcontral()
Description: 下行通信，发送控制指令
Input:
Output:
Return:执行结果
Others:
History:
<author>               <time>                <desc>
1)  dawn				Jun,2,2017              新增
*********************************************************/
int SocketSever::sendcontral(cv::Rect& box)
{
	if(box.width<MIN_WIN || box.height<MIN_WIN) {
	  std::cout << "the box is invalid!" << std::endl;
	  return ERROR;
	}

	if(send(sclient, (char *)(&box), sizeof(box), 0 )<0) {
	  std::cout<<"uploadboxs error:"<<strerror(errno)<<std::endl;
	  return ERROR;
	}
	
	return OK;
}
/********************************************************
Function: disconnect
Description: 关闭socket连接
Input:
Output:c
Return:执行结果
Others:
History:
<author>               <time>                <desc>
1)  dawn				Jun,2,2017              修改
*********************************************************/
int SocketSever::disconnect(void)
{
	close(sclient);
    

	return 1;
}
 