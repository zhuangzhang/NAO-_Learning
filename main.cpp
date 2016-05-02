//---------------------------------【NAO所包含的API】----------------------------
//		描述：调用NAO的API所需要包含的头文件
//-------------------------------------------------------------------------------
#include <iostream>
#include <alerror/alerror.h>
//---------------------------------视觉----------------------------
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
//---------------------------------运动----------------------------
#include <alproxies/almotionproxy.h>
//---------------------------------模块间通信----------------------
#include <alproxies/dcmproxy.h>
//---------------------------------内存储存关键信息和硬件配置------
#include <alproxies/almemoryproxy.h>
//---------------------------------声呐----------------------------
#include <alproxies/alsonarproxy.h>
//---------------------------------传感器--------------------------
#include <alproxies/alsensorsproxy.h>
//---------------------------------追踪器--------------------------
#include <alproxies/altrackerproxy.h>  
//---------------------------------说话----------------------------
#include <alproxies/altexttospeechproxy.h>
//---------------------------------【OpenCV】------------------------------------
//		描述：使用OpenCV进行基本的绘图操作所需要包含的头文件
//-------------------------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//---------------------------------【命名空间】----------------------------------
using namespace cv;
//---------------------------------【其他头文件】--------------------------------
#include <qi/os.hpp>
#include <math.h>
#include <time.h>
#include <stdlib.h>
//---------------------------------【调用子函数】--------------------------------
#include "NAOrobot.h"
//-----------------------------------【宏定义部分】------------------------------
//		描述：定义一些辅助宏 
//-------------------------------------------------------------------------------
#define WINDOW_NAME1 "【坐标信息】"        //为窗口标题定义的宏 
#define WINDOW_NAME2 "【距离】"        //为窗口标题定义的宏 
#define WINDOW_CHANG 400//定义窗口大小的宏
#define WINDOW_KUANG 600//定义窗口大小的宏
#define WINDOW_WIDTH 150//定义窗口大小的宏

#define WINDOW_JIA 200  //定义增加值
//--------------------------------【全局函数声明部分】-------------------------------------
//		描述：全局函数声明
//-----------------------------------------------------------------------------------------
//void huazuobiao(Mat atomImage, float xballPositoin,float yballPositoin);
//void DrawFilledCircle( Mat img, Point center );
void DrawFilledCircle(float balldist, Mat img, Point center );
//void RedBallTracker(Mat matOriginal);
//--------------------------------【全局变量声明部分】-------------------------------------
//		描述：全局变量声明
//-----------------------------------------------------------------------------------------
bool robotHalted = false;
bool NewTargetDetect = false;//new
double time_from_start = 0;



//---------------------------------------【main( )函数】-----------------------------------
//		描述：控制台应用程序的入口函数，我们的程序从这里开始执行
//-----------------------------------------------------------------------------------------
int main(int argc, char* argv[]) {
	if(argc != 2){
		std::cerr << "Wrong number of arguments!" << std::endl;
		std::cerr << "Usage: RApath NAO_IP" << std::endl;
		exit(2);
	}

// ---------------------模块初始化及订阅------------------------
		// 初始化声纳和内存代理启动声纳 
	AL::ALSonarProxy sonarPrx(argv[1], 9559);
	AL::ALMemoryProxy memPrxSonar(argv[1], 9559);
	sonarPrx.subscribe("RAPlanner");
	int period = sonarPrx.getMyPeriod("RAPlanner");
		//------------------------------------------------------
		//初始化缓冲器和内存代理 
	AL::ALSensorsProxy headTouchPrx(argv[1], 9559);
	AL::ALMemoryProxy memPrxBumper(argv[1], 9559);
	headTouchPrx.subscribe("RAPlanner");
	float headTouched = memPrxBumper.getData("FrontTactilTouched");
		//初始化说话
	AL::ALTextToSpeechProxy textToSpeech(argv[1], 9559);
		// 初始化运动代理 
	AL::ALMotionProxy motionPrx(argv[1], 9559);
		//初始化红球追踪 
	AL::ALTrackerProxy redBallPrx(argv[1], 9559);
		//初始化DCM
	AL::DCMProxy dcm_proxy(argv[1], 9559);
		//初始化摄像头及订阅TOP摄像头
	/*AL::ALVideoDeviceProxy camProxy(argv[1], 9559);
	camProxy.setActiveCamera(AL::kTopCamera);        // Connect to top camera.
	const std::string clientName = camProxy.subscribe("test", AL::kQVGA, AL::kBGRColorSpace, 30);
	Mat imgload = Mat(cv::Size(320, 240), CV_8UC3);	*/
		// 创建空白的Mat图像
	Mat atImage = Mat::zeros( WINDOW_KUANG, WINDOW_CHANG, CV_8UC3 );
	
		// 设置值来存储当前测量距离 
	AL::ALValue rightDist, leftDist;
	float rightD, leftD;
		// 获取声纳值(位置在内存中) 
	const std::string keyR = "Device/SubDeviceList/US/Right/Sensor/Value";
	const std::string keyL = "Device/SubDeviceList/US/Left/Sensor/Value";
		// 初始化行走速度变量 
	float Vx = 0;
	float Vy = 0;
	float Om = 0;
		//获取NAO姿势数据及摇头数据
	bool useSensorValues = true;
	std::vector<float> pose = motionPrx.getRobotPosition(useSensorValues);
	std::vector<float> headAngle = motionPrx.getAngles("HeadYaw", true);
		//设置识别红球前的头部初始位置
	motionPrx.setStiffnesses("Head", 1.0f);
	motionPrx.setAngles("HeadYaw", 0.0f, 0.2f);//摇头
	motionPrx.setAngles("HeadPitch", 0.0f, 0.2f);//点头
	//初始化变量
	float ballMag = 1000.0f;
	int ballCounter = 0;
	std::vector<float> ballPose;//
	ballPose.push_back(1000.0f);//
	ballPose.push_back(0.0f);
	ballPose.push_back(0.0f);

// ---------------------执行红球识别与追踪------------------------
		//定义目标参数
	redBallPrx.registerTarget("RedBall",0.06);//单位 m
		//实现追踪目标
	redBallPrx.track("RedBall");  

	motionPrx.moveInit();

	motionPrx.moveToward(Vx, Vy, Om);//new

		/* 文件处理：如果文件不存在则创建，如果文件已存在则把文件
		长度截断为0字节再重新写，也就是替换掉原来的文件内容.*/
	FILE* fh = fopen("sonarLog1.txt", "w");
		//计时
	int t1 = dcm_proxy.getTime(0);	
		
	try {
		std::cout << "Test" << std::endl;
			
		while(true)
		{
			//AL::ALValue img = camProxy.getImageRemote(clientName);	//远程调取图像
			//imgload.data = (uchar*) img[6].GetBinary();//存在imgload
			//RedBallTracker(imgload);
			////camProxy.releaseImage(clientName);
			////imshow("NAO摄像头",imgload);
				//检测头部是否Touched
			headTouched = memPrxBumper.getData("FrontTactilTouched");
			if(headTouched) robotHalted = true;
			bool NewTarget = redBallPrx.isNewTargetDetected();//检测是否新目标，如果是，则返回true
			if(NewTarget) NewTargetDetect = true;

			if(!robotHalted)
			{
				int t2 = dcm_proxy.getTime(0);	//计时		
				time_from_start = (t2-t1)/1000.0;//距开始时间
					// 更新目标位置
				if(NewTarget)
				{    
					std::cout<<"---------------------锁定目标-------------------\n";
					ballPose = redBallPrx.getTargetPosition(0);//需要在()中添加0 1 2选择坐标系;
						// 转换为厘米 
					ballPose[0] *= 100;
					ballPose[1] *= 100;
					ballPose[2] *= 100;
					//求距离
					ballMag = sqrt(pow(ballPose[0],2) + pow(ballPose[1],2));
					//画坐标图
					DrawFilledCircle(ballMag, atImage, Point( ballPose[1]+WINDOW_JIA,ballPose[0]) );
				}
				else
				{
					std::cout<<"---------------------寻找目标-------------------\n";
						//设置头位置水平
					motionPrx.setAngles("HeadYaw", 0.0f, 0.2f);
					motionPrx.setAngles("HeadPitch", 0.0f, 0.2f);
					
				}
					// 获得声呐参数
				rightDist = memPrxSonar.getData(keyR);
				leftDist = memPrxSonar.getData(keyL);
					//转换为厘米 
				rightD = 100*float(rightDist);
				leftD  = 100*float(leftDist);
					//调用子函数robotNav
				robotNav(ballPose, Om, Vx, rightD, leftD);

					//判断 球距离阈值 、球计数器阈值 
				if(ballMag < 50.0f)
				{
					if(ballCounter >= 6)
					{
						Vx = 0;
						Om = 0;
						//已经找到目标
						robotHalted = true;
						std::cout << std::endl << " GOAL REACHED " << std::endl;
						textToSpeech.say("goal reached!");

					}
					ballCounter++;
					std::cout << " Ball Counter: " << ballCounter << std::endl;
				}

				// Constrain velocities限制速度 
				if(Vx < -1.0f) Vx = -1.0f;
				if(Vx > 1.0f) Vx = 1.0f;
				if(Om < -1.0f) Om = -1.0f;
				if(Om > 1.0f) Om = 1.0f;
			

					// Update Walk更新走
				motionPrx.moveToward(Vx, Vy, Om);

					// Update Pose更新姿势
				pose = motionPrx.getRobotPosition(useSensorValues);

					// Get Head Yaw获得头的偏转角度 
				headAngle = motionPrx.getAngles("HeadYaw", true);

					// Print Values
			/*	std::cout << " LDist: " << leftD;
				std::cout << " RDist: " << rightD << std::endl;
				std::cout << " Vx: " << Vx << " Om: " << Om << std::endl;
				std::cout << " Period (ms): "<<time_from_start;
				std::cout << " Pose: " << pose << std::endl;
				std::cout << " Ball Pose ";
				std::cout << " x: " << ballPose[0];
				std::cout << " y: " << ballPose[1];
				std::cout << " z: " << ballPose[2];*/
				std::cout << " ballMag: " << ballMag << std::endl;
				std::cout << std::endl;
				
					// 将声纳数据 写入fh文件
				fprintf(fh,"%lf %f %f %f %f %f %f \n", time_from_start, ballMag, ballPose[0], ballPose[1], leftD, rightD, headAngle);//,ballPose[2]
				//fprintf(fh,"%lf %f %f %f %f %f %f \n", time_from_start, ballMag, ballPose[0], ballPose[1], pose[0], pose[1],pose[2]);
			}
			else
			{
				
				std::cout << "Robot Halted." << std::endl;//暂停并停止所有运动
				motionPrx.moveToward(0.0f, 0.0f, 0.0f);
				redBallPrx.stopTracker();
				motionPrx.setStiffnesses("Head", 0.0f);
				qi::os::sleep(30);
				fclose(fh);
				
			}
		}
			//取消摄像头订阅
		//camProxy.unsubscribe(clientName);

	}
	catch (const AL::ALError& e)
	 {
		//motionPrx.setWalkTargetVelocity(0.0f, 0.0f, 0.0f, 0.0f);//和上面一样被弃用
		motionPrx.moveToward(0.0f, 0.0f, 0.0f);
	    std::cerr << "Caught exception: " << e.what() << std::endl;
		exit(1);
	 }
	exit(0);//退出主程序,并返回0.
}


void DrawFilledCircle(float balldist, Mat img, Point center )
{

	int thickness = -1;
	int thickness1 = 2;
	int lineType = 8;
	Point center1 = Point(200,0);
	circle( img,
		center1,
		50,
		Scalar( 255, 129, 0 ),//蓝色区域
		thickness1,
		lineType );
	circle( img,
		center1,
		10,
		Scalar(  0, 0, 255 ),//红色红球
		thickness,
		lineType );

	circle( img,
		center,
		2,
		Scalar( 0, 255, 0 ),//绿色散点
		thickness,
		lineType );
	imshow( WINDOW_NAME1, img );
	//插入文字
	//参数为：承载的图片，插入的文字，文字的位置（文本框左下角），字体，大小，颜色
	Mat atImage1 = Mat::zeros( WINDOW_WIDTH, WINDOW_WIDTH, CV_8UC3 );
	char buf[10];
	itoa(balldist, buf, 10);
	//string words= "good luck";
	putText( atImage1, buf, Point( WINDOW_WIDTH/2-20,WINDOW_WIDTH/2),CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0) );
	imshow( WINDOW_NAME2, atImage1 );
	//destroyWindow(WINDOW_NAME2);

	waitKey(1);
}
/*void RedBallTracker(Mat matOriginal)
{
	
	//cv::Mat matOriginal;		// input image
	cv::Mat matProcessed;		// output image

	std::vector<cv::Vec3f> v3fCircles;				// 3 element vector of floats, this will be the pass by reference output of HoughCircles()

	char charCheckForEscKey = 0;
		// smooth the image 降噪，高斯滤波（低通是模糊，高通是锐化），高斯平滑（平滑处理或者说模糊处理）
		cv::GaussianBlur(matOriginal,			// function input
			matProcessed,						// function output
			cv::Size(5, 5),						// smoothing window width and height in pixels
			2);									// sigma value, determines how much the image will be blurred

		// filter on color
		cv::inRange(matProcessed,				// funcion input
			//cv::Scalar(0, 0, 175),				// min filtering value (if greater than or equal to this) (in BGR format)
			cv::Scalar(0, 0, 140),	
			cv::Scalar(100, 100, 256),			// max filtering value (and if less than this) (in BGR format)
			matProcessed);						// function output
		// cv::inRange(matProcessed,				// funcion input黄球识别
		// 	cv::Scalar(0, 100, 200),				// min filtering value (if greater than or equal to this) (in BGR format)
		// 	cv::Scalar(100, 200, 256),			// max filtering value (and if less than this) (in BGR format)
		// 	matProcessed);						// function output

		// smooth again
		cv::GaussianBlur(matProcessed,			// function input
			matProcessed,						// function output
			cv::Size(5, 5),						// smoothing window width and height in pixels
			2);									// sigma value, determines how much the image will be blurred

		cv::dilate(matProcessed, matProcessed, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));		// close image (dilate, then erode)
		cv::erode(matProcessed, matProcessed, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));		// closing "closes" (i.e. fills in) foreground gaps
		//先腐蚀后膨胀去除小黑点

		// fill circles vector with all circles in processed image
		cv::HoughCircles(matProcessed,			// 是个找圆算法input image
			v3fCircles,							// 圆的输出矢量function output (must be a standard template library vector
			CV_HOUGH_GRADIENT,					// nt类型的method，即使用的检测方法，目前OpenCV中就霍夫梯度法一种可以使用，它的标识符为CV_HOUGH_GRADIENT，在此参数处填这个标识符即可。two-pass algorithm for detecting circles, this is the only choice available
			2,									// size of image / this value = "accumulator resolution", i.e. accum res = size of image / 2
			matProcessed.rows / 4,				// min distance in pixels between the centers of the detected circles
			100,								// high threshold of Canny edge detector (called by cvHoughCircles)						
			50,									// low threshold of Canny edge detector (set at 1/2 previous value)
			10,									// min circle radius (any circles with smaller radius will not be returned)
			400);								// max circle radius (any circles with larger radius will not be returned)

		for (int i = 0; i < v3fCircles.size(); i++) {		// for each circle . . .all the circles detected.
			// show ball position x, y, and radius to command line
			std::cout << "ball position x = " << v3fCircles[i][0]			// x position of center point of circle
			<< ", y = " << v3fCircles[i][1]								// y position of center point of circle
			<< ", radius = " << v3fCircles[i][2] << "\n";				// radius of circle

			// draw small green circle at center of detected object
			cv::circle(matOriginal,												// draw on original image
				cv::Point((int)v3fCircles[i][0], (int)v3fCircles[i][1]),		// center point of circle
				3,																// radius of circle in pixels
				cv::Scalar(0, 255, 0),											// draw pure green (remember, its BGR, not RGB)
				CV_FILLED);														// thickness, fill in the circle

			// draw red circle around the detected object
			cv::circle(matOriginal,												// draw on original image
				cv::Point((int)v3fCircles[i][0], (int)v3fCircles[i][1]),		// center point of circle
				(int)v3fCircles[i][2],											// radius of circle in pixels
				cv::Scalar(0, 0, 255),											// draw pure red (remember, its BGR, not RGB)
				3);																// thickness of circle in pixels
		}	// end for

		// declare windows
		cv::namedWindow("Original", CV_WINDOW_AUTOSIZE);	// note: you can use CV_WINDOW_NORMAL which allows resizing the window
		cv::namedWindow("Processed", CV_WINDOW_AUTOSIZE);	// or CV_WINDOW_AUTOSIZE for a fixed size window matching the resolution of the image
		// CV_WINDOW_AUTOSIZE is the default

		cv::imshow("Original", matOriginal);			// show windows
		cv::imshow("Processed", matProcessed);

		charCheckForEscKey = cv::waitKey(1);			// delay (in ms) and get key press, if any
	
}*/
