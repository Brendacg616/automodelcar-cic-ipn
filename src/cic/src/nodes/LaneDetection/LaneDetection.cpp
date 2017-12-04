//Version final 27/06/17 
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>  
#include <sensor_msgs/image_encodings.h>
#include <ros/console.h>
#include <vector>
#include <iostream>
#include "std_msgs/Int16.h"
#include "LocMax_pw.cpp"

cv::Vec4f line;
std_msgs::Int16 angDir,vel;
int antCent=128;
int x,y,cont; 
int MAX_DIST=50;
int MAX_SPEED_BACK=300;
int MAX_SPEED_FOR=-1000;
int MAX_STEER_LEFT=20;
int MAX_STEER_RIGHT=160;
int LANE_WIDTH=4;
int TOL=3;
int err = 0;
int tht = 90;
int errAnt = 0;
int MAX_VEL = -900;



// Global variables
bool DEBUG = false;
static const std::string LANE_DETECTION_WINDOW = "Lane Detection";

bool DIRECT_CONTROL = false;

class LaneDetection
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_; 
	ros::Publisher pubDir,pubVel;

public:

	LaneDetection() 
    : it_(nh_)  
  	{
		image_sub_ = it_.subscribe(
			"/image_processed", 1,
			&LaneDetection::imageCb, this);

		if (DIRECT_CONTROL)
		{
			pubDir= nh_.advertise<std_msgs::Int16>(
				"/manual_control/steering",1); 
   			pubVel= nh_.advertise<std_msgs::Int16>(
				"/manual_control/speed",1);
		}
		else
		{
			pubDir= nh_.advertise<std_msgs::Int16>(
				"/lane_follower_FP/steering",1);
			pubVel= nh_.advertise<std_msgs::Int16>(
				"/lane_follower_FP/speed",1); 
		}

   		if (DEBUG)
   		{
			cv::namedWindow(LANE_DETECTION_WINDOW);
   		}
  	}

	~LaneDetection()
  	{
    	if(DEBUG)
    	{
      		cv::destroyWindow(LANE_DETECTION_WINDOW);
    	}
  	}

	int dirControl(int err,int tht, int errAnt) 
	{
    	int pe;
    	int pp;
    	int sal;    
	  	if (tht>110 || tht<90)
    	{
			pe = 1.25;
			pp = 1;
			MAX_VEL = -1200;
    	}
		else	
		{
            pe = 0.3;
			pp = 1;
			MAX_VEL= -1200;
    	}
		  
		sal = err*pe + tht*pp + (err-errAnt)*1;
		
		return sal;
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
  	{
    	cv_bridge::CvImagePtr cv_ptr;
    try
    	{
     		cv_ptr = cv_bridge::toCvCopy(
				 msg,sensor_msgs::image_encodings::MONO8);
    	}
    catch (cv_bridge::Exception& e) 
    	{
      		ROS_ERROR("cv_bridge exception: %s", e.what());
    		return;
      	}
	
	int e1=cv::getTickCount();

	//Color transform
    cv::Mat image=cv_ptr->image;
    int hght=image.size().height;
	int wdth=image.size().width;
	
	for (int i=hght*0.5; i<hght; i++)
	{
		for (int j=0; j<wdth; j++)
		{
			if (image.at<uchar>(i,j)<50){
				image.at<uchar>(i,j)= rand()%6+87;
			}
		}	
	}

	// Image filtering
	medianBlur(image, image, 5);

	//Lane center detection
    int jI=antCent;
    int jD=antCent;
    int point=antCent;
    int i=hght-9;
    int desv;  
       
    std::vector<cv::Point> cents;
    std::vector<cv::Point> iniIzq;
    std::vector<cv::Point> iniDer;
    std::vector<int> pc,v,loc;
   
    /*cents.push_back(cv::Point2f(antCent,i));
	iniIzq.push_back(cv::Point2f(antCent-MAX_DIST,i));
    iniDer.push_back(cv::Point2f(antCent+MAX_DIST,i)); */
	cents.push_back(cv::Point2f(antCent,hght-3));	
    iniIzq.push_back(cv::Point2f(antCent-MAX_DIST,hght-3));
    iniDer.push_back(cv::Point2f(antCent+MAX_DIST,hght-3));  
	  
	if(DEBUG)
	{
		cv::circle(image,cv::Point(cents.back()),3,155,-1);
		cv::circle(image,cv::Point(iniIzq.back()),3,255,-1);
		cv::circle(image,cv::Point(iniDer.back()),3,55,-1);
	}
     
       
	while (i>hght*0.8)
  	{ 
 		//Lado Derecho
   		bool centDerFlag = false;
		v=(std::vector<int>) image.row(i).colRange(jD,wdth);
		loc=LocMax_pw(v,25,15);
		if (!(loc.empty()))
		{
			x=std::abs(iniDer.back().x-(loc[0]+jD));
			y=std::abs(iniDer.back().y-i);
			
			if (x<(20*TOL) && y<(10*TOL))
                { 
                	iniDer.push_back(cv::Point2f(loc[0]+jD,i));
                    cv::circle(image,cv::Point(iniDer.back()),1,250,-1);
                    point= iniDer.back().x-TOL;
                    centDerFlag = true;
                }
			else 
			point=cents.back().x;
		}
		else
		{
			point=cents.back().x;
		} 
		jD=point;
   
		//Lado izquierdo
   		bool centIzqFlag = false;
		v=(std::vector<int>) image.row(i).colRange(0,jI);
		loc=LocMax_pw(v,25,15);
		if (!(loc.empty()))
		{
			x=std::abs(iniIzq.back().x-(loc.back()));
        	y=std::abs(iniIzq.back().y-i);
			if (x<(20*TOL) && y<(10*TOL))
        	{ 
          		iniIzq.push_back(cv::Point2f(loc.back(),i));
            	cv::circle(image,cv::Point(iniIzq.back()),1,50,-1);
            	point= iniIzq.back().x+(3*TOL);
            	centIzqFlag = true;
        	}
			else 
			point=cents.back().x;
		}
		else 
		{
			point=cents.back().x;
   		}
		jI=point;


		if (centDerFlag == true)
		{
		//	if ((iniDer.back().x-MAX_DIST>0)
			int cent= iniDer.back().x-MAX_DIST>50? iniDer.back().x-MAX_DIST:MAX_DIST;
	   		cents.push_back(cv::Point2f(cent,i));
			antCent= cents[1].x;
		}
		else if (centIzqFlag == true)
		{
		//	int cent= iniIzq.back().x+MAX_DIST<wdth? iniIzq.back().x+MAX_DIST:wdth;
			int cent= iniIzq.back().x+MAX_DIST<wdth-MAX_DIST? iniIzq.back().x+MAX_DIST:wdth-MAX_DIST;
	   		cents.push_back(cv::Point2f(cent,i));
	   		antCent= cents[1].x;    
		}
		cv::circle(image,cv::Point(cents.back()),1,155,-1);		
		i=i-4;	
	}

	//Steering calculation
	bool rf=false;
	bool DerFlag=false;
	bool IzqFlag=false;
	if (iniDer.size()>8)
	{
	     cv::fitLine(iniDer,line,CV_DIST_L2,0,0.01,0.01);
	     desv =-MAX_DIST;
	     rf=true;    
	     DerFlag=true;    
	}		
	else if (iniIzq.size()>5)
	{
	     cv::fitLine(iniIzq,line,CV_DIST_L2,0,0.01,0.01);;
	     desv = MAX_DIST;
	     rf=true; 
	     IzqFlag=true;
	}		

	if (rf==true)
	{
	    err = wdth/2-cents[1].x;	
		int p1=line[2];
		int p2=line[3];
	    int p3=(line[2]+1)+100*line[0];
	    int p4=(line[3]+1)+100*line[1];
	    if (IzqFlag==true)
	    {
			p3=iniIzq.back().x;
		 	p4=iniIzq.back().y; 
	    }
	    else if (DerFlag==true)
	    {
			p3=iniDer.back().x;
			p4=iniDer.back().y; 
	    }

	    float m1=p4-p2;
	    float m2=p3-p1;
	    float m=(m2/m1); 
		int thtCalc = 90 + int(57*atan(m));

		tht = thtCalc; 
		
		if(DEBUG)
		{
			cv::circle(image,cv::Point(p1,p2),6,200,-1);
			cv::circle(image,cv::Point(p3,p4),6,200,-1);
			cv::line(image,cv::Point(p1,p2),cv::Point(p3,p4),100,3);                                
			ROS_INFO(" tht= %i",tht);
		}
		  
	}
	
	errAnt = err;
	int calcDir = dirControl(err, tht, errAnt); 
	ROS_INFO(" Calcdir= %i ",calcDir); 
	if (calcDir>angDir.data+3)
	{
		angDir.data += 3;
	}
	else if (calcDir<angDir.data-3)
	{
		angDir.data -= 3;
	}

	//Speed Calculation and publication
	//int maxVel = int(MAX_VEL*exp(-1*(0.002)*std::abs(err*10)));
	float nouvtht=(tht/90)-1;
	if (nouvtht<0)
	{
  		nouvtht=-nouvtht;
	}
	int maxVel=int(MAX_VEL*exp(-0.5*nouvtht));
	ROS_INFO(" MaxVel= %i ",maxVel); 
	if (vel.data > (maxVel+9))
   	{vel.data -= 2;}
	else if (vel.data<=maxVel)
   	{vel.data += 7;}
	
	//Servomotor and saturation////////////////////////////////////////////////
	if (angDir.data>MAX_STEER_RIGHT)
	{
    	angDir.data = MAX_STEER_RIGHT;
	}
	else if (angDir.data<MAX_STEER_LEFT)
	{
    	angDir.data = MAX_STEER_LEFT;
	}

	pubDir.publish(angDir);
	pubVel.publish(vel);

	if(DEBUG)
	{
		cv::imshow(LANE_DETECTION_WINDOW, image);
		cv::waitKey(3);
	}

	int e2=cv::getTickCount();
	float t=(e2-e1)/cv::getTickFrequency();
	ROS_INFO("frame time: %f-------------------------------block end",t);        

	}
};

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "LaneDetection");
	ROS_INFO("LaneDetection node running...");

	// Get parameters from launch
	ros::param::get("~debug_mode", DEBUG);
	ros::param::get("~direct_mode", DIRECT_CONTROL);

	LaneDetection ld;
	ros::spin();
	return 0;
}
