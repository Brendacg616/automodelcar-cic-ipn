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
#include "cic/Lane.h"
#include "LocMax_pw.cpp"

// Global Parameters
static const std::string LANE_DETECTION_WINDOW = "Lane Detection";
bool DEBUG;
bool DIRECT_CONTROL = false;
float IMAGE_PERCENTAGE = 0.8;
int MAX_STEERING_ANGLE_LEFT = 20;
int MAX_STEERING_ANGLE_RIGHT = 160;
int MAX_VEL = -500;
int LANE_WIDTH = 110;
int ROW_STEP = 4;
int TOLERANCE = 3;
bool DRIVE_RIGHT_LANE = true;

// Global Constants
int RIGHT_LANE_ORIGIN = 128;
int LEFT_LANE_ORIGIN = RIGHT_LANE_ORIGIN - LANE_WIDTH;

//Global variables
cv::Vec4f line;
std_msgs::Int16 steering_PWM,vel;
int last_center_position = RIGHT_LANE_ORIGIN;
int x, y, cont; 
int err = 0;
int tht = 90;
int errAnt = 0;
int e1, e2;
float elapsed_time;

class LaneDetection
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_; 
	ros::Publisher pubMsg, pubDir, pubVel;

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
		pubMsg = nh_.advertise<cic::Lane>(
			"/lane_detection",1);
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

int CalculateServoPWM(int err,int tht, int errAnt) 
{
    int pe;
    int pp;
	int output;  
		  
	if (tht>110 || tht<90)
    {
		pe = 1.25;
		pp = 1;
    }
	else	
	{
        pe = 0.3;
		pp = 1;
    }
		  
	output = err*pe + tht*pp + (err-errAnt)*1;
	return output;
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
	
	e1 = cv::getTickCount();

	// --- Color transform (fill black corners) --- //
    cv::Mat image = cv_ptr->image;
    int hght = image.size().height;
	int wdth = image.size().width;
	
	for (int i = hght * IMAGE_PERCENTAGE; i<hght; i++)
	{
		for (int j = 0; j < wdth; j++)
		{
			if (image.at<uchar>(i,j) < 50)
			{
				image.at<uchar>(i,j) = rand()%6 + 87;
			}
		}	
	}

	// --- Image filtering --- //
	medianBlur(image, image, 5);

	// --- Lane detection algorithm --- //
	// Initial conditions
    int jI = last_center_position;
    int jD = last_center_position;
    int point=last_center_position;
    int current_row = hght-9;
    int desv;  
       
    std::vector<cv::Point> cents;
    std::vector<cv::Point> iniIzq;
    std::vector<cv::Point> iniDer;
    std::vector<int> v,loc;
   
    /*cents.push_back(cv::Point2f(last_center_position,i));
	iniIzq.push_back(cv::Point2f(last_center_position-(LANE_WIDTH/2),i));
    iniDer.push_back(cv::Point2f(last_center_position+(LANE_WIDTH/2),i)); */
	cents.push_back(cv::Point2f(last_center_position,hght-3));	
    iniIzq.push_back(cv::Point2f(last_center_position-(LANE_WIDTH/2),hght-3));
    iniDer.push_back(cv::Point2f(last_center_position+(LANE_WIDTH/2),hght-3));
	  
	if(DEBUG)
	{
		cv::circle(image,cv::Point(iniIzq.back()),3,255,-1);
		cv::circle(image,cv::Point(iniDer.back()),3,55,-1);
	}
     
       
	while (current_row > (hght * IMAGE_PERCENTAGE))
  	{ 
 		//Lado Derecho
   		bool centDerFlag = false;
		v=(std::vector<int>) image.row(current_row).colRange(jD,wdth);
		loc=LocMax_pw(v,25,15);
		if (!(loc.empty()))
		{
			x = std::abs(iniDer.back().x-(loc[0]+jD));
			y = std::abs(iniDer.back().y - current_row);
			
			if (x<(20*TOLERANCE) && y<(10*TOLERANCE))
                { 
                	iniDer.push_back(
						cv::Point2f(loc[0]+jD, current_row));
                    cv::circle(
						image,
						cv::Point(iniDer.back()),1,250,-1);
                    point= iniDer.back().x-TOLERANCE;
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
		v=(std::vector<int>) image.row(current_row).colRange(0,jI);
		loc=LocMax_pw(v,25,15);
		if (!(loc.empty()))
		{
			x=std::abs(iniIzq.back().x-(loc.back()));
        	y=std::abs(iniIzq.back().y - current_row);
			if (x<(20*TOLERANCE) && y<(10*TOLERANCE))
        	{ 
          		iniIzq.push_back(
					  cv::Point2f(loc.back(), current_row));
            	cv::circle(
					image,cv::Point(iniIzq.back()),1,50,-1);
            	point= iniIzq.back().x+(3*TOLERANCE);
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
		//	if ((iniDer.back().x-(LANE_WIDTH/2)>0)
			int cent= iniDer.back().x - (LANE_WIDTH/2)>50? iniDer.back().x - (LANE_WIDTH/2):(LANE_WIDTH/2);
	   		cents.push_back(cv::Point2f(cent, current_row));
			last_center_position= cents[1].x;
		}
		else if (centIzqFlag == true)
		{
		//	int cent= iniIzq.back().x+(LANE_WIDTH/2)<wdth? iniIzq.back().x+(LANE_WIDTH/2):wdth;
			int cent= iniIzq.back().x+(LANE_WIDTH/2)<wdth-(LANE_WIDTH/2)? iniIzq.back().x+(LANE_WIDTH/2):wdth-(LANE_WIDTH/2);
	   		cents.push_back(cv::Point2f(cent, current_row));
	   		last_center_position = cents[1].x;    
		}
		cv::circle(image,cv::Point(cents.back()),1,155,-1);	

		current_row -= ROW_STEP;	
	}

	// --- Steering calculation --- //
	bool rf=false;
	bool DerFlag=false;
	bool IzqFlag=false;
	if (iniDer.size() > 8)
	{
	     cv::fitLine(iniDer,line,CV_DIST_L2,0,0.01,0.01);
	     desv =-(LANE_WIDTH/2);
	     rf=true;    
	     DerFlag=true;    
	}		
	else if (iniIzq.size() > 5)
	{
	     cv::fitLine(iniIzq,line,CV_DIST_L2,0,0.01,0.01);;
	     desv = (LANE_WIDTH/2);
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
	int servo_PWM = CalculateServoPWM(err, tht, errAnt); 
	 
	if (servo_PWM > steering_PWM.data + 3)
	{
		steering_PWM.data += 3;
	}
	else if (servo_PWM < steering_PWM.data - 3)
	{
		steering_PWM.data -= 3;
	}

	// --- Speed Calculation --- //
	//int maxVel = int(MAX_VEL*exp(-1*(0.002)*std::abs(err*10)));
	float nouvtht=(tht/90)-1;
	if (nouvtht<0)
	{
  		nouvtht=-nouvtht;
	}

	int maxVel=int(MAX_VEL*exp(-0.5*nouvtht));
	
	if (vel.data > (maxVel+9))
   	{vel.data -= 2;}
	else if (vel.data<=maxVel)
   	{vel.data += 7;}
	
	//Servomotor and saturation
	if (steering_PWM.data > MAX_STEERING_ANGLE_RIGHT)
	{
    	steering_PWM.data = MAX_STEERING_ANGLE_RIGHT;
	}
	else if (steering_PWM.data < MAX_STEERING_ANGLE_LEFT)
	{
    	steering_PWM.data = MAX_STEERING_ANGLE_LEFT;
	}

	if (DIRECT_CONTROL)
	{
		pubDir.publish(steering_PWM);
		pubVel.publish(vel);
	}
	else
	{
		cic::Lane LaneMsg;
		LaneMsg.header.stamp = ros::Time::now();
		LaneMsg.steering_value = steering_PWM.data;
		LaneMsg.speed_value = vel.data;
		pubMsg.publish(LaneMsg);
	}

	e2 = cv::getTickCount();
	elapsed_time = (e2 - e1)/cv::getTickFrequency();

	if(DEBUG)
	{
		ROS_INFO(" servo_PWM= %i ",servo_PWM);
		ROS_INFO("frame time: %f ----- block end", elapsed_time);
		cv::imshow(LANE_DETECTION_WINDOW, image);
		cv::waitKey(3); 
	}     

}
};

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "LaneDetection");
	ROS_INFO("LaneDetection node running...");

	// Get parameters from launch
	ros::param::get("~debug_mode", DEBUG);
	ros::param::get("~direct_mode", DIRECT_CONTROL);
	ros::param::get("~max_vel", MAX_VEL);
	ros::param::get("~lane_width", LANE_WIDTH);
	ros::param::get("~drive_right_lane", DRIVE_RIGHT_LANE);
	ros::param::get("~image_percentage", IMAGE_PERCENTAGE);
	ros::param::get("~row_step", ROW_STEP);
	ros::param::get("~max_steering_angle_right", 
		MAX_STEERING_ANGLE_RIGHT);
	ros::param::get("~max_steering_angle_left", 
		MAX_STEERING_ANGLE_LEFT);

	last_center_position = DRIVE_RIGHT_LANE == true? 
		RIGHT_LANE_ORIGIN : LEFT_LANE_ORIGIN;

	LaneDetection ld;
	ros::spin();
	return 0;
}
