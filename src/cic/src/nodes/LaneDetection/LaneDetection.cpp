//Version  13/12/17 
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
#include <LocalMaximaDetection.cpp>
#include "LaneDetection.h"
#include <math.h>

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
			"/manual_control/steering", 1); 
   		pubVel= nh_.advertise<std_msgs::Int16>(
			"/manual_control/speed", 1);
	}
	else
	{
		pubMsg = nh_.advertise<cic::Lane>(
			"/lane_detection", 1);
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

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	
    try
    {
     	cv_ptr = cv_bridge::toCvCopy(
			msg, sensor_msgs::image_encodings::MONO8);
    }
   	catch (cv_bridge::Exception& e) 
    {
      	ROS_ERROR("cv_bridge exception: %s", e.what());
    	return;
    }
	
	start_time = cv::getTickCount();

	cv::Mat image;
	int image_height;
	int image_width;
	std::vector<cv::Point> lane_centers;
	std::vector<cv::Point> left_line_points;
	std::vector<cv::Point> right_line_points;
	bool line_found;
	bool right_line_found;
	bool left_line_found;
	cv::Vec4f line;

	// Image allocator and shape extraction
	image = cv_ptr -> image;
	image_height = image.size().height;
	image_width = image.size().width;

	// Color transform used to fill black corners 	
	for (int i = image_height * IMAGE_PERCENTAGE; i < image_height; i++)
	{
		for (int j = 0; j < image_width; j++)
		{
			if (image.at<uchar>(i,j) < GRAY_THRESHOLD)
			{
				image.at<uchar>(i,j) = rand()%6 + 87;
			}
		}	
	}

	// Image filtering 
	medianBlur(image, image, FILTER_KERNEL_SIZE);

	// Line detection algorithm
	LineDetection(
		image,
		image_height,
		image_width,
		lane_centers,
		right_line_points,
		left_line_points);

	// Curvature degree calculation
	line_found = false;
	right_line_found = false;
	left_line_found = false;
	if (right_line_points.size() > MIN_RIGHT_LINE_POINTS)
	{
	     cv::fitLine(
			 right_line_points, line,
			 CV_DIST_L2, 0, 0.01 ,0.01);
	     line_found = true;    
	     right_line_found = true;    
	}		
	else if (left_line_points.size() > MIN_LEFT_LINE_POINTS)
	{
	     cv::fitLine(
			 left_line_points, line,
			 CV_DIST_L2, 0, 0.01, 0.01);
	     line_found = true; 
	     left_line_found = true;
	}		

	if (line_found == true)
	{
	    center_deviation = int(image_width/2) - lane_centers[1].x;	
		int p1 = line[2];
		int p2 = line[3];
	    int p3 = (line[2]+1)+100*line[0];
		int p4 = (line[3]+1)+100*line[1];
		
	    if (left_line_found == true)
	    {
			p3=left_line_points.back().x;
		 	p4=left_line_points.back().y; 
	    }
	    else if (right_line_found == true)
	    {
			p3=right_line_points.back().x;
			p4=right_line_points.back().y; 
	    }

	    float m1 = p4 - p2;
	    float m2 = p3 - p1;
	    float m = (m2 / m1); 
		 
		curvature_degree = SERVO_CENTER + int(atan(m)*(180.0/M_PI)); 
		
		if(DEBUG)
		{		
			// Draw cord line
			cv::line(image, cv::Point(p1,p2), cv::Point(p3,p4), 100, 3);
			cv::circle(image, cv::Point(p1, p2), 3, 200, -1);
			cv::circle(image, cv::Point(p3, p4), 3, 200, -1); 
		}

		// Servo PWM calculation
		steering_PWM.data = ServoSaturation(
			CalculateServoPWM(
				curvature_degree,
				center_deviation,  
				last_center_deviation),
			steering_PWM.data);

		// Speed PWM calculation
		speed_PWM.data = 
			CalculateSpeedPWM(
				steering_PWM.data,
				speed_PWM.data);		  
	}
	else{
		if (speed_PWM.data < 0)
			speed_PWM.data += SPEED_DECREASE_STEP;
	}

	// Messages publication
	if (DIRECT_CONTROL)
	{
		pubDir.publish(steering_PWM);
		pubVel.publish(speed_PWM);
	}
	else
	{	
		cic::Lane LaneMsg;
		LaneMsg.header.stamp = ros::Time::now();
		LaneMsg.steering_value = steering_PWM.data;
		LaneMsg.speed_value = speed_PWM.data;
		pubMsg.publish(LaneMsg);
	}

	// Set last values
	last_center_deviation = center_deviation;

	// Get elapsed time
	end_time = cv::getTickCount();
	elapsed_time = (end_time - start_time)/cv::getTickFrequency();

	if(DEBUG)
	{		
		// Print debug info 
		ROS_INFO(" devistion from center = %i", center_deviation);
		ROS_INFO(" curvature_degree = %i", curvature_degree);
		ROS_INFO(" steering_PWM = %i", steering_PWM.data);
		ROS_INFO(" speed_PWM =  %i", speed_PWM.data);
		ROS_INFO(" frame time: %.4f ----- block end", elapsed_time);
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
	ros::param::get("~steering_speed_ratio", STEERING_SPEED_RATIO);
	ros::param::get("~lane_width", LANE_WIDTH);
	ros::param::get("~servo_center_position", SERVO_CENTER);
	ros::param::get("~servo_step", SERVO_STEP);
	ros::param::get("~drive_right_lane", DRIVE_RIGHT_LANE);
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
