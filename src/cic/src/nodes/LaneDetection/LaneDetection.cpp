//Version  7/12/17 
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
#include "LaneDetection.h"

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

	cv::Mat image;
	int image_height;
	int image_width;
	int left_index;
	int right_index;
	int current_row;
	int lane_center;
	float dist_to_found_center; 
	bool right_line_point_found;
	bool left_line_point_found; 
	int servo_PWM;
	cv::Vec4f line;
	cv::Point center_point_found;
	std::vector<cv::Point> lane_centers;
	std::vector<cv::Point> left_line_points;
	std::vector<cv::Point> right_line_points;
	std::vector<int> image_row_vector, local_maxima_found;

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

	/* ---  Image filtering --- */
	medianBlur(image, image, FILTER_KERNEL_SIZE);

	/* --- Lane detection algorithm --- */

	// Set initial conditions
    left_index = last_center_position;
    right_index = last_center_position;
	current_row = image_height - 9; 
	dist_to_found_center = -1.0;
    lane_centers.clear();
    left_line_points.clear();
	right_line_points.clear();
	
	// Set initial line points
    lane_centers.push_back(
		cv::Point2f(last_center_position, image_height - 1));	
    left_line_points.push_back(cv::Point2f(
		last_center_position - (LANE_WIDTH/2), image_height - 1));
    right_line_points.push_back(cv::Point2f(
		last_center_position + (LANE_WIDTH/2), image_height - 1));
	
	// Image scanning by rows
	while (current_row > (image_height * IMAGE_PERCENTAGE))
  	{

		/* Right side scanning */
		right_line_point_found = false;
		// Prepare image row vector to scan
		image_row_vector = 
			(std::vector<int>) image.row(current_row).colRange(right_index, image_width);
		// Search for local maxima
		local_maxima_found = 
			LocMax_pw(image_row_vector, MAX_PEAK_HEIGHT, MAX_PEAK_WIDTH);
		// Local maxima validation (if found)
		LocalMaximaValidation(
			RIGHT_LINE,
			local_maxima_found,
			lane_centers,
			current_row,
			right_line_points,
			right_index,
			right_line_point_found);

		/* Left side scanning */ 
		left_line_point_found = false;
		// Prepare image row vector to scan
		image_row_vector = 
			(std::vector<int>) image.row(current_row).colRange(0, left_index);
		// Search for local maxima
		local_maxima_found = 
			LocMax_pw(image_row_vector, MAX_PEAK_HEIGHT, MAX_PEAK_WIDTH);
		// Local maxima validation (if found)
		LocalMaximaValidation(
			LEFT_LINE,
			local_maxima_found,
			lane_centers,
			current_row,
			left_line_points,
			left_index,
			left_line_point_found);

		// Center points calculation
		if (right_line_point_found == true)
		{			
			int cent = right_line_points.back().x - (ALLOWED_DEVIATION)>50? 
				right_line_points.back().x - ALLOWED_DEVIATION:
				ALLOWED_DEVIATION;
			lane_centers.push_back(cv::Point2f(cent, current_row));
		 	last_center_position = lane_centers[1].x;
	 	}
		else if (left_line_point_found == true)
		{
			int cent = left_line_points.back().x + ALLOWED_DEVIATION<image_width-ALLOWED_DEVIATION? 
				left_line_points.back().x + ALLOWED_DEVIATION:
				image_width-ALLOWED_DEVIATION;
			lane_centers.push_back(cv::Point2f(cent, current_row));
			last_center_position = lane_centers[1].x;    
	 	}

		current_row -= ROW_STEP;	
	}

	/* --------------- Steering calculation --------------- */
	bool rf = false;
	bool DerFlag = false;
	bool IzqFlag = false;
	if (right_line_points.size() > 8)
	{
	     cv::fitLine(right_line_points, line, CV_DIST_L2,0,0.01,0.01);
	     rf=true;    
	     DerFlag=true;    
	}		
	else if (left_line_points.size() > 5)
	{
	     cv::fitLine(left_line_points, line, CV_DIST_L2,0,0.01,0.01);;
	     rf=true; 
	     IzqFlag=true;
	}		

	if (rf==true)
	{
	    center_deviation = image_width/2 - lane_centers[1].x;	
		int p1=line[2];
		int p2=line[3];
	    int p3=(line[2]+1)+100*line[0];
	    int p4=(line[3]+1)+100*line[1];
	    if (IzqFlag==true)
	    {
			p3=left_line_points.back().x;
		 	p4=left_line_points.back().y; 
	    }
	    else if (DerFlag==true)
	    {
			p3=right_line_points.back().x;
			p4=right_line_points.back().y; 
	    }

	    float m1 = p4 - p2;
	    float m2 = p3 - p1;
	    float m = (m2 / m1); 
		 
		curvature_degree = 90 + int(57*atan(m)); 
		
		if(DEBUG)
		{
			// Draw lines begin
			cv::circle(image,cv::Point(left_line_points.front()), 3, 55, -1);
			cv::circle(image,cv::Point(right_line_points.front()), 3, 255, -1);
		
			// Draw lane centers
			for (std::vector<cv::Point>::iterator point = 
				lane_centers.begin() ; 
				point != lane_centers.end(); ++point)
				cv::circle(image, *point, 1, 155, -1);
	
			// Draw right line points
			for (std::vector<cv::Point>::iterator point = 
				right_line_points.begin() ; 
				point != right_line_points.end(); ++point)
				cv::circle(image, *point, 1, 250, -1);

			// Draw left line points
			for (std::vector<cv::Point>::iterator point = 
				left_line_points.begin() ; 
				point != left_line_points.end(); ++point)
				cv::circle(image, *point, 1, 50, -1);
		
			// Draw cord line
			cv::line(image, cv::Point(p1,p2), cv::Point(p3,p4), 100, 3);
			cv::circle(image, cv::Point(p1, p2), 3, 200, -1);
			cv::circle(image, cv::Point(p3, p4), 3, 200, -1); 
		}
		  
	}
	
	last_center_deviation = center_deviation;

	// Servo PWM calculation
	servo_PWM = CalculateServoPWM(
		center_deviation, curvature_degree, last_center_deviation); 
	
	//Servo PWM saturation
	steering_PWM.data = ServoSaturation(servo_PWM);

	/* --------------- Speed PWM Calculation --------------- */
	float nouvtht=(curvature_degree/90)-1;
	if (nouvtht<0)
	{
  		nouvtht=-nouvtht;
	}

	int maxVel=int(MAX_VEL*exp(-0.5*nouvtht));
	
	if (speed_PWM.data > (maxVel+9))
   	{speed_PWM.data -= 2;}
	else if (speed_PWM.data<=maxVel)
   	{speed_PWM.data += 7;}


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

	e2 = cv::getTickCount();
	elapsed_time = (e2 - e1)/cv::getTickFrequency();

	if(DEBUG)
	{		
		// Print debug info 
		ROS_INFO(" curvature_degree = %i", curvature_degree);
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
	ros::param::get("~servo_center_position", SERVO_CENTER);
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
