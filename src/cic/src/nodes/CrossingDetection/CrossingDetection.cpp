//Version  13/12/17
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/console.h>
#include <math.h>
#include <numeric>
#include "cic/Intersection.h"
#include "std_msgs/Int16.h"
#include "LocMax_pw.cpp"
#include "CrossingDetection.h"

//#include <opencv2/video/tracking.hpp>

class CrossingDetection
{
  	ros::NodeHandle nh_;
  	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
  	image_transport::Publisher image_pub_;
	ros::Publisher pubMsg;

public:

CrossingDetection()
   	: it_(nh_)
{
	image_sub_ = it_.subscribe(
		"/image_processed", 1,
		&CrossingDetection::imageCb, this);

	pubMsg = nh_.advertise<cic::Intersection>(
		"/crossing_detection", 1);
	
	if (DEBUG)
	{
 		cv::namedWindow(CROSSING_DETECTION_WINDOW);
	}
}

~CrossingDetection()
{
	if(DEBUG)
	{
		  cv::destroyWindow(CROSSING_DETECTION_WINDOW);
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

	cv::Mat image, transposed_image;
	int image_height;
	int image_width;   
	
	int i,j,a_point,p_point;
	std::vector<int> v,loc,pc;
	std::vector<cv::Point>  ini;
	
	// Image allocator and shape extraction
	image = cv_ptr -> image;
	transposed_image = image.t();
	image_height = image.size().height;
	image_width = image.size().width;
	
	// Image filtering
	medianBlur(image, image, 5);

	i=image_height-2;
	j=image_width*0.4;
	p_point=image_height-1;

	while (j<(image_width*0.7))
	{
		v=(std::vector<int>) transposed_image.row(j).colRange(image_height*0.5,image_height);
		loc=LocMax_pw(v,25,25);
		if (!(loc.empty()))
		{
			a_point=int (loc.back()+image_height*0.5);
			if (abs(p_point-a_point)<7)
			{
				ini.push_back(cv::Point2f(j,a_point));
				cv::circle(image,cv::Point(j,a_point),1,250,-1);
			}

		p_point = a_point;
		}
		j+=2;
		
	}
	if (ini.size()>15){
		cv::Vec4f p;
		cv::fitLine(ini, p,CV_DIST_L1,0,0.01,0.01);
		float ang = (57*atan(p[1]/p[0]));
		
		
		//ROS_INFO("dist: %i	|| ang: %i",dist,ang);
		if ((ang>-15.0)&&(ang<15.0))
		{
			line_angle = ang;
			dist_to_line = (image_height-p[3])/2;
			cv::line(image,cv::Point(p[2],p[3]),cv::Point(p[2]+p[0]*50,p[3]+p[1]*50),0);
			cv::circle(image,cv::Point(p[2],p[3]),1,0,-1);
		}
		else
		{
			dist_to_line = NO_LINE_DIST;
		}
	}
	else 
	{
		dist_to_line = NO_LINE_DIST;
	}

	// Message publication
	cic::Intersection CrossingMsg;
	CrossingMsg.header.stamp = ros::Time::now();
	CrossingMsg.angle = line_angle;
	CrossingMsg.distance = dist_to_line;
	pubMsg.publish(CrossingMsg);

	// Get elapsed time
	end_time = cv::getTickCount();
	elapsed_time = (end_time - start_time) / cv::getTickFrequency();

	if (DEBUG)
	{
		// Print debug info 
		ROS_INFO(" frame time: %.4f ----- block end", elapsed_time);
		cv::imshow(CROSSING_DETECTION_WINDOW, image);
		cv::waitKey(3); 
	}
	

}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "CrossingDetection");
	ROS_INFO("CrossingDetection node running...");

	// Get parameters from launch
	ros::param::get("~debug_mode", DEBUG);
	
	CrossingDetection cd;
  	ros::spin();
  	return 0;
}
