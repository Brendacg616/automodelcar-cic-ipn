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
	ros::Publisher pub_ang;
	ros::Publisher pub_dist;

public:

CrossingDetection()
   	: it_(nh_)
{
	image_sub_ = it_.subscribe(
		"/image_processed", 1,
		&CrossingDetection::imageCb, this);

	pub_ang= nh_.advertise<std_msgs::Int16>(
		"inter_line/ang",1);
	pub_dist= nh_.advertise<std_msgs::Int16>(
		"inter_line/dist",1);
	
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
	sensor_msgs::ImagePtr sal_ptr;
		
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

	int e1=cv::getTickCount();
   	cv::Mat image=cv_ptr->image;
	medianBlur(image, image,5);
	cv::Mat imaget=image.t();
	int wdth, hght,i,j,a_point,p_point,dist;
	std::vector<int> v,loc,pc;
	std::vector<cv::Point>  ini;
	
	wdth=image.cols;
	hght=image.rows;
	i=hght-2;
	j=wdth*0.4;
	p_point=hght-1;

	while (j<(wdth*0.7))
	{
		v=(std::vector<int>) imaget.row(j).colRange(hght*0.5,hght);
		loc=LocMax_pw(v,25,25);
		if (!(loc.empty()))
		{
			a_point=int (loc.back()+hght*0.5);
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
		int ang=(57*atan(p[1]/p[0]));
		int dist=(hght-p[3])/2;
		std_msgs::Int16 p_ang;
		std_msgs::Int16 p_dist;	
		p_ang.data=ang;
		p_dist.data=dist;
		ROS_INFO("dist: %i	|| ang: %i",dist,ang);
		if ((ang>-15)&&(ang<15))
		{
			pub_ang.publish(p_ang);
			pub_dist.publish(p_dist);
			cv::line(image,cv::Point(p[2],p[3]),cv::Point(p[2]+p[0]*50,p[3]+p[1]*50),0);
			cv::circle(image,cv::Point(p[2],p[3]),1,0,-1);
		}
	}
	else 
	{
		dist=200;
	}

	int e2=cv::getTickCount();
	float t=(e2-e1)/cv::getTickFrequency();

	if (DEBUG)
	{
		
		ROS_INFO(" frame time: %.4f ----- block end", t);
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
