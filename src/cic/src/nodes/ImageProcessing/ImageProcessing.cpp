#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>  
#include <sensor_msgs/image_encodings.h>
#include <ros/console.h>


class PreProsc
{
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_,image_pub2_; 

public:
    PreProsc() 
    : it_(nh_)
  {
     image_sub_ = it_.subscribe("/app/camera/rgb/image_raw", 1,&PreProsc::imageCb, this);
     image_pub_ = it_.advertise("/img_prepros", 1); 
     image_pub2_= it_.advertise("/calib_cam",1);
  }
  


void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    
    	cv_bridge::CvImagePtr cv_ptr;
        sensor_msgs::ImagePtr sal_ptr; 
    try
    	{
     	cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    	}
 
    catch (cv_bridge::Exception& e) 
    	{
      	ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
    	}

       int e1=cv::getTickCount();
       cv::Mat image=cv_ptr->image,rs,gray_image,crop,Lambda,BV;
       cv::Point2f imgPts[4],objPts[4] ;
       cv::Rect ROI(0,90,640,350);
       crop=image(ROI);

       cv::resize(crop, rs, cv::Size(), 0.4, 1.2,CV_INTER_LINEAR);
       cv::cvtColor(rs, gray_image,CV_BGR2GRAY);
       
       int wh=gray_image.size().width;
       int ht=gray_image.size().height;
     
       
       objPts[0].x=wh/2-45; 
       objPts[0].y=0;
       objPts[1].x=wh/2+50;
       objPts[1].y=0;
       objPts[2].x=0;
       objPts[2].y=ht;
       objPts[3].x=wh;
       objPts[3].y=ht;

       imgPts[0].x=0;
       imgPts[0].y=0;
       imgPts[1].x=wh;
       imgPts[1].y=0; 
       imgPts[2].x=wh/2-45;
       imgPts[2].y=ht;
       imgPts[3].x=wh/2+45;
       imgPts[3].y=ht;  

       Lambda=cv::getPerspectiveTransform(objPts,imgPts);
       cv::warpPerspective(gray_image,BV,Lambda,gray_image.size());
      
       cv_bridge::CvImage out_msg;
       out_msg.header = msg->header;
       out_msg.encoding= sensor_msgs::image_encodings::MONO8;
       out_msg.image= BV; 
       image_pub_.publish(out_msg.toImageMsg());
       cv::Mat calib;
       calib=BV;
       cv::line(BV,cv::Point(128,0),cv::Point(128,420),255);
       cv::line(BV,cv::Point(78,75),cv::Point(178,75),255);
       cv::line(BV,cv::Point(78,200),cv::Point(178,200),255);
       cv::line(BV,cv::Point(78,350),cv::Point(178,350),255);
       out_msg.image=calib;
       
       image_pub2_.publish(out_msg.toImageMsg());
       

       int e2=cv::getTickCount();
       float t=(e2-e1)/cv::getTickFrequency();
       ROS_INFO("frame time: %f-------------------------------block end",t);   
  }
};

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "ImageProcessing");
  ROS_INFO("ImageProcessing node running");
  PreProsc ic;
  ros::spin();
  return 0;
}
