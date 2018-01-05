#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

 
// Wrap perspective points
int a, b, c, d;
int pixel_cm_ratio_x, pixel_cm_ratio_y;
float scale_x, scale_y;

// Debug global variables
bool CALIBRATION = false;
bool ON_CAR = true;
int LANE_WIDTH = 100;
static const std::string CALIBRATION_WINDOW = "Camera Calibration";

class ImageProssesing
{

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_, image_pub2_; 

public:

  ImageProssesing() 
    : it_(nh_)
  {
    // Subscriber
    if (ON_CAR)
    {
      image_sub_ = it_.subscribe(
        "/app/camera/rgb/image_raw", 1, 
        &ImageProssesing::imageCb, this);
    }
    else 
    {
      image_sub_ = it_.subscribe(
        "/app/camera/rgb/image_raw", 1, 
        &ImageProssesing::imageCb, this, 
        image_transport::TransportHints("compressed"));
    }
    
    // Publisher
    image_pub_ = it_.advertise("/image_processed", 1); 

    if (CALIBRATION)
    {
      cv::namedWindow(CALIBRATION_WINDOW);
    }

  }

  ~ImageProssesing()
  {
    if (CALIBRATION)
    {
      cv::destroyWindow(CALIBRATION_WINDOW);
    }
    
  }


void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
     	cv_ptr = cv_bridge::toCvCopy(
        msg, sensor_msgs::image_encodings::BGR8);
    }
 
    catch (cv_bridge::Exception& e) 
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    int e1 = cv::getTickCount();

    // OpenCV Mat files declaration
    cv::Mat image = cv_ptr-> image,
      croped_image,  
      scaled_image,
      gray_scale_image, 
      lambda_image,
      wrapped_image;

    // 1. Region Of Interest cropping
    cv::Rect ROI(0,90,640,350);
    croped_image = image(ROI);

    // 2. Image Resizing 
    cv::resize(croped_image, scaled_image, 
      cv::Size(), scale_x, scale_y, CV_INTER_LINEAR);
    
    // 3. To gray scale color transform
    cv::cvtColor(scaled_image, gray_scale_image, 
      CV_BGR2GRAY);
      
    // Points used for homology
    cv::Point2f image_points[4], object_points[4];

    // Ger image size
    int image_width = gray_scale_image.size().width;
    int image_height = gray_scale_image.size().height;
     
    object_points[0].x = image_width/2 - a; 
    object_points[0].y = 0;
    object_points[1].x = image_width/2 + b;
    object_points[1].y = 0;
    object_points[2].x = 0;
    object_points[2].y = image_height;
    object_points[3].x = image_width;
    object_points[3].y = image_height;

    image_points[0].x = 0;
    image_points[0].y = 0;
    image_points[1].x = image_width;
    image_points[1].y = 0; 
    image_points[2].x = image_width/2 - c;
    image_points[2].y = image_height;
    image_points[3].x = image_width/2 + d;
    image_points[3].y = image_height;  

    lambda_image = 
      cv::getPerspectiveTransform(object_points, image_points);
 
    // Image wrapping
    cv::warpPerspective(
      gray_scale_image, wrapped_image,
      lambda_image, gray_scale_image.size());
    
    // Image setup to publication
    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;
    out_msg.encoding = sensor_msgs::image_encodings::MONO8;
    out_msg.image= wrapped_image; 
    image_pub_.publish(out_msg.toImageMsg());
    
    if (CALIBRATION)
    {
      
      // Using a 35x35 cm chessboard
      int half_width = image_width/2;
      int half_height = image_height/2;

      // Draw vertical lines
      cv::line(wrapped_image, 
        cv::Point(half_width + LANE_WIDTH/2, 0), 
        cv::Point(half_width + LANE_WIDTH/2, image_height), 
        255);
      cv::line(wrapped_image, 
        cv::Point(half_width - LANE_WIDTH/2, 0), 
        cv::Point(half_width - LANE_WIDTH/2, image_height), 
        255);       

      int i = 0;
      while (i <= (35*pixel_cm_ratio_y)/2)
      {
        cv::line(wrapped_image, 
          cv::Point(0,half_height + i), 
          cv::Point(image_width,half_height+i), 
          255);
        i = i + (5*pixel_cm_ratio_y);
      }
      
      cv::imshow(CALIBRATION_WINDOW, wrapped_image);
      cv::waitKey(3);
    }

    // Node execution time calculation and publication
    int e2 = cv::getTickCount();
    float t = (e2-e1) / cv::getTickFrequency();
    ROS_INFO("  Elapsed time: %f ------- ",t);
     
  }
};

int main(int argc, char** argv) 
{

  // Init ROS node
  ros::init(argc, argv, "ImageProcessing");
  ROS_INFO(" ImageProcessing node running ...");

  // Get parameters from launch
  ros::param::get("~run_on_car", ON_CAR);
  ros::param::get("~calibration_mode", CALIBRATION);
  ros::param::get("~lane_width", LANE_WIDTH);
  ros::param::get("~pixel_cm_ratio_x", pixel_cm_ratio_x);
  ros::param::get("~pixel_cm_ratio_y", pixel_cm_ratio_y);
  ros::param::get("~scale_x", scale_x);
  ros::param::get("~scale_y", scale_y);
  ros::param::get("~p1", a);
  ros::param::get("~p2", b);
  ros::param::get("~p3", c);
  ros::param::get("~p4", d);
 
  // Image callback
  ImageProssesing ip;
  ros::spin();
  return 0;
}
