// Global Parameters initialization
static const std::string LANE_DETECTION_WINDOW = "Lane Detection";
bool DEBUG = false;
bool DIRECT_CONTROL = false;
float IMAGE_PERCENTAGE = 0.8;
int MAX_STEERING_ANGLE_LEFT = 20;
int MAX_STEERING_ANGLE_RIGHT = 160;
int MAX_VEL = -500;
int LANE_WIDTH = 110;
bool DRIVE_RIGHT_LANE = true;

/* Global Constants initialization */
int const RIGHT_LANE_ORIGIN = 128;
int const LEFT_LANE_ORIGIN = RIGHT_LANE_ORIGIN - LANE_WIDTH;
int ROW_STEP = 4;
int SEARCH_RANGE = 10;
int  ALLOWED_DEVIATION = LANE_WIDTH/2;

/* Global variables initialization */
int image_height;
int image_width;
cv::Vec4f line;
std_msgs::Int16 steering_PWM, speed_PWM;
int servo_PWM = 90;
cv::Point found_point;
int last_center_position = RIGHT_LANE_ORIGIN;
int center_deviation = 0;
int curvature_degree = 90;
int last_center_deviation = 0;
int e1, e2;
float elapsed_time;

// imageCb variables decalration
float dist_to_found_point;
int left_index;
int right_index;
int row_index;
int current_row; 
bool right_line_point_found = false;
bool left_line_point_found = false; 
std::vector<cv::Point> lane_centers;
std::vector<cv::Point> left_line_points;
std::vector<cv::Point> right_line_points;
std::vector<int> image_row_vector, local_maxima_found;

 /*
  * Calculates the servo PWM accordingly to the center_deviation,
  * the curvature degree and the last center deviation using a PD
  * control.
  */
int CalculateServoPWM(
    int center_deviation,
    int curvature_degree, 
    int last_center_deviation) 
{
    int pe;
    int pp;
	int output;  
		  
	if (curvature_degree>110 || curvature_degree<90)
    {
		pe = 1.25;
		pp = 1;
    }
	else	
	{
        pe = 0.3;
		pp = 1;
    }
		  
    output = 
        center_deviation * pe + 
        curvature_degree * pp + 
        (center_deviation - last_center_deviation) * 1;
    
    return output;
}

 /*
  * Saturates the servo PWM value accordingly to the 
  * servo limits.
  */
int ServoSaturation(int servo_PWM)
{
    int PWM_value;

    
    if (servo_PWM > steering_PWM.data + 3)
	{
		servo_PWM += 3;
	}
	else if (servo_PWM < steering_PWM.data - 3)
	{
		servo_PWM -= 3;
	}

    if (servo_PWM > MAX_STEERING_ANGLE_RIGHT)
    	PWM_value = MAX_STEERING_ANGLE_RIGHT;
	else if (servo_PWM < MAX_STEERING_ANGLE_LEFT)
    	PWM_value = MAX_STEERING_ANGLE_LEFT;
    else
        PWM_value = servo_PWM;

    return PWM_value;
}