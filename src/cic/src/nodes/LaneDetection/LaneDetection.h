// Global Parameters initialization
static const std::string LANE_DETECTION_WINDOW = "Lane Detection";
bool DEBUG = false;
bool DIRECT_CONTROL = false;
int MAX_STEERING_ANGLE_LEFT = 20;
int MAX_STEERING_ANGLE_RIGHT = 160;
int MAX_VEL = -500;
int LANE_WIDTH = 110;
int SERVO_CENTER = 90;
bool DRIVE_RIGHT_LANE = true;

/* Global Constants initialization */
int RIGHT_LINE = -1;
int LEFT_LINE = 1;
int const RIGHT_LANE_ORIGIN = 128;
int const LEFT_LANE_ORIGIN = RIGHT_LANE_ORIGIN - LANE_WIDTH;
float const IMAGE_PERCENTAGE = 0.7;
int const ROW_STEP = 4;
int const SEARCH_RANGE = 10;
int const ALLOWED_DEVIATION = LANE_WIDTH/2;
int const FILTER_KERNEL_SIZE = 5;
int const GRAY_THRESHOLD = 50;
int const MAX_PEAK_HEIGHT = 25;
int const MAX_PEAK_WIDTH = 15;

/* Global variables initialization */
std_msgs::Int16 steering_PWM, speed_PWM;
int last_center_position = RIGHT_LANE_ORIGIN;
int center_deviation = 0;
int curvature_degree = SERVO_CENTER;
int last_center_deviation = 0;
int e1, e2;
float elapsed_time;

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

 /*
  * Validates the local maxima found through 
  * euclidian distance to last line point
  */
 void LocalMaximaValidation(
    int line,
    std::vector<int> local_maxima_found,
    std::vector<cv::Point> lane_centers,
    int current_row,
    std::vector<cv::Point>& line_points,
    int& index,
    bool& point_found_flag)
  {
    int dist_to_found_point = 0.0;
    int peak_location;
    cv::Point found_point;

    if (!(local_maxima_found.empty()))
    {
        // Get peak location accordingly to the line
        peak_location = 
            line == RIGHT_LINE ?
            local_maxima_found.front() + index:
            local_maxima_found.back();

        // Set the first local maxima found as found point
        found_point = 
            cv::Point(peak_location, current_row);

        // Euclidian distance from last right line point to found point
        dist_to_found_point = 
            cv::norm(line_points.back() - found_point);

        // Point found validation through distance
        if (dist_to_found_point < ALLOWED_DEVIATION)
            { 
                // Add found point to line
                line_points.push_back(found_point);
                // Adjust row index accordingly to the found point and 
                // a search range value.
                index = line_points.back().x + line * SEARCH_RANGE;
                point_found_flag = true;
            }
        else 
            // No right line point found
            index = lane_centers.back().x;
    }
    else
    {
        // No local maxima found
        index = lane_centers.back().x;
    } 
  }