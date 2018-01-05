/* Global Parameters initialization */
static const std::string LANE_DETECTION_WINDOW = "Lane Detection";
bool DEBUG = false;
bool DIRECT_CONTROL = false;
int MAX_STEERING_ANGLE_LEFT = 20;
int MAX_STEERING_ANGLE_RIGHT = 160;
int MAX_VEL = -500;
int SERVO_STEP = 2;
int LANE_WIDTH = 110;
int SERVO_CENTER = 90;
bool DRIVE_RIGHT_LANE = true;
float STEERING_SPEED_RATIO = 1.5;

/* Global Constants initialization */
int RIGHT_LINE = -1;
int LEFT_LINE = 1;
int const RIGHT_LANE_ORIGIN = 128;
int const LEFT_LANE_ORIGIN = RIGHT_LANE_ORIGIN - LANE_WIDTH;
float const IMAGE_PERCENTAGE = 0.75;
int const ROW_STEP = 4;
int const SEARCH_RANGE = 10;
int const ALLOWED_DEVIATION = LANE_WIDTH/2;
int const FILTER_KERNEL_SIZE = 5;
int const GRAY_THRESHOLD = 50;
int const MAX_PEAK_HEIGHT = 25;
int const MAX_PEAK_WIDTH = 15;
int const SAFE_MARGIN = 10;
int const SPEED_INCREASE_STEP = 5;
int const SPEED_DECREASE_STEP = 10;
float const MULTIPLY_FACTOR = MAX_VEL / 100.0;
short int const GAP = 10;
short int const MIN_RIGHT_LINE_POINTS = 8;
short int const MIN_LEFT_LINE_POINTS = 5;

 /* Global variables initialization */
std_msgs::Int16 steering_PWM, speed_PWM;
int last_center_position = RIGHT_LANE_ORIGIN;
int center_deviation = 0;
int curvature_degree = SERVO_CENTER;
int last_center_deviation = 0;
int start_time, end_time;
float elapsed_time;


 /*
  * Calculates the servo PWM accordingly to the center_deviation,
  * the curvature degree and the last center deviation using a PD
  * control.
  */
int CalculateServoPWM(
    int curvature_degree,
    int center_deviation, 
    int last_center_deviation) 
{
    int pe;
    int pp;
	int output;  
		  
	if (curvature_degree>110 || curvature_degree<90)
    {
		pe = 1.3;
		pp = 1.1;
    }
	else	
	{
        pe = 0.45;
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
int ServoSaturation(
    int calculated_servo_PWM,
    int current_steering_PWM)
{

    if (calculated_servo_PWM > current_steering_PWM + SERVO_STEP)
	{
		current_steering_PWM += SERVO_STEP;
	}
	else if (calculated_servo_PWM < current_steering_PWM - SERVO_STEP)
	{
		current_steering_PWM -= SERVO_STEP;
	}

    if (current_steering_PWM > MAX_STEERING_ANGLE_RIGHT)
    {
        current_steering_PWM = MAX_STEERING_ANGLE_RIGHT;
    }
    else if (current_steering_PWM < MAX_STEERING_ANGLE_LEFT)
    {
        current_steering_PWM = MAX_STEERING_ANGLE_LEFT;
    }
    

    return current_steering_PWM;
}

/*
  * Calculates the speed PWM value accordingly to the 
  * steering values.
  */
  int CalculateSpeedPWM(
    int current_steering_PWM,
    int current_speed_PWM)
  {
    float tmp;
    int calculated_PWM;

    // Calculate the PWM speed value (lineal function) 
    tmp = 
        abs((SERVO_CENTER - current_steering_PWM) * MULTIPLY_FACTOR);

    calculated_PWM = int(STEERING_SPEED_RATIO * tmp) + MAX_VEL;

    if (calculated_PWM > (current_speed_PWM + SPEED_INCREASE_STEP))
    {
        current_speed_PWM += SPEED_DECREASE_STEP;
    }
    else if (calculated_PWM < (current_speed_PWM + SPEED_INCREASE_STEP))
    {
        current_speed_PWM -= SPEED_INCREASE_STEP;
    }
  
    return current_speed_PWM;
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
    int dist_to_found_point = 0;
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
            int(cv::norm(line_points.back() - found_point));

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

/*
 * Funtion that extracts the left line, 
 * right line and center line points using 
 * local maxima detection
 */
void LineDetection(
    cv::Mat image,
    int image_height,
	int image_width,
    std::vector<cv::Point>& lane_centers,
    std::vector<cv::Point>& right_line_points,
    std::vector<cv::Point>& left_line_points)
{
    int left_index;
	int right_index;
	int current_row;
	int lane_center;
	float dist_to_found_center; 
	bool right_line_point_found;
	bool left_line_point_found; 
    std::vector<int> image_row_vector, local_maxima_found;
    cv::Point center_found;

    // Set initial conditions
    left_index = last_center_position;
    right_index = last_center_position;
	current_row = image_height - GAP; 
	dist_to_found_center = -1.0;
	
	// Set initial line points
    lane_centers.push_back(
		cv::Point2f(last_center_position, image_height - ROW_STEP));	
    left_line_points.push_back(cv::Point2f(
		last_center_position - (LANE_WIDTH/2), image_height - ROW_STEP));
    right_line_points.push_back(cv::Point2f(
		last_center_position + (LANE_WIDTH/2), image_height - ROW_STEP));
	
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
			int cent = right_line_points.back().x - (LANE_WIDTH/2) > SAFE_MARGIN ? 
				right_line_points.back().x - (LANE_WIDTH/2) :
				SAFE_MARGIN;
            lane_centers.push_back(cv::Point2f(cent, current_row));
		 	last_center_position = lane_centers[2].x;
	 	}
		else if (left_line_point_found == true)
		{
			int cent = left_line_points.back().x + (LANE_WIDTH/2) < image_width - SAFE_MARGIN ? 
				left_line_points.back().x + (LANE_WIDTH/2):
				image_width - SAFE_MARGIN;
            lane_centers.push_back(cv::Point2f(cent, current_row));
			last_center_position = lane_centers[2].x;    
         }
         else
            last_center_position = 
                DRIVE_RIGHT_LANE == true? 
                RIGHT_LANE_ORIGIN :
                LEFT_LANE_ORIGIN;

		current_row -= ROW_STEP;	
    }
    
    if(DEBUG)
	{
		// Draw lines begin
		cv::circle(
            image, cv::Point(left_line_points.front()), 
            3, 55, -1);
		cv::circle(
            image, cv::Point(right_line_points.front()), 
            3, 255, -1);
	
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
	}
}