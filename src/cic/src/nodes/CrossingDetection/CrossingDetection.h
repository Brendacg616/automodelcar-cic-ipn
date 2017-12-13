/* Global parameters initialization */
static const std::string CROSSING_DETECTION_WINDOW = "Crossing Lines Detection";
bool DEBUG = false;

/* Global constants initialization */
short int const NO_LINE_DIST = -1;

/* Global variables initialization */
short int dist_to_line = NO_LINE_DIST; 
float line_angle = 0.0;
int start_time, end_time;
float elapsed_time;