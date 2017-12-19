/* Global parameters initialization */
static const std::string CROSSING_DETECTION_WINDOW = "Crossing Lines Detection";
bool DEBUG = false;

/* Global constants initialization */
short int const NO_LINE_DIST = -1;
short int const FILTER_KERNEL_SIZE = 5;
short int const COLUMN_STEP = 2;
float const IMAGE_PERCENTAGE_START = 0.4;
float const IMAGE_PERCENTAGE_END = 0.7;
short int const MAX_PEAK_HEIGHT = 25;
short int const MAX_PEAK_WIDTH = 25;
short int const SEARCH_RANGE = 15;
short int const LINE_POINTS_REQUIRED = 15;
short int const MIN_LINE_POINTS = 3;
short int const MAX_DIST_ALLOWED = 7;

/* Global variables initialization */
short int dist_to_line = NO_LINE_DIST; 
float line_angle = 0.0;
int start_time, end_time;
float elapsed_time;