#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

/**
 * Stair detection ros ndoe which utilizes OpenCV to classify stairways in a height map 
 * provided in the form of a grid map
 * 
 */

// If true the different filter steps will be visualized and parameters can be adjusted via sliders
bool debug = true;

// Parameters for the Canny filter
int lowThreshold = 10;
int highThreshold = 129;
const int max_lowThreshold = 100;
int canny_kernel_size = 1;

// The different OpenCV images used
cv::Mat src_scaled, dst, zoomedImage, cdst, processed, erosion_dst, dilation_dst, stairs_dst;

// The region of interest in the grid map
cv::Rect roi;

// Parameters for the Probabilistic Hough Line Transform
int lineThreshold = 13;
int minLineLength = 9;
int maxLineGap = 4;
int rho = 1;
int theta_scale = 100;

// Parameters for the Laplacian filter
int scale = 1;
int delta = 0;
int laplacian_kernel_size = 0;

// Radius of the Gaussian blur
int blurRadius = 37;


// Titles of the debug windows
const char *src_window_name = "Original Image";
const char *process_window_name = "Processed Image";
const char *canny_window_name = "Canny";
const char *laplacian_window_name = "Laplacian";
const char *line_window_name = "Hough Line Transform";
const char *extracted_line_window_name = "Extracted Lines";


void updateCanny(int, void *);
void updateLaplacian(int, void *);
void updatePreprocess(int, void *);
static void updateLineDetection(int, void *);
void gridMapCallback(grid_map_msgs::GridMap msg);


/**
 * Callback method for the grid map. Starts the filtering Process.
 * 
 */
void gridMapCallback(grid_map_msgs::GridMap msg)
{
    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(msg, map);

    if (debug)
    {
        grid_map::Matrix &elevation = map.get("elevation");

        float min = 9999.0;
        float max = -9999.0;

        // Calculating the minimum and maximum elevation in the height map
        // to calibrate the grid map to opencv conversion
        for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
        {
            const int i = iterator.getLinearIndex();
            if (elevation(i) < min)
                min = elevation(i);
            if (elevation(i) > max)
                max = elevation(i);
        }
    }
    cv::Mat src;

    // Converting the grid map to an opencv image
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, "elevation", CV_8UC1, -0.075, 1.995, src);

    double resizeFactor = 3.0;

    cv::resize(src, src_scaled, cv::Size(), resizeFactor, resizeFactor);
    
    // Specifying the region of interest in the image
    roi.width = src_scaled.size().width / 11;
    roi.height = src_scaled.size().height / 11;
   
    roi.x = 500;
    roi.y = 3100;
    updatePreprocess(0, 0);
}

/**
 * Applies the Gaussian blur to reduce noise.
 */
void updatePreprocess(int, void *)
{
    blurRadius = blurRadius - (blurRadius % 2) + 1;
    cv::GaussianBlur(src_scaled, processed, cv::Size(blurRadius, blurRadius), 0.0, 0.0);

    if (debug)
    {
        updateLaplacian(0, 0);
        updateCanny(0, 0);
    }
}

/**
 * Applies the Laplacian to create alternative edge image. Not used at the moment.
 */
void updateLaplacian(int, void *)
{
    int ddepth = CV_8U;
    Laplacian(processed, dst, ddepth, 3 + laplacian_kernel_size * 2, scale, delta, cv::BORDER_DEFAULT);
}

/**
 * Applies the Canny filter to create edge image.
 */ 
void updateCanny(int, void *)
{
    Canny(processed, dst, lowThreshold, highThreshold, 3 + canny_kernel_size * 2);
    if (debug)
    {
        imshow(canny_window_name, dst(roi));
        updateLineDetection(0, 0);
    }
}

/**
 * Detect lines using the Probabilistic Hough Line Transform
 */ 
static void updateLineDetection(int, void *)
{
    cv::Mat cdstP;
    cvtColor(dst, cdst, cv::COLOR_GRAY2BGR);
    cdstP = cdst.clone();

    // Probabilistic Hough Line Transform
    std::vector<cv::Vec4i> linesP;                                                                            // will hold the results of the detection
    HoughLinesP(dst, linesP, rho, theta_scale / 100 * CV_PI / 180, lineThreshold, minLineLength, maxLineGap); // runs the actual detection

    // Draw the lines
    stairs_dst = cv::Mat(src_scaled.size(), src_scaled.type(), cv::Scalar(255, 255, 255));
    /*for (size_t i = 0; i < linesP.size(); i++)
    {
        cv::Vec4i l = linesP[i];
        line(stairs_dst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 0), 0.5, cv::LINE_AA);
        line(cdstP, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 0.5, cv::LINE_AA);
    }
	
	for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
		cout << "The value at index " << (*iterator).transpose() << " is " << map.at("layer", *iterator) << endl;
	}
	for (grid_map::LineIterator iterator(map_, start, end); !iterator.isPastEnd(); ++iterator) {
		map_.at("type", *iterator) = 1.0;
		publish();
		ros::Duration duration(0.02);
		duration.sleep();
	}*/
	
	int sample_dist = 2;
	grid_map::Matrix& data = map_["type"]; // locally store a reference to the map data for improved performance. 
	for (size_t i = 0; i < linesP.size(); i++)
    {
        cv::Vec4i l = linesP[i];
		cv::Point pointA = cv::Point(l[0], l[1]);
		cv::Point pointB = cv::Point(l[2], l[3]);
        line(stairs_dst, pointA, pointB, cv::Scalar(0, 0, 0), 0.5, cv::LINE_AA);
        line(cdstP, pointA, pointB, cv::Scalar(0, 0, 255), 0.5, cv::LINE_AA);
		
		Index start(l[0], l[1]);
		Index end(l[2], l[3]);
		cv::Point direction = pointA-pointB;
		cv::Point direction_perpendicular = cv::Point(-direction.y, direction.x);;
		direction_perpendicular /= norm(direction_perpendicular); // normalize
		direction_perpendicular *= sample_dist; // scale to sample_dist
		for (grid_map::LineIterator iterator(data, start, end); !iterator.isPastEnd(); ++iterator) {
            
            float sample1 = map.at("elevation", *iterator - direction_perpendicular);
            float sample2 = map.at("elevation", *iterator + direction_perpendicular)
			map.at("elevation", *iterator) //// add/subtract direction from *iterator (index type), then round
			//// subtract height at new points.
		}

        /*
        OR USE PYTHON Iterator:
        pt_a = np.array([10, 11]) # start punkt
        pt_b = np.array([45, 67]) # end punkt
        im = np.zeros((80, 80, 3), np.uint8) # daten
        for p in np.linspace(pt_a, pt_b, np.linalg.norm(pt_a-pt_b)):
            # do stuff with p
            # access grid map like this:
            # https://docs.ros.org/en/kinetic/api/grid_map_msgs/html/msg/GridMap.html
        */


    }
	

}



/**
 * Main method. Gets called when the node is started
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "edge_height_detection");
    ros::NodeHandle ns;
    ros::NodeHandle np;

    ns.param("debug", debug, debug);

    ros::Subscriber zed_pose_subscriber = ns.subscribe("/elevation_mapping/elevation_map_raw", 1, gridMapCallback);

    ros::Rate loop_rate(5);
    ros::spinOnce();
    
    // Main loop
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Fin suscriptor");
    return 0;
}
