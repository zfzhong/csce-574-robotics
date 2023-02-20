#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> // Needed for rand()
#include <ctime>   // Needed to seed random number generator with a time value
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>

using namespace boost::posix_time;

class GridMapper
{
public:
    // Construst a new occupancy grid mapper  object and hook up
    // this ROS node to the simulated robot's pose, velocity control,
    // and laser topics
    GridMapper(ros::NodeHandle &nh, int width, int height) : fsm(FSM_MOVE_FORWARD),
                                                             rotateStartTime(ros::Time::now()),
                                                             rotateDuration(0.f),
                                                             canvas(height, width, CV_8UC1)
    {
        // Initialize random time generator
        srand(time(NULL));

        // Advertise a new publisher for the current simulated robot's
        // velocity command topic (the second argument indicates that
        // if multiple command messages are in the queue to be sent,
        // only the last command will be sent)
        commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        // Subscribe to the current simulated robot's laser scan topic and
        // tell ROS to call this->laserCallback() whenever a new message
        // is published on that topic
        laserSub = nh.subscribe("base_scan", 1,
                                &GridMapper::laserCallback, this);

        // Subscribe to the current simulated robot' ground truth pose topic
        // and tell ROS to call this->poseCallback(...) whenever a new
        // message is published on that topic
        poseSub = nh.subscribe("base_pose_ground_truth", 1,
                               &GridMapper::poseCallback, this);

        // Create resizeable named window
        cv::namedWindow("Occupancy Grid Canvas",
                        cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_EXPANDED);
    };

    // Save a snapshot of the occupancy grid canvas
    // NOTE: image is saved to same folder where code was executed
    void saveSnapshot()
    {
        std::string filename = "grid_" + to_iso_string(second_clock::local_time()) + ".jpg";
        canvasMutex.lock();
        cv::imwrite(filename, canvas);
        canvasMutex.unlock();
    };

    // Update grayscale intensity on canvas pixel (x, y) (in robot coordinate frame)
    void plot(int x, int y, char value)
    {
        canvasMutex.lock();
        x += canvas.rows / 2;
        y += canvas.cols / 2;
        if (x >= 0 && x < canvas.rows && y >= 0 && y < canvas.cols)
        {
            if (canvas.at<char>(x, y) == CELL_UNKNOWN)
            {
                canvas.at<char>(x, y) = value;
            }
        }
        canvasMutex.unlock();
    };

    void plotObstacle(float dist, float angle, char value)
    {
        float x1 = x - dist * sin(angle);
        float y1 = y + dist * cos(angle);
        plot(x1, y1, value);
    }

    void plotLine(float dist, float angle, char value)
    {
        // std::cout << "dist: " << dist << ", angle: " << angle << std::endl;
        float x0 = x + canvas.rows / 2;
        float y0 = y + canvas.cols / 2;

        float x1 = x0 - dist * sin(angle);
        float y1 = y0 + dist * cos(angle);
        // std::cout << "(" << x0 << ", " << y0 << "), (" << x1 << ", " << y1 << ")" << std::endl;

        // a rough implementation for see the effect
        int currX = x0;
        int currY = y0;
        int prevX = currX;
        int prevY = currY;

        // What if x1-x0 is too small?
        // Todo: We need to deal with this case specifically.
        if (abs(x1 - x0) < 0.1)
        {
            while (abs(currY - x0) < abs(y1 - y0))
            {
                plotImg(currX, currY, value);
                currY = y1 > y0 ? currY + 1 : currY - 1;
            }
            return;
        }

        float slope = (y1 - y0) / (x1 - x0);

        plotImg(currX, currY, value);

        while (abs(currX - x0) < abs(x1 - x0))
        {
            currX = x1 > x0 ? currX + 1 : currX - 1;
            currY = y0 + slope * (currX - x0);

            while (prevY != (int)(currY))
            {
                prevY = y1 > y0 ? prevY + 1 : prevY - 1;
                plotImg(prevX, prevY, value);
            }

            plotImg(currX, currY, value);

            prevX = currX;
            prevY = currY;
        }
    }

    // Update grayscale intensity on canvas pixel (x, y) (in image coordinate frame)
    void plotImg(int x, int y, char value)
    {
        canvasMutex.lock();
        if (x >= 0 && x < canvas.rows && y >= 0 && y < canvas.cols)
        {
            if (canvas.at<char>(x, y) == CELL_UNKNOWN)
            {
                canvas.at<char>(x, y) = value;
            }
        }
        canvasMutex.unlock();
    };

    // Send a velocity command
    void move(double linearVelMPS, double angularVelRadPS)
    {
        geometry_msgs::Twist msg; // The default constructor will set all commands to 0
        msg.linear.x = linearVelMPS;
        msg.angular.z = angularVelRadPS;
        commandPub.publish(msg);
    };

    // Process incoming laser scan message
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        // TODO: parse laser data and update occupancy grid canvas
        //       (use CELL_OCCUPIED, CELL_UNKNOWN, CELL_FREE, and CELL_ROBOT values)
        // (see http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html)

        int indexBound = ceil((msg->angle_max - msg->angle_min) / msg->angle_increment);

        if (fsm == FSM_MOVE_FORWARD)
        {
            unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
            unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);

            // Make sure index wont be off bounds.
            if (minIndex < 0)
            {
                minIndex = 0;
            }
            if (maxIndex > indexBound)
            {
                maxIndex = indexBound;
            }

            float closestRange = msg->ranges[minIndex];
            float closestIndex = 0;

            for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++)
            {
                if (msg->ranges[currIndex] < closestRange)
                {
                    closestRange = msg->ranges[currIndex];
                    closestIndex = currIndex;
                }
            }

            float dist = PROXIMITY_RANGE_M;
            if (closestRange < dist)
            {
                dist = closestRange;
            }

            for (int i = minIndex + 1; i < maxIndex; ++i)
            {
                float angle = msg->angle_min + i * msg->angle_increment + heading;
                plotLine(dist, angle, CELL_FREE);
            }

            if (closestRange < PROXIMITY_RANGE_M)
            {
                // The robot switches to ROTATE mode.
                fsm = FSM_ROTATE;
                // std::cout << "set to ROTATE" << std::endl;

                // Now we got an obstacle in front of us, the robot needs to rotate.
                rotateStartTime = ros::Time::now();
                rotateDuration = ros::Duration(rand() % 5 + 1);

                float angle = msg->angle_min + closestIndex * msg->angle_increment + heading;
                plotObstacle(dist, angle, CELL_OCCUPIED);
            }

            // ros::Time now = ros::Time::now();
            // std::cout << now << ", closest: " << closestRange << std::endl;
        }
    };

    // Process incoming ground truth robot pose message
    void poseCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        double roll, pitch;
        x = -msg->pose.pose.position.y;
        y = msg->pose.pose.position.x;

        // 1 unit in stageros equals 5 units in map
        x = x * 5;
        y = y * 5;

        heading = tf::getYaw(msg->pose.pose.orientation);

        plot(x, y, CELL_ROBOT);
    };

    // Main FSM loop for ensuring that ROS messages are
    // processed in a timely manner, and also for sending
    // velocity controls to the simulated robot based on the FSM state
    void spin()
    {
        int key = 0;

        // Initialize all pixel values in canvas to CELL_UNKNOWN
        canvasMutex.lock();
        canvas = cv::Scalar(CELL_UNKNOWN);
        canvasMutex.unlock();

        ros::Rate rate(10); // Specify the FSM loop rate in Hz

        while (ros::ok())
        { // Keep spinning loop until user presses Ctrl+C
            // TODO: remove following demo code and make robot move around the environment
            /*
            plot(x, y, rand() % 255); // Demo code: plot robot's current position on canvas
            plot(x + 1, y + 1, rand() % 255);

            plotImg(0, 0, CELL_OCCUPIED); // Demo code: plot different colors at 4 canvas corners
            plotImg(1, 1, CELL_OCCUPIED);

            plotImg(0, canvas.rows - 1, CELL_UNKNOWN);

            plotImg(canvas.cols - 1, 0, CELL_FREE);
            plotImg(canvas.cols - 1, 1, CELL_FREE);

            plotImg(canvas.cols - 1, canvas.rows - 1, CELL_ROBOT);
            plotImg(canvas.cols - 1, canvas.rows - 2, CELL_ROBOT);
*/

            //  std::cout << x << ", " << y << std::endl;

            if (fsm == FSM_MOVE_FORWARD)
            {
                move(FORWARD_SPEED_MPS, 0);
            }

            if (fsm == FSM_ROTATE)
            {
                // std::cout << "send ROTATE command" << std::endl;
                move(0, ROTATE_SPEED_RADPS);

                // Be careful that when the robot is rotating, it should check surroundings
                // to make sure that it won't bump into obstacles while rotating. When it
                // is moving ahead, it only checks with the obstacles ahead, without considering
                // obstacles on the left/right hand side.

                // Check the duration of rotation. If the duration exceeds the given
                // amount of time, the robot switches back to MOVE_FORWARD mode.
                ros::Time now = ros::Time::now();

                if (now - rotateStartTime >= rotateDuration)
                {
                    // Stop rotating
                    fsm = FSM_MOVE_FORWARD;
                }
            }

            // NOTE: DO NOT REMOVE CODE BELOW THIS LINE
            cv::imshow("Occupancy Grid Canvas", canvas);
            ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages

            key = cv::waitKey(1000 / SPIN_RATE_HZ); // Obtain keypress from user; wait at most N milliseconds
            if (key == 'x' || key == 'X')
            {
                break;
            }
            else if (key == ' ')
            {
                saveSnapshot();
            }

            rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
        }

        ros::shutdown(); // Ensure that this ROS node shuts down properly
    };

    enum FSM
    {
        FSM_MOVE_FORWARD,
        FSM_ROTATE
    };

    constexpr static double MIN_SCAN_ANGLE_RAD = -30.0 / 180 * M_PI;
    constexpr static double MAX_SCAN_ANGLE_RAD = +30.0 / 180 * M_PI;

    // Should be smaller than  sensor_msgs::LaserScan::range_max
    constexpr static float PROXIMITY_RANGE_M = 0.6;

    // safety distance is 0.5 meter, given the robot itself is 0.5 meters wide
    constexpr static float SAFETY_RANGE_M = 0.5;

    constexpr static double FORWARD_SPEED_MPS = 0.5;
    constexpr static double ROTATE_SPEED_RADPS = M_PI / 2;

    const static int SPIN_RATE_HZ = 30;

    const static char CELL_OCCUPIED = 0;
    const static char CELL_UNKNOWN = 86;
    const static char CELL_FREE = 172;
    const static char CELL_ROBOT = 255;

protected:
    ros::Publisher commandPub; // Publisher to the current robot's velocity command topic
    ros::Subscriber laserSub;  // Subscriber to the current robot's laser scan topic
    ros::Subscriber poseSub;   // Subscriber to the current robot's ground truth pose topic

    double x;       // in simulated Stage units, + = East/right
    double y;       // in simulated Stage units, + = North/up
    double heading; // in radians, 0 = East (+x dir.), pi/2 = North (+y dir.)

    enum FSM fsm;
    ros::Time rotateStartTime;    // Start time of the rotation
    ros::Duration rotateDuration; // Duration of the rotation

    cv::Mat canvas;           // Occupancy grid canvas
    boost::mutex canvasMutex; // Mutex for occupancy grid canvas object
};

int main(int argc, char **argv)
{
    int width, height;
    bool printUsage = false;

    // Parse and validate input arguments
    if (argc <= 2)
    {
        printUsage = true;
    }
    else
    {
        try
        {
            width = boost::lexical_cast<int>(argv[1]);
            height = boost::lexical_cast<int>(argv[2]);

            if (width <= 0)
            {
                printUsage = true;
            }
            else if (height <= 0)
            {
                printUsage = true;
            }
        }
        catch (std::exception err)
        {
            printUsage = true;
        }
    }
    if (printUsage)
    {
        std::cout << "Usage: " << argv[0] << " [CANVAS_WIDTH] [CANVAS_HEIGHT]" << std::endl;
        return EXIT_FAILURE;
    }

    ros::init(argc, argv, "grid_mapper"); // Initiate ROS node
    ros::NodeHandle n;                    // Create default handle
    GridMapper robbie(n, width, height);  // Create new grid mapper object
    robbie.spin();                        // Execute FSM loop

    return EXIT_SUCCESS;
};
