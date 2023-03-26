#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <string>
#include <vector>
#include <cmath>
#include <cstdlib>                    // Needed for rand()
#include <ctime>                      // Needed to seed random number generator with a time value
#include <tf/LinearMath/Quaternion.h> // Needed to convert rotation ...
#include <tf/LinearMath/Matrix3x3.h>  // ... quaternion into Euler angles
#include <tf/transform_listener.h>

using namespace std;

class PotFieldBot
{
public:
    // Construst a new Potential Field controller object and hook up
    // this ROS node to the simulated robot's pose, velocity control,
    // and laser topics
    PotFieldBot(ros::NodeHandle &nh, int robotID, int n, double gx, double gy)
        : ID(robotID), numRobots(n), goalX(gx), goalY(gy)
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
        laserSub = nh.subscribe("base_scan", 1, &PotFieldBot::laserCallback, this);

        poseSub = nh.subscribe("base_pose_ground_truth", 1, &PotFieldBot::poseCallback, this);
    };

    // Process incoming ground truth robot pose message
    void poseCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // cout << "posecallback" << endl;
        double roll, pitch;
        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;

        heading = tf::getYaw(msg->pose.pose.orientation);
    };

    // Send a velocity command
    void move(double linearVelMPS, double angularVelRadPS)
    {
        geometry_msgs::Twist msg; // The default constructor will set all commands to 0
        msg.linear.x = linearVelMPS;
        msg.angular.z = angularVelRadPS;
        commandPub.publish(msg);
    };

    double attractiveForce(double x1, double y1)
    {
        // Calculate the attractive force from the destination (x1,y1)
        double force = (x1 - x) * (x1 - x) + (y1 - y) * (y1 - y);
        return force;
    }

    double attractiveForceX(double x1, double y1)
    {
        // Decompose the attractive force along the X direction
        double a = x1 - x;
        double b = y1 - y;
        double c = sqrt(a * a + b * b);

        // double force = a * a + b * b;
        return c * a;
    }

    double attractiveForceY(double x1, double y1)
    {
        // Decompose the attractive force along the Y direction
        double a = x1 - x;
        double b = y1 - y;
        double c = sqrt(a * a + b * b);

        // double force = a * a + b * b;
        return c * b;
    }

    double attractiveDirection(double x1, double y1)
    {
        double a = x1 - x;
        double b = y1 - y;
        double direction = atan2(b, a);
        return direction - heading;
    }

    double repulsiveForce(double d)
    {
        if (d < (PROXIMITY_RANGE_M + EPSILON))
        {
            return ALPHA / (EPSILON * EPSILON);
        }

        if (d < BETA)
        {
            double z = (d - PROXIMITY_RANGE_M) * (d - PROXIMITY_RANGE_M);
            return ALPHA / z;
        }

        return 0;
    }

    double repulsiveForceX(double d, double angle)
    {
        double force = repulsiveForce(d);
        return -force * cos(angle);
    }

    double repulsiveForceY(double d, double angle)
    {
        double force = repulsiveForce(d);
        return -force * sin(angle);
    }

    bool arrived()
    {
        if (abs(x - goalX) < 0.5 && abs(y - goalY) < 0.5)
        {
            return true;
        }
        return false;
    }

    // Process incoming laser scan message
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        //  TODO: parse laser data
        //  (see http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html)

        // Upon receiving the laserscan data, the robot takes 2 actions:
        // 1. turn to the calculated direction
        // 2. move forward towards that direction

        int indexBound = ceil((msg->angle_max - msg->angle_min) / msg->angle_increment);

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

        double forceX = attractiveForceX(goalX, goalY);
        double forceY = attractiveForceY(goalX, goalY);

        for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++)
        {
            double d = msg->ranges[currIndex];
            double angle = msg->angle_min + currIndex * msg->angle_increment + heading;
            forceX += repulsiveForceX(d, angle);
            forceY += repulsiveForceY(d, angle);
        }

        // cout << "forceX: " << forceX << ", forceY: " << forceY << endl;

        targetDirection = atan2(forceY, forceX);
        // cout << "targetDirection: " << targetDirection << ", heading: " << heading << endl;

        double angleDiff = targetDirection - heading;
        if (angleDiff > M_PI)
        {
            // Rotate counter-clock wise instead
            angleDiff = angleDiff - 2 * M_PI;
        }
        if (angleDiff < -M_PI)
        {
            // Rotate clock wise instead
            angleDiff = 2 * M_PI + angleDiff;
        }

        // Angular velocity
        rotate_speed_radps = KAPPA * angleDiff;
        // cout << "rotate_speed_radps: " << rotate_speed_radps << endl;

        // Linear velocity: there is heading force only if the targetDirection and the heading
        // form an acute angle,
        if (abs(angleDiff) < M_PI / 2)
        {
            forward_speed_mps = 0.5;
        }
    }

    // Main FSM loop for ensuring that ROS messages are
    // processed in a timely manner, and also for sending
    // velocity controls to the simulated robot based on the FSM state
    void
    spin()
    {
        ros::Rate rate(30); // Specify the FSM loop rate in Hz

        while (ros::ok())
        {
            if (!arrived())
            {
                move(forward_speed_mps, rotate_speed_radps);
            }

            ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
            rate.sleep();    // Sleep for the rest of the cycle, to enforce the FSM loop rate
        }
    };

    // Tunable motion controller parameters
    double forward_speed_mps = 0.2;
    double rotate_speed_radps = M_PI / 4;

    constexpr static double MIN_SCAN_ANGLE_RAD = -30.0 / 180 * M_PI;
    constexpr static double MAX_SCAN_ANGLE_RAD = 30.0 / 180 * M_PI;
    constexpr static double PROXIMITY_RANGE_M = 1;

    // Only care about obstacles that is in a range of 4 meters
    constexpr static double BETA = 4.0;
    constexpr static double EPSILON = 0.05;
    constexpr static double GAMMA = 2.0;
    constexpr static double KAPPA = 0.3;
    constexpr static double ALPHA = 1.0;
    constexpr static double ANGLE_EPSILON = 0.1;

protected:
    ros::Publisher commandPub; // Publisher to the current robot's velocity command topic
    ros::Subscriber laserSub;  // Subscriber to the current robot's laser scan topic
    ros::Subscriber poseSub;   // Subscriber to current robots' pose topic
    double x;
    double y;
    double heading;
    double targetDirection;

    int ID;              // 0-indexed robot ID
    int numRobots;       // Number of robots, positive value
    double goalX, goalY; // Coordinates of goal
};

int main(int argc, char **argv)
{
    int robotID = -1, numRobots = 0;
    double goalX, goalY;
    bool printUsage = false;

    // Parse and validate input arguments
    if (argc <= 4)
    {
        printUsage = true;
    }
    else
    {
        try
        {
            robotID = boost::lexical_cast<int>(argv[1]);
            numRobots = boost::lexical_cast<int>(argv[2]);
            goalX = boost::lexical_cast<double>(argv[3]);
            goalY = boost::lexical_cast<double>(argv[4]);

            if (robotID < 0)
            {
                printUsage = true;
            }
            if (numRobots <= 0)
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
        std::cout << "Usage: " << argv[0] << " [ROBOT_NUM_ID] [NUM_ROBOTS] [GOAL_X] [GOAL_Y]" << std::endl;
        return EXIT_FAILURE;
    }

    ros::init(argc, argv, "potfieldbot");                    // Initiate ROS node
    ros::NodeHandle n;                                       // Create named handle "robot_#"
    PotFieldBot robbie(n, robotID, numRobots, goalX, goalY); // Create new random walk object
    robbie.spin();                                           // Execute FSM loop

    return EXIT_SUCCESS;
};
