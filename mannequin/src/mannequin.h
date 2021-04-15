#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>
#include "boost/thread.hpp"


#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 1
#define LEFT   2
#define RIGHT  0

#define LINEAR_VELOCITY  0.3
#define ANGULAR_VELOCITY 1.5

#define GET_MANN_DIRECTION 0
#define MANN_DRIVE_FORWARD 1
#define MANN_RIGHT_TURN    2
#define MANN_LEFT_TURN     3
#define ESCAPE_RANGE       45
#define FORWARD_DISTANCE   90 //IN cm
#define SIDE_DISTANCE      40 //IN cm

class MANNEQUIN {
    public:
        MANNEQUIN(std::string prefix);
        void laser_cb( sensor_msgs::LaserScanConstPtr );
        void odom_cb( nav_msgs::OdometryConstPtr );
        void updatecommandVelocity(double, double);
        void run();
        void wander();
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;
        ros::Subscriber _laser_sub;
        ros::Publisher _cmd_vel_pub;
        double scan_data_[3];
        double _yaw;
        double _prev_yaw;
        float _xp;
        float _yp;
	double escape_range_;
	double check_forward_dist_;
	double check_side_dist_;
};
