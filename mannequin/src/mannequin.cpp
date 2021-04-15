
#include "mannequin.h"

MANNEQUIN::MANNEQUIN(std::string prefix){
        _laser_sub = _nh.subscribe("/"+prefix+"/laser/scan",0,&MANNEQUIN::laser_cb,this);
        _odom_sub = _nh.subscribe("/"+prefix+"/odom", 0, &MANNEQUIN::odom_cb, this);
        _cmd_vel_pub = _nh.advertise< geometry_msgs::Twist>("/"+prefix+"/cmd_vel", 0);
        escape_range_       = 30.0 * DEG2RAD;
        check_forward_dist_ = FORWARD_DISTANCE*0.01;
        check_side_dist_    = SIDE_DISTANCE*0.01;

}


void MANNEQUIN::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;
//ROS_INFO("UPDATING VELOCITY L: %.2f A: %.2f",cmd_vel.linear.x,cmd_vel.angular.z);
  _cmd_vel_pub.publish(cmd_vel);
}



void MANNEQUIN::laser_cb( sensor_msgs::LaserScanConstPtr  msg) {
  uint16_t num_values=msg->ranges.size();
  uint16_t center=num_values/2;
  uint16_t scan_angle[3] = {0, center, num_values-1};

  for (int num = 0; num < 3; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      scan_data_[num] = msg->range_max;
    }
    else
    {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}
 
void MANNEQUIN::odom_cb( nav_msgs::OdometryConstPtr odom ) {
    
    tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,  odom->pose.pose.orientation.w);
    double roll, pitch;
    tf::Matrix3x3(q).getRPY(roll, pitch, _yaw); //ottengo angolo di imbardata

    _xp = odom->pose.pose.position.x;
    _yp = odom->pose.pose.position.y;
      
    
}
 
//main loop! 
void MANNEQUIN::wander(){
	ros::Rate r(10);
	uint16_t mannequin_state_num = 0;
while(ros::ok()){

 switch(mannequin_state_num)
  {
    case GET_MANN_DIRECTION:
      if (scan_data_[CENTER] > check_forward_dist_)
      {
        if (scan_data_[LEFT] < check_side_dist_)
        {
          _prev_yaw = _yaw;
          mannequin_state_num = MANN_RIGHT_TURN;
        }
        else if (scan_data_[RIGHT] < check_side_dist_)
        {
          _prev_yaw = _yaw;
          mannequin_state_num = MANN_LEFT_TURN;
        }
        else
        {
          mannequin_state_num = MANN_DRIVE_FORWARD;
        }
      }

      if (scan_data_[CENTER] < check_forward_dist_)
      {
        _prev_yaw = _yaw;
        mannequin_state_num = MANN_RIGHT_TURN;
      }
      break;

    case MANN_DRIVE_FORWARD:
      updatecommandVelocity(LINEAR_VELOCITY, 0.0);
      mannequin_state_num = GET_MANN_DIRECTION;
      break;

    case MANN_RIGHT_TURN:
      if (fabs(_prev_yaw - _yaw) >= escape_range_)
        mannequin_state_num = GET_MANN_DIRECTION;
      else
        updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
      break;

    case MANN_LEFT_TURN:
      if (fabs(_prev_yaw - _yaw) >= escape_range_)
        mannequin_state_num = GET_MANN_DIRECTION;
      else
        updatecommandVelocity(0.0, ANGULAR_VELOCITY);
      break;

    default:
      mannequin_state_num = GET_MANN_DIRECTION;
      break;
  }

r.sleep();
}
}
 
 
void MANNEQUIN::run() {
    boost::thread wander_t(&MANNEQUIN::wander,this);
    ROS_INFO("started");
    ros::spin();
}




int main( int argc, char** argv) {
    std::string pref;
    
    if(argc<2){ pref="mann1";}
    else{ pref=argv[1];}
    std::cout<<"name is:"<<pref<<std::endl;
    ros::init(argc, argv, "MANNEQUIN_task"+pref );
    MANNEQUIN s(pref); //MANNEQUIN(std::string prefix);
    s.run();
    
    return 0;

}   


