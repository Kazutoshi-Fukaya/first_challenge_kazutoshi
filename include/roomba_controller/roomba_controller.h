#ifndef ROOMBA_CONTROLLER_H
#define ROOMBA_CONTROLLER_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"

class RoombaController
{
    public:
        RoombaController();
        void process();

    private:
        void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);
        void go_straight();
        void turn();

        int hz_;

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        ros::Subscriber sub_pose;
        ros::Publisher pub_cmd_vel;
        geometry_msgs::Pose current_pose;
        geometry_msgs::Pose past_pose;
        double current_r;
        double current_p;
        double current_y;
        double past_r;
        double past_p;
        double past_y;
};
#endif
