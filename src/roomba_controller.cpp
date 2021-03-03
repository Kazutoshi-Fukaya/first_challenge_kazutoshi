#include "roomba_controller/roomba_controller.h"

RoombaController::RoombaController():private_nh("~")
{
    private_nh.param("hz",hz_,{10});

    sub_pose = nh.subscribe("/roomba/odometry",10,&RoombaController::odometry_callback,this);

    pub_cmd_vel = nh.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control",1);
}

void RoombaController::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    past_pose = current_pose;
    current_pose = msg->pose.pose;
    tf::Quaternion current_quat(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z);
    tf::Quaternion past_quat(past_pose.orientation.x, past_pose.orientation.y, past_pose.orientation.z);
    tf::Matrix3x3(current_quat).getRPY(current_r, current_p, current_y);
    tf::Matrix3x3(past_quat).getRPY(past_r, past_p, past_y);
}

void RoombaController::go_straight()
{
//    std::cout<<current_pose<<std::endl;
    roomba_500driver_meiji::RoombaCtrl cmd_vel;
    cmd_vel.cntl.linear.x = 0.2;
    cmd_vel.mode = 11;
    pub_cmd_vel.publish(cmd_vel);
}

void RoombaController::turn()
{
//    std::cout<<current_pose<<std::endl;
    roomba_500driver_meiji::RoombaCtrl cmd_vel;
    cmd_vel.cntl.angular.z = 0.1;
    cmd_vel.mode = 11;
    pub_cmd_vel.publish(cmd_vel);
}

void RoombaController::process()
{
    ros::Rate loop_rate(hz_);

    ros::Duration(0.1).sleep();

//    geometry_msgs::Pose past_pose = current_pose;
    int straight = 0;
    double dist_x = 0.0;
    double dist_y = 0.0;
    double delta_y = 0.0;
    double sum_y = 0.0;

    while(ros::ok())
    {
        dist_x = current_pose.position.x-past_pose.position.x;
        dist_y = current_pose.position.y-past_pose.position.y;

        if (current_y*past_y < 0.0) delta_y = current_y-past_y+2*M_PI;
        else delta_y = current_y-past_y;

        sum_y += delta_y;

        std::cout<<current_y<<" "<<past_y<<std::endl;

        if ((dist_x)*(dist_x)+(dist_y)*(dist_y) > 1.0) straight = 0;
        if (sum_y >= M_PI)
        {
            straight = 1;
            sum_y = 0.0;
        }


        if (straight == 1) go_straight();
        else turn();

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv,"roomba_controller");
    RoombaController roomba_controller;
    roomba_controller.process();
    return 0;
}
