#include "roomba_controller/roomba_controller.h"

RoombaController::RoombaController():private_nh("~")
{
    private_nh.param("hz",hz_,{10});

    sub_pose = nh.subscribe("/roomba/odometry",10,&RoombaController::odometry_callback,this);
    sub_laserscan = nh.subscribe("/scan",10,&RoombaController::laserscan_callback,this);

    pub_cmd_vel = nh.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control",1);
}

void RoombaController::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    past_pose = current_pose;
    current_pose = msg->pose.pose;
    tf::Quaternion current_quat(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
    tf::Quaternion past_quat(past_pose.orientation.x, past_pose.orientation.y, past_pose.orientation.z, past_pose.orientation.w);
    tf::Matrix3x3(current_quat).getRPY(current_r, current_p, current_y);
    tf::Matrix3x3(past_quat).getRPY(past_r, past_p, past_y);

    sum_x += current_pose.position.x-past_pose.position.x;
    if(current_y-past_y>-M_PI) sum_yaw += current_y-past_y;
}

void RoombaController::laserscan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    current_ranges = msg->ranges;
    wall_dist = current_ranges[current_ranges.size()/2];
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

void RoombaController::stop()
{
    roomba_500driver_meiji::RoombaCtrl cmd_vel;
    cmd_vel.mode = 11;
    pub_cmd_vel.publish(cmd_vel);
}

void RoombaController::process()
{
    ros::Rate loop_rate(hz_);

    ros::Duration(0.1).sleep();

//    geometry_msgs::Pose past_pose = current_pose;
    int stage = 0;
//    double delta_x = 0.0;
//    double sum_x = 0.0;
//    double delta_y = 0.0;
//    double delta_dist = 0.0;

//    double current_r, current_p, current_y;
//    double past_r, past_p, past_y;

//    double delta_yaw = 0.0;
//    double sum_yaw = 0.0;

    while(ros::ok())
    {


//        delta_x = current_pose.position.x-past_pose.position.x;
//        delta_y = current_pose.position.y-past_pose.position.y;
//        delta_dist = (dist_x)*(dist_x)+(dist_y)*(dist_y);

//        tf::Quaternion current_quat(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
//        tf::Quaternion past_quat(past_pose.orientation.x, past_pose.orientation.y, past_pose.orientation.z, past_pose.orientation.w);
//        tf::Matrix3x3(current_quat).getRPY(current_r, current_p, current_y);
//        tf::Matrix3x3(past_quat).getRPY(past_r, past_p, past_y);

//        if (current_y*past_y < 0.0) delta_yaw = 0.1;
//        else delta_yaw = current_y-past_y;

//        sum_yaw += delta_yaw;
//        sum_x += delta_x;

//        std::cout<<"current_y="<<current_y<<"  past_y="<<past_y<<std::endl;
//        std::cout<<"delta_x="<<delta_x<<"  delta_yaw="<<delta_yaw<<std::endl;
        std::cout<<"sum_x="<<sum_x<<"  sum_yaw="<<sum_yaw<<std::endl;
        std::cout<<"wall_dist="<<wall_dist<<std::endl;

        if (stage==0 && sum_x>1.0) {
            stage = 1;
            sum_yaw = 0.0;
            sum_x = 0.0;
        }
        if (stage==1 && sum_yaw>=2*M_PI)
        {
            stage = 2;
            sum_yaw = 0.0;
            sum_x = 0.0;
        }
        if (stage==2 && wall_dist<0.5) {
            stage = 3;
            sum_yaw;
            sum_x;
        }


        if (stage == 0) go_straight();
        if (stage == 1) turn();
        if (stage == 2) go_straight();
        if (stage == 3) stop();

//        past_pose = current_pose;

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
