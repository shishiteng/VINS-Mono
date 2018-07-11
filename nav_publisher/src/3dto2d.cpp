#include <stdio.h>

#include <fstream>
#include <iostream>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>

#include <eigen3/Eigen/Dense>

#define PI 3.141592653


using namespace std;

ros::Publisher pub_odom;
ros::Publisher pub_odom2d;
ros::Publisher pub_path;



std::mutex gmutex;
ofstream inStream;
int save_flag = 0;
int saved_count = 0;

// pose:body to odometry
void pose_frame_b2o (geometry_msgs::Pose pose_in, geometry_msgs::Pose &pose_out, Eigen::Vector3d &euler)
{
  // imu to odom
  Eigen::Matrix<double, 3, 3> Rmat;
  Rmat <<0, 1, 0,
         0, 0,-1,
        -1, 0, 0;
  Eigen::Quaterniond q_oi(Rmat);

  // world to imu,in imu
  Eigen::Quaterniond q_iw = Eigen::Quaterniond(pose_in.orientation.w,
                                            pose_in.orientation.x,
                                            pose_in.orientation.y,
                                            pose_in.orientation.z);
  Eigen::Vector3d t = Eigen::Vector3d(pose_in.position.x,
                                         pose_in.position.y,
                                         pose_in.position.z);

  // world to odom,in odom
  Eigen::Quaterniond q_ow =  q_iw * q_oi;

  // odom: world to local(x-forward,y-left,z-upward)
  Eigen::Matrix<double, 3, 3> mat;
  mat << 0, 1, 0,
        -1, 0, 0,
         0, 0, 1;
  Eigen::Quaterniond q_lw(mat);
  Eigen::Quaterniond q_lo = q_ow * q_lw; //odom to local

  euler = q_lo.toRotationMatrix().eulerAngles(0, 1, 2);
  //std::cout << euler.transpose()*57.3 << std::endl;

  Eigen::Vector3d t_ = q_lw.inverse() * t;
  pose_out.position.x = t_.x();
  pose_out.position.y = t_.y();
  pose_out.position.z = t_.z();
  pose_out.orientation.w = q_lo.w();
  pose_out.orientation.x = q_lo.x();
  pose_out.orientation.y = q_lo.y();
  pose_out.orientation.z = q_lo.z();
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
  nav_msgs::Odometry odometry;
  odometry = *odom_msg;

  Eigen::Vector3d euler;
  pose_frame_b2o(odom_msg->pose.pose, odometry.pose.pose, euler);

  pub_odom.publish(odometry);

  nav_msgs::Odometry odometry2d = odometry;
  odometry2d.header = odometry.header;
  odometry2d.pose.pose.position.z = 0;
  pub_odom2d.publish(odometry2d);

  if(save_flag) {
    inStream << odometry2d.pose.pose.position.x << " " <<odometry2d.pose.pose.position.y<<" " << euler[2] <<std::endl;
    saved_count++;
  }
}

void path_callback(const nav_msgs::Path::ConstPtr &path_msg)
{
  nav_msgs::Path path;
  path.header.stamp = path_msg->header.stamp;
  path.header.frame_id = "world";

  for(int i=0;i<path_msg->poses.size();i++) {
    geometry_msgs::PoseStamped pose;
    pose.header = path_msg->poses[i].header;
    Eigen::Vector3d euler;
    pose_frame_b2o(path_msg->poses[i].pose, pose.pose, euler);
    path.poses.push_back(pose);
  }

  pub_path.publish(path);
}


void save_path2d(nav_msgs::Path path_msg)
{
  int n = 0;

  for(int i=0;i<path_msg.poses.size();i++) {
    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.poses[i].header;
    Eigen::Vector3d euler;
    pose_frame_b2o(path_msg.poses[i].pose, pose.pose, euler);
    inStream<<pose.pose.position.x<<" "<<pose.pose.position.y<<" "<<pose.pose.position.z<<std::endl;
    n++;
  } 
}

void command()
{
  while(1) {
    char c = getchar();
    if (c == 'r') {
      if(!save_flag) {
        saved_count = 0;
        inStream.open("goal.txt",std::ios::out);
        if(!inStream.is_open()){
          ROS_ERROR("open goal.txt false!!!!");
          return;
        }
        save_flag = 1;
        cout<<"save start ... ";
      } else {
        inStream.close();
        save_flag = 0;
        cout<<"save stop:" << saved_count <<endl;
      }
    }
    std::chrono::milliseconds dura(5);
    std::this_thread::sleep_for(dura);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "nav_publisher");
  ros::NodeHandle n("~");

  ros::Subscriber sub_odom = n.subscribe("/pose_graph/pose_graph_odom", 20, odom_callback);
  ros::Subscriber sub_path = n.subscribe("/pose_graph/pose_graph_path", 10, path_callback);
  
  pub_odom = n.advertise<nav_msgs::Odometry>("/nav_publisher/odom_3d", 1);
  pub_odom2d = n.advertise<nav_msgs::Odometry>("/nav_publisher/odom_2d", 1);
  pub_path = n.advertise<nav_msgs::Path>("/nav_publisher/path", 10);


  std::thread keyboard_command_process;
  keyboard_command_process = std::thread(command);

  ros::spin();

  

  return 0;
}
