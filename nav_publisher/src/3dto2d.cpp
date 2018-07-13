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
int start_flag = 1; //初始化成功那一刻odmetry的pose当作start frame
int saved_count = 0;
int pose_convert_flag = 0;

nav_msgs::Odometry start_odom;
nav_msgs::Path path;

// pose:body to odometry
void pose_frame_b2o (geometry_msgs::Pose pose_in, geometry_msgs::Pose &pose_out)
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
  Eigen::Quaterniond q_ow =  q_iw * q_oi; //没有问题了

#if 0
  Eigen::Vector3d euler = q_ow.toRotationMatrix().eulerAngles(0, 1, 2);
  std::cout <<"     ==="<< euler.transpose()*57.3 << std::endl;
#endif

  pose_out.position.x = t.x();
  pose_out.position.y = t.y();
  pose_out.position.z = t.z();
  pose_out.orientation.w = q_ow.w();
  pose_out.orientation.x = q_ow.x();
  pose_out.orientation.y = q_ow.y();
  pose_out.orientation.z = q_ow.z();
}

// pose: relative to start frame,in odom cor
void pose_frame_o2start (geometry_msgs::Pose pose_in, geometry_msgs::Pose &pose_out)
{
  Eigen::Quaterniond q_start = Eigen::Quaterniond(start_odom.pose.pose.orientation.w,
                                              start_odom.pose.pose.orientation.x,
                                              start_odom.pose.pose.orientation.y,
                                              start_odom.pose.pose.orientation.z);
  Eigen::Vector3d t_start = Eigen::Vector3d{start_odom.pose.pose.position.x,
                                            start_odom.pose.pose.position.y,
                                            start_odom.pose.pose.position.z};
  Eigen::Quaterniond q_curr = Eigen::Quaterniond(pose_in.orientation.w,
                                            pose_in.orientation.x,
                                            pose_in.orientation.y,
                                            pose_in.orientation.z);
  Eigen::Vector3d t_curr = Eigen::Vector3d{pose_in.position.x,
                                            pose_in.position.y,
                                            pose_in.position.z};

  //Eigen::Quaterniond baseRgt = q_start * q_curr.inverse();
  //Eigen::Vector3d baseTgt = t_start - baseRgt * t_curr;
  Eigen::Quaterniond baseRgt = q_start.inverse() * q_curr;
  Eigen::Vector3d baseTgt = q_start.inverse()*(t_curr - t_start);

  pose_out.position.x = baseTgt.x();
  pose_out.position.y = baseTgt.y();
  pose_out.position.z = baseTgt.z();
  pose_out.orientation.w = baseRgt.w();
  pose_out.orientation.x = baseRgt.x();
  pose_out.orientation.y = baseRgt.y();
  pose_out.orientation.z = baseRgt.z();
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
  nav_msgs::Odometry odometry = *odom_msg;
  pose_frame_b2o(odom_msg->pose.pose, odometry.pose.pose);
  pub_odom.publish(odometry);

  // 设置初始的odom位姿
  if(start_flag) {
    start_odom = odometry;
    start_flag = 0;
    pose_convert_flag = 1;
    cout<<"set start frame success"<<endl;
  }

  // 转换pose，以start_odom为基准
  if(pose_convert_flag) {
#if 1
  //publish tf
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(start_odom.pose.pose.position.x,
                                  start_odom.pose.pose.position.y,
                                  start_odom.pose.pose.position.z));
  q.setW(start_odom.pose.pose.orientation.w);
  q.setX(start_odom.pose.pose.orientation.x);
  q.setY(start_odom.pose.pose.orientation.y);
  q.setZ(start_odom.pose.pose.orientation.z);
  transform.setRotation(q.inverse());
  br.sendTransform(tf::StampedTransform(transform, odom_msg->header.stamp, "world", "odom"));
#endif

    // start: world to i, curr: world to j
    // baseRgt: i to j
    Eigen::Quaterniond q_start = Eigen::Quaterniond(start_odom.pose.pose.orientation.w,
                                              start_odom.pose.pose.orientation.x,
                                              start_odom.pose.pose.orientation.y,
                                              start_odom.pose.pose.orientation.z);
    Eigen::Vector3d t_start = Eigen::Vector3d{start_odom.pose.pose.position.x,
                                              start_odom.pose.pose.position.y,
                                              start_odom.pose.pose.position.z};
    Eigen::Quaterniond q_curr = Eigen::Quaterniond(odometry.pose.pose.orientation.w,
                                              odometry.pose.pose.orientation.x,
                                              odometry.pose.pose.orientation.y,
                                              odometry.pose.pose.orientation.z);
    Eigen::Vector3d t_curr = Eigen::Vector3d{odometry.pose.pose.position.x,
                                              odometry.pose.pose.position.y,
                                              odometry.pose.pose.position.z};

    //Eigen::Quaterniond baseRgt = q_start * q_curr.inverse();
    //Eigen::Vector3d baseTgt = t_start - baseRgt * t_curr;
    Eigen::Quaterniond baseRgt = q_start.inverse() * q_curr;
    Eigen::Vector3d baseTgt = q_start.inverse()*(t_curr - t_start);



    // euler angle
    // eigen的转欧拉角有问题,16变-164
    //Eigen::Vector3d euler = baseRgt.normalized().toRotationMatrix().eulerAngles(0, 1, 2); 
    //Eigen::Vector3d euler = baseRgt.normalized().toRotationMatrix().eulerAngles(1, 2, 0);
    //std::cout <<odometry.header.stamp<<"  "<<baseTgt.x()<<","<<baseTgt.y()<<"  |  "<< euler.transpose()*57.3<<std::endl;
    //std::cout << "      q: " <<baseRgt.w() <<" "<< baseRgt.x() <<" "<< baseRgt.y() <<" "<< baseRgt.z() <<std::endl;

    //
    nav_msgs::Odometry odom_relative = odometry;
    odom_relative.header = odometry.header;
    odom_relative.header.frame_id = "odom";
    odom_relative.pose.pose.position.x = baseTgt.x();
    odom_relative.pose.pose.position.y = baseTgt.y();
    //odom_relative.pose.pose.position.z = baseTgt.z();
    odom_relative.pose.pose.position.z = 0;
    odom_relative.pose.pose.orientation.w = baseRgt.w();
    odom_relative.pose.pose.orientation.x = baseRgt.x();
    odom_relative.pose.pose.orientation.y = baseRgt.y();
    odom_relative.pose.pose.orientation.z = baseRgt.z();
    pub_odom2d.publish(odom_relative);
  }

}

void path_callback(const nav_msgs::Path::ConstPtr &path_msg)
{
  path.poses.clear();
  path.poses.resize(0);
  path.header = path_msg->header;
  path.header.frame_id = "odom";

  for(int i=0;i<path_msg->poses.size();i++) {
    geometry_msgs::PoseStamped pose;
    pose.header = path_msg->poses[i].header;
    pose_frame_b2o(path_msg->poses[i].pose, pose.pose);

    geometry_msgs::PoseStamped pose_relative;
    pose_frame_o2start(pose.pose, pose_relative.pose);
    path.poses.push_back(pose_relative);
    if(save_flag) {
      tf::Quaternion quat;
      tf::quaternionMsgToTF(pose_relative.pose.orientation, quat);
      double roll,pitch,yaw;
      tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
      inStream<<pose_relative.pose.position.x<<" "<<pose_relative.pose.position.y<<" "<< yaw<<std::endl;
    }
  }

  pub_path.publish(path);

  if(save_flag) {
    save_flag = 0;
    inStream.close();
    cout<<"record trojactory stop, poses count:" << path.poses.size() <<endl;
  }  
}


void save_path2d(nav_msgs::Path path_msg)
{
  int n = 0;

  for(int i=0;i<path_msg.poses.size();i++) {
    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.poses[i].header;
    pose_frame_b2o(path_msg.poses[i].pose, pose.pose);
    inStream<<pose.pose.position.x<<" "<<pose.pose.position.y<<" "<<pose.pose.position.z<<std::endl;
    n++;
  } 
}

void command()
{
  while(1) {
    char c = getchar();
    switch(c) {
      case 'r':{
        if(!save_flag) {
          saved_count = 0;
          inStream.open("goal.txt",std::ios::out);
          if(!inStream.is_open()){
            ROS_ERROR("open goal.txt false!!!!");
            return;
          }
          save_flag = 1;
          cout<<"record trojactory,start ... "<<endl;
        }
        break;
      }
      case 's':{
        cout<<"set start postion."<<endl;
        start_flag = 1;

        break;
      }
      default:
        break;
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
  
  pub_odom = n.advertise<nav_msgs::Odometry>("/nav_publisher/odom_3d", 10);
  pub_odom2d = n.advertise<nav_msgs::Odometry>("/odom", 10);
  pub_path = n.advertise<nav_msgs::Path>("/nav_publisher/odom_path", 10);

  std::thread keyboard_command_process;
  keyboard_command_process = std::thread(command);

  ros::spin();

  

  return 0;
}
