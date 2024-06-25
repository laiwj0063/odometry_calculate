#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <cstdint>
#include <vector>
#include <chrono>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cmath>

using std::cout;

#define DEBUG

typedef struct {
  double x;
  double y;
  double z;
}Position;

Position position;
void caculate(double,double);

class Publisher{
public:
    Publisher(){
        pub = n.advertise<geometry_msgs::Point>("car_position", 1);
    }

    void publish(){
        geometry_msgs::Point msg;
        msg.x = position.x;
        msg.y = position.y;
        msg.z = position.z;
        pub.publish(msg);
    }

private:
    ros::NodeHandle n;
    ros::Publisher pub;
};

class Subscriber{
public:
    Subscriber(){
        ros::NodeHandle n;
        sub = n.subscribe("topic_name", 1, &Subscriber::callback, this);

        // create a Publisher object for republishing
        republisher = new Publisher();
    }

    ~Subscriber(){
        delete republisher;
    }

    void callback(const std_msgs::Float64MultiArray::ConstPtr& msg){

        double omega_L = msg->data[0];
        double omega_R = msg->data[1];
        caculate(omega_L,omega_R);
        republisher->publish();
    }

private:
    ros::Subscriber sub;
    Publisher* republisher;
};

struct MotionState {
    double vx;  // x 方向速度
    double vy;  // y 方向速度
    double omega; // 角速度
};

MotionState calculateMotion(double omega_L, double omega_R, double R, double L, double theta, double dt) {//L是兩輪間距
    // 计算左右轮的线速度
    double v_L = omega_L * R;
    double v_R = omega_R * R;

    // 计算车辆的线速度和角速度
    double v = (v_L + v_R) / 2;
    double omega = (v_R - v_L) / L;

    // 计算 x 和 y 方向的速度
    double vx = v * cos(theta);
    double vy = v * sin(theta);

    // 更新航向角
    double new_theta = theta + omega * dt;

    #ifdef DEBUG
    cout<<"vx is "<<vx<<", vy is "<<vy<<", omega_z is "<<omega<<"\n";
    cout<<"-------------------------------------------------------------\n";
    #endif

    return {vx, vy, omega};
}

void caculate(double omega_L, double omega_R){

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    double R = 0.1;  // 轮子半径
    double L = 0.5;  // 轮子间的轴距

    ros::Rate r(1.0);
   
    current_time = ros::Time::now();

    // Compute time step
    double dt = (current_time - last_time).toSec();       

    #ifdef DEBUG
    cout<<"omega_L is "<<omega_L<<"\n";
    cout<<"omega_R is "<<omega_R<<"\n";
    #endif

    // Calculate motion state
    MotionState motion = calculateMotion(omega_L, omega_R, R, L, position.z, dt);

    // Update position and orientation
    position.x += motion.vx * dt;
    position.y += motion.vy * dt;
    position.z += motion.omega * dt;

    #ifdef DEBUG
    cout<<"x is "<<position.x<<", y is "<<position.y<<", theta_z is "<<position.z<<"\n";
    cout<<"-------------------------------------------------------------\n";
    #endif

    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "odom_calculate");

    position.x = 0.0;
    position.y = 0.0;
    position.z = 0.0;

    Subscriber sub;
    ros::spin();

    return 0;
}