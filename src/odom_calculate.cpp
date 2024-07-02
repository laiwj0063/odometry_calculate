#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
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

// #define DEBUG

typedef struct {
  double x;
  double y;
  double th;
}Position;

typedef struct{
    double omega_L;
    double omega_R;
}Velocity;

Position position;
Velocity velocity;
void caculate(Velocity *velocity, Position* position);

struct MotionState {
    double vx;  // x 方向速度
    double vy;  // y 方向速度
    double omega; // 角速度
};

MotionState calculateMotion(Velocity* velocity, double R, double L, Position* position, double dt) {//L是兩輪間距
    // 计算左右轮的线速度
    double v_L = velocity->omega_L * R;
    double v_R = velocity->omega_R * R;

    // 计算车辆的线速度和角速度
    double v = (v_L + v_R) / 2;
    double omega = (v_R - v_L) / L;

    // 计算 x 和 y 方向的速度
    double vx = v * cos(position->th);
    double vy = v * sin(position->th);

    // 更新航向角
    double new_theta = position->th + omega * dt;

    #ifdef DEBUG
    cout<<"vx is "<<vx<<", vy is "<<vy<<", omega_z is "<<omega<<"\n";
    cout<<"-------------------------------------------------------------\n";
    #endif

    return {vx, vy, omega};
}

void callbackleft(const std_msgs::Float64::ConstPtr& data){
    velocity.omega_L = data->data;
    velocity.omega_L /= 60.0;
    velocity.omega_L *= 2*M_PI;
}
void callbackright(const std_msgs::Float64::ConstPtr& data){
    velocity.omega_R = data->data;
    velocity.omega_R /= 60.0;
    velocity.omega_R *= 2*M_PI;
}

void caculate(Velocity* velocity, Position* position){

    static ros::Time last_time = ros::Time::now();

    double R = 0.1;  // 轮子半径
    double L = 0.5;  // 轮子间的轴距

    ros::Rate r(10);
   
    ros::Time current_time = ros::Time::now();

    // Compute time step
    double dt = (current_time - last_time).toSec();       

    #ifdef DEBUG
    cout<<"dt is"<<dt<<"\n";
    cout<<"omega_L is "<<velocity->omega_L<<"\n";
    cout<<"omega_R is "<<velocity->omega_R<<"\n";
    #endif

    // Calculate motion state
    MotionState motion = calculateMotion(velocity, R, L, position, dt);

    // Update position and orientation
    position->x += motion.vx * dt;
    position->y += motion.vy * dt;
    position->th += motion.omega * dt;

    position->th = fmod(position->th, 2 * M_PI);
    if (position->th < 0)   position->th += 2 * M_PI;

    #ifdef DEBUG
    cout<<"x is "<<position->x<<", y is "<<position->y<<", theta_z is "<<position->th<<"\n";
    cout<<"-------------------------------------------------------------\n";
    #endif

    last_time = current_time;

    return;
}

void clear(Velocity* velocity, Position* position){
    velocity->omega_L = 0.0;
    velocity->omega_R = 0.0;
    position->x = 0.0;
    position->y = 0.0;
    position->th = 0.0;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "odom_calculate");
    
    clear(&velocity,&position);

    ros::NodeHandle n;
    ros::Subscriber subleft = n.subscribe("/left_wheel/rpm", 1, callbackleft);
    ros::Subscriber subright = n.subscribe("/right_wheel/rpm", 1, callbackright);
    ros::Publisher pub = n.advertise<geometry_msgs::Point>("/car_position", 1);
    geometry_msgs::Point msg;
    while (ros::ok()){
        caculate(&velocity,&position);
        #ifdef DEBUG
        cout<<"x is "<<position.x<<", y is "<<position.y<<"theta_z is "<<position.th<<"\n";
        #endif
        msg.x = position.x;
        msg.y = position.y;
        msg.z = position.th;
        #ifdef DEBUG
        cout<<"x is "<<msg.x<<", y is "<<msg.y<<"theta_z is "<<msg.z<<"\n";
        #endif
        pub.publish(msg);
        ros::spinOnce();
    }
    return 0;
}