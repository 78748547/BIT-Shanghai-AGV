#ifndef ABC_H
#define ABC_H

//所需要包含的头文件
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h> //plugin基类的头文件
#include <std_msgs/String.h>
#include <QTimer>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <gps/MyGPS_msg.h>
#include <car_ctr/car_state.h>
#include <car_ctr/car_ctr.h>
#endif

#include "ui_global_plan_sim.h"
#include "node_check.h"
#include <yaml-cpp/yaml.h>
#include <mqtt_comm/task.h>
#include <mqtt_comm/mqtt_task.h>
#include <mqtt_comm/path_point.h>
#include <mqtt_comm/mqtt_controls.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <data_comm/battery.h>
#include <car_ctr/charge_station.h>

using namespace std;

namespace rviz_gui
{
    // 所有的plugin都必须是rviz::Panel的子类
    class Panel_Global_Plan_Sim : public rviz::Panel
    {
        // 后边需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏
        Q_OBJECT
    public:
        // 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可
        Panel_Global_Plan_Sim(QWidget *parent = 0);

        // 公共槽.
    public Q_SLOTS:

        // 内部槽.
    protected Q_SLOTS:
        void qtmrfunc();

        void btn_load_onclick();
        void btn_cleartrack_onclick();
        void btn_stop_onclick();
        void btn_syscheck_onclick();
        void btn_enable_onclick();
        void btn_sample_onclick();
        void btn_charge_ready_onclick();
        void btn_charge_onclick();
        void btn_charge_close_onclick();
        void btn_obstacle_onclick();
        void btn_home_onclick();
        
        // 内部变量.
    protected:
        ros::NodeHandle *nh, *nh_local;
        ros::Publisher task_pub, trackpath_pub, simpose_pub, charge_cmd_pub;
        ros::Subscriber carstate_sub, carctr_sub, target_sub, mqtttask_sub,battery_sub,charge_station_sub;
        ros::Subscriber front_obs_dis_sub,back_obs_dis_sub,left_obs_dis_sub,right_obs_dis_sub,remainpath_sub,control_sub;

        geometry_msgs::PoseStamped target_pose, pose_map;
        double utm_x_zero = 0, utm_y_zero = 0;
        nav_msgs::Path track_path;
        car_ctr::car_ctr car_ctr;
        car_ctr::car_state car_state;
        bool run_enable = false, obs_enable = false;
        string pathfilepath;
        data_comm::battery battery_info;
        mqtt_comm::task cur_task;
        car_ctr::charge_station charge_station;
        string next_path="";   
        double remain_path_length; 
        int repeat_flag;
        bool charge_state=false;
        bool obstacle_enable=false;
        string task_id="task001";
                
        float front_obs_dis=999, back_obs_dis=999, left_obs_dis=999,right_obs_dis=999;

        void LoadPath(string fn);
        void GenHomePath();

        void PubTrackPath();
        void CarStateCallback(const car_ctr::car_state::ConstPtr &msg);
        void CarCtrCallback(const car_ctr::car_ctr::ConstPtr &msg);
        void TargetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void FrontObsDisCallback(const std_msgs::Float32::ConstPtr &msg);
        void BackObsDisCallback(const std_msgs::Float32::ConstPtr &msg);
        void LeftObsDisCallback(const std_msgs::Float32::ConstPtr &msg);
        void RightObsDisCallback(const std_msgs::Float32::ConstPtr &msg);
        void mqttTaskCallback(const mqtt_comm::mqtt_task::ConstPtr &msg);
        void BatteryCallback(const data_comm::battery::ConstPtr &msg);
        void RemainPathCallback(const std_msgs::Float64::ConstPtr &msg);
        void ChargeStationCallback(const car_ctr::charge_station::ConstPtr &msg);
        void MqttCtrCallback(const mqtt_comm::mqtt_controls::ConstPtr &msg);
        int CheckRosNode(string name);
        void UpdateErrCode();

    private:
        Ui::Panel_Global_Plan_Sim *ui;
        dialog_node_check *d_nodecheck;
        QTimer qtmr;
        bool matching_loc_enable;
        void ContinueProcess();    
    };
}

#endif
