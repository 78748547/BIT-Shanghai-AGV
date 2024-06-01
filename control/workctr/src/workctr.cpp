#include <mqtt_comm/path_point_action.h>
#include <common/public.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <mqtt_comm/task.h>
#include <common/mydisplay.h>
#include <sys/stat.h>
#include <ctime>

#include <iostream>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>

using namespace std;

class TWorkCtr
{
private:
    ros::NodeHandle *nh; 
    ros::Subscriber task_sub, turntable_sub; 
    ros::Publisher turntablectr_pub, work_speedlimit_pub, charge_cmd_pub; 

    float remain_path_length=0; // 剩余路径长度
    mqtt_comm::task cur_task; // 当前任务
    TNodeCheck *nodecheck; // 节点检查对象
    vector<mqtt_comm::path_point_action> actions; // 动作列表
    float act_pitch_angle, act_azimuth_angle; // 动作角度
    string task_id="ABC001", pos_id=""; // 任务ID和位置ID
    char savepath[200]; // 保存路径
    bool working_flag=false;

    void CreateSavePath();

public:
    void TaskCallback(const mqtt_comm::task::ConstPtr &msg); 
    void TurntableCallback(const std_msgs::String::ConstPtr &msg); 
    void FindPoseAction(); 
    void Run(); 
    int TurntableActionRun(mqtt_comm::path_point_action action);
    int ChargeStationActionRun(mqtt_comm::path_point_action action);

    TWorkCtr()
    {
        nh = new ros::NodeHandle("~");  // 参数"~"表示使用默认命名空间

        // 创建一个TNodeCheck对象，并设置其节点句柄和参数名?
        nodecheck=new TNodeCheck(nh, "node_rate");
        // 调用TNodeCheck对象的Find方法查找参数"node_rate"，并设置其限制值为15?
        nodecheck->Find("node_rate")->SetLimit(15);

        task_sub = nh->subscribe<mqtt_comm::task>("/task_cmd", 10, &TWorkCtr::TaskCallback, this);
        turntable_sub = nh->subscribe<std_msgs::String>("/turntable/table_state", 10, &TWorkCtr::TurntableCallback, this);
        
        turntablectr_pub = nh->advertise<std_msgs::String>("/turntable_ctr", 10);
        work_speedlimit_pub = nh->advertise<std_msgs::Float32>("speedlimit", 10);
        charge_cmd_pub = nh->advertise<std_msgs::String>("/charge_cmd", 10);

        nh->setParam("/work_state","work_done"); //设置参数work_state为work_done?
    }
};


//  接收任务指令
void TWorkCtr::TaskCallback(const mqtt_comm::task::ConstPtr &msg)
{
    cur_task=*msg;
    task_id=msg->taskId;
    // ROS_INFO("%s", task_id.c_str());
}

void TWorkCtr::TurntableCallback(const std_msgs::String::ConstPtr &msg)
{
    vector<string> ss = split(msg->data, ";");
    for (int i = 0; i < ss.size(); i++)
    {
        vector<string> subs = split(ss[i], " ");
        if (subs[0] == "pitch")
        {
            act_pitch_angle = atof(subs[2].c_str());
            // reached_flag = atoi(subs[3].c_str());
        }
        else if (subs[0] == "azimuth")
        {
            act_azimuth_angle = atof(subs[2].c_str());
            // reached_flag = atoi(subs[3].c_str());
        }
    }

    // ROS_INFO("%.2f %.2f", act_pitch_angle, act_azimuth_angle);
}

void TWorkCtr::FindPoseAction()
{
    if(actions.size()>0 || cur_task.path.size()==0)  return;   //为什么actions.size()>0？

    for(auto &it:cur_task.path)
        if(it.actions.size()>0)
        {
            geometry_msgs::PointStamped p_map, p_base;
            p_map.header.frame_id="map";
            p_map.header.stamp=ros::Time::now();
            p_map.point.x=it.pointX, p_map.point.y=it.pointY;
            transformPoint("base_link", p_map, p_base, "AAA"); //把p_map转到base_link坐标系，并赋值给p_base，AAA？

            // 如果p_base的x坐标绝对值小于0.02，并且x坐标和y坐标的绝对值都小于0.1,
            // 则将路径点的动作列表赋值给全局动作列表actions，清空路径点的动作列表，并设置pos_id为路径点的caption
            if(p_base.point.x<0.02 && fabs(p_base.point.x)<0.1 && fabs(p_base.point.y)<0.1)
            // if( fabs(p_base.point.x)<0.5 && fabs(p_base.point.y)<0.5)   //根据跟踪定位精度修改
            {
                actions=it.actions;
                it.actions.clear();
                // ROS_INFO("action size=%d", actions.size());
                pos_id=it.caption;
                break;
            }
        }
}

int TWorkCtr::ChargeStationActionRun(mqtt_comm::path_point_action action)
{
    int finished_flag=0;
    
    float duration=action.params[0];  //持续时间

    static int action_flag=0;  //动作标志位
    static TTimer act_tmr; //动作计时器
    if(action_flag==0)  //  打开充电站门
    {
        std_msgs::String cmd;
        cmd.data="open_out";
        charge_cmd_pub.publish(cmd);
        action_flag++;  act_tmr.Clear();
    }
    else if(action_flag==1 && act_tmr.GetValue()>duration)
    {
        action_flag=0;
        finished_flag=1;
    }

    working_flag=action_flag>0;
    
    return finished_flag;
}

void TWorkCtr::CreateSavePath()
{
    strcpy(savepath,"/home/bit/save_data");
    mkdir(savepath,S_IRWXU);
    
    time_t t = time(nullptr);
    struct tm* now = localtime(&t);
    sprintf(savepath,"%s/%04d-%02d-%02d",savepath, now->tm_year+1900, now->tm_mon+1, now->tm_mday);
    mkdir(savepath,S_IRWXU);

    // char task_id[500];
    // sprintf(task_id,"task-%02d", now->tm_hour);  // 模拟ID

    sprintf(savepath,"%s/%s",savepath, task_id.c_str());
    mkdir(savepath,S_IRWXU);
    
    sprintf(savepath,"%s/%s",savepath,pos_id.c_str());
    mkdir(savepath,S_IRWXU);
    nh->setParam("/save_file_path", savepath);
}

int TWorkCtr::TurntableActionRun(mqtt_comm::path_point_action action)
{
    int finished_flag=0;
    
    float pitch_angle=action.params[0];  //俯仰角
    float azimuth_angle=action.params[1];  //方位角
    float duration=action.params[2];  //持续时间

    static int action_flag=0;  //动作标志位
    static TTimer act_tmr; //动作计时器
    if(action_flag==0)
    {
        char strbuf[500] = {0};
        sprintf(strbuf, "PITCH MOV %.2f 10;AZIMUTH MOV %.2f 10", pitch_angle, azimuth_angle); 
            
        std_msgs::String str_msg;
        str_msg.data = strbuf;
        turntablectr_pub.publish(str_msg);  //控制转台运动

        action_flag++;  act_tmr.Clear();

        CreateSavePath();
        // ROS_INFO("%s start", action.caption.c_str());
    }
    else if(action_flag==1 && fabs(pitch_angle-act_pitch_angle)<0.2 && fabs(azimuth_angle-act_azimuth_angle)<0.2)
    {
        action_flag++;  act_tmr.Clear();
        if(actions.size()>1)   
        {
            nh->setParam("/work_state","start_work");
            char cmd[400];
            // 默认采样率48000hz
            sprintf(cmd,"ffmpeg -y -f alsa -i default -t %.1f %s/audio.wav &",duration, savepath);
            // sprintf(cmd,"ffmpeg -y -f alsa -i default -t %.1f %s/audio.wav",duration, globalTaskPath);
            system(cmd);
            //  ROS_INFO("work start!");
        }
    }
    else if(action_flag==2)
    {
        //  此处加入作业控制代码
        if(act_tmr.GetValue()>duration)  
        {
            // ROS_INFO("%s stop", action.caption.c_str());
            action_flag=0;
            if(actions.size()>1)  
            {
                nh->setParam("/work_state","work_done");
                // ROS_INFO("work done!");
            }

            finished_flag=1;
            // actions.erase(actions.begin());
        }
    }

    working_flag=action_flag>0;

    return finished_flag;
}

void TWorkCtr::Run()
{
    nodecheck->Find("node_rate")->Beat();  // 调用nodecheck对象的Beat函数？

    FindPoseAction();  //找到当前位置的动作列表

    std_msgs::Float32 speedlimit_msg;
    if(actions.size()>0 || working_flag)   
    {
        speedlimit_msg.data=0;
        mqtt_comm::path_point_action action=actions.front(); //获取第一个动作

        if(action.caption.find("act")!=string::npos && TurntableActionRun(action))  actions.erase(actions.begin());
        if(action.caption.find("charge")!=string::npos && ChargeStationActionRun(action))  actions.erase(actions.begin());
    }
    else
    {
        speedlimit_msg.data=999;
    }

    work_speedlimit_pub.publish(speedlimit_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "work_ctr");

    TWorkCtr workctr;
    ros::Rate looprate(20);
    // system("sudo mount -t nfs -o nolock,nfsvers=3 192.168.3.101:/test /home/bit/test_data");
    while (ros::ok())
    {
        workctr.Run();

        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
};