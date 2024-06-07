#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QDebug>
#include <std_msgs/String.h>
#include <common/public.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <common/mydisplay.h>

#include <QDir>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include "global_plan_sim_panel.h"

using namespace std;

namespace rviz_gui
{
    QStringList filenameInDir(string path)
    {
        //存储文件名称
        QStringList string_list;

        //判断路径是否存在
        QDir dir(QString::fromStdString(path));
        if (!dir.exists())  return string_list;

        //查看路径中后缀为.yaml格式的文件
        QStringList filters;
        filters << QString("*.yaml");
        dir.setFilter(QDir::Files | QDir::NoSymLinks); //设置类型过滤器，只为文件格式
        dir.setNameFilters(filters);                   //设置文件名称过滤器，只为filters格式

        //统计文件个数
        int dir_count = dir.count();
        for (int i = 0; i < dir_count; i++)
        {
            QString file_name = dir[i]; //文件名称
            file_name=file_name.left(file_name.size()-5);
            string_list.append(file_name);
        }
        return string_list;
    }

    Panel_Global_Plan_Sim::Panel_Global_Plan_Sim(QWidget *parent)
        : rviz::Panel(parent), ui(new Ui::Panel_Global_Plan_Sim)
    {
        ui->setupUi(this);

        nh = new ros::NodeHandle();
        nh_local = new ros::NodeHandle("~");

        trackpath_pub = nh->advertise<nav_msgs::Path>("/track_path", 10);
        task_pub = nh->advertise<mqtt_comm::task>("/task_cmd", 10);
        simpose_pub = nh->advertise<geometry_msgs::PoseStamped>("/sim_pose", 10);
        charge_cmd_pub = nh->advertise<std_msgs::String>("/charge_cmd", 10);
        
        battery_sub = nh->subscribe<data_comm::battery>("/battery_info", 10, &Panel_Global_Plan_Sim::BatteryCallback, this);   
        charge_station_sub = nh->subscribe<car_ctr::charge_station>("/charge_station", 10, &Panel_Global_Plan_Sim::ChargeStationCallback, this);   

        carstate_sub = nh->subscribe<car_ctr::car_state>("/car_state", 10, &Panel_Global_Plan_Sim::CarStateCallback, this);
        carctr_sub = nh->subscribe<car_ctr::car_ctr>("/car_cmd", 10, &Panel_Global_Plan_Sim::CarCtrCallback, this);
        target_sub = nh->subscribe<geometry_msgs::PoseStamped>("/local_path_plan/target_pose", 10, &Panel_Global_Plan_Sim::TargetCallback, this);
        mqtttask_sub = nh->subscribe<mqtt_comm::mqtt_task>("/mqtt_task", 10, &Panel_Global_Plan_Sim::mqttTaskCallback, this);

        front_obs_dis_sub = nh->subscribe<std_msgs::Float32>("/cloud_calculation_front/obs_dis", 10, &Panel_Global_Plan_Sim::FrontObsDisCallback, this);
        back_obs_dis_sub = nh->subscribe<std_msgs::Float32>("/cloud_calculation_back/obs_dis", 10, &Panel_Global_Plan_Sim::BackObsDisCallback, this);
        left_obs_dis_sub = nh->subscribe<std_msgs::Float32>("/cloud_calculation_left/obs_dis", 10, &Panel_Global_Plan_Sim::LeftObsDisCallback, this);
        right_obs_dis_sub = nh->subscribe<std_msgs::Float32>("/cloud_calculation_right/obs_dis", 10, &Panel_Global_Plan_Sim::RightObsDisCallback, this);
        remainpath_sub = nh->subscribe<std_msgs::Float64>("/local_path_plan/remainpath", 10, &Panel_Global_Plan_Sim::RemainPathCallback, this);
        
        control_sub = nh->subscribe<mqtt_comm::mqtt_controls>("/mqtt_controls", 10, &Panel_Global_Plan_Sim::MqttCtrCallback, this);


        qtmr.start(200);
        connect(&qtmr, SIGNAL(timeout()), this, SLOT(qtmrfunc()));

        connect(ui->btn_load, SIGNAL(clicked()), this, SLOT(btn_load_onclick()));
        connect(ui->btn_cleartrack, SIGNAL(clicked()), this, SLOT(btn_cleartrack_onclick()));

        connect(ui->btn_stop, SIGNAL(clicked()), this, SLOT(btn_stop_onclick()));
        connect(ui->btn_syscheck, SIGNAL(clicked()), this, SLOT(btn_syscheck_onclick()));
        connect(ui->btn_enable, SIGNAL(clicked()), this, SLOT(btn_enable_onclick()));
        connect(ui->btn_sample, SIGNAL(clicked()), this, SLOT(btn_sample_onclick()));
        connect(ui->btn_charge_ready, SIGNAL(clicked()), this, SLOT(btn_charge_ready_onclick()));
        connect(ui->btn_charge_close, SIGNAL(clicked()), this, SLOT(btn_charge_close_onclick()));
        connect(ui->btn_charge, SIGNAL(clicked()), this, SLOT(btn_charge_onclick()));
        connect(ui->btn_obstacle, SIGNAL(clicked()), this, SLOT(btn_obstacle_onclick()));
        connect(ui->btn_home, SIGNAL(clicked()), this, SLOT(btn_home_onclick()));

        char filename[1000];
        GetPackagePath("global_plan_sim", filename);
        int pos = strlen(filename);
        if (pos > 0)  filename[pos-1] = 0;
        sprintf(filename, "%s/path", filename);
        pathfilepath=filename;

        QStringList filenames = filenameInDir(pathfilepath);
        ui->cb_load->clear();
        ui->cb_load->addItems(filenames);

        string agvid, ver;
        nh->getParam("/agvId", agvid);
        nh->getParam("/version", ver);
        agvid="AGV: "+agvid+" "+ver;
        ui->label_agvid->setText(QString::fromStdString(agvid));
        
        matching_loc_enable = false;

        system("/home/bit/bit_agv_ws/mymount.sh &");
    }

    int Panel_Global_Plan_Sim::CheckRosNode(string name)
    {
        string msg="";
        nh->getParam(name+"/check/msg", msg);
        
        if(msg.find("OK")==msg.npos) return 1;
        else return 0;
    }

    void Panel_Global_Plan_Sim::UpdateErrCode()
    {
        string errcode="", warncode="";
        nh->getParam("/iot_comm/err", errcode);
        nh->getParam("/iot_comm/warn", warncode);
        
        char buf[100];
        sprintf(buf,"err: %s   ; warn: %s", errcode.c_str(), warncode.c_str()); 
        ui->lab8->setText(QString::fromStdString(buf));
    }

    void Panel_Global_Plan_Sim::PubTrackPath()  // 发布车辆走过的轨迹
    {
        track_path.header.frame_id = "map";
        track_path.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped pose_map, pose_base;
        pose_base.header.stamp = ros::Time::now();
        pose_base.header.frame_id = "base_link";
        pose_base.pose.orientation.w=1;
        transformPose("map", pose_base, pose_map, "XXX");

        bool add_flag = true;
        if (track_path.poses.size() > 0)
        {
            geometry_msgs::PoseStamped last_pose = *(track_path.poses.end() - 1);
            float dx = last_pose.pose.position.x - pose_map.pose.position.x;
            float dy = last_pose.pose.position.y - pose_map.pose.position.y;
            float ds = sqrt(pow(dx, 2) + pow(dy, 2));
            add_flag = (ds > 0.5);
        }
        if (add_flag)  track_path.poses.push_back(pose_map);
        //  保留1000m轨迹
        if(track_path.poses.size()>1000)  track_path.poses.erase(track_path.poses.begin());

        trackpath_pub.publish(track_path);
    }

    void Panel_Global_Plan_Sim::qtmrfunc()  // 定时
    {
        nh_local->getParam("/gps_base/utmx_zero", utm_x_zero);
        nh_local->getParam("/gps_base/utmy_zero", utm_y_zero);

        char buf[200];
        
        // float obs_dis_laser = 999, obs_dis_lidar = 999, obs_dis;
        // nh_local->getParam("/laser_radar_obs/obstacle_dis", obs_dis_laser);
        // nh_local->getParam("/lidar_radar_obs/obstacle_dis", obs_dis_lidar);
        // obs_dis = min(obs_dis_laser, obs_dis_lidar);

        sprintf(buf, "obstacle: front %.2f back %.2f left %.2f right %.2f", front_obs_dis, back_obs_dis,left_obs_dis,right_obs_dis);
        ui->lab7->setText(QString::fromUtf8(buf));

        string errcode="";
        nh_local->getParam("/mqtt_comm/err",errcode);
        sprintf(buf, "errcode: %s", errcode.c_str());
        ui->lab8->setText(QString::fromUtf8(buf));

        // PubTrackPath();

        nh_local->getParam("/pathtrack/run_enable", run_enable);
        // ROS_INFO("%d", run_enable);
        if(run_enable)  ui->btn_enable->setStyleSheet("background-color: rgb(0, 255, 0);");
        else ui->btn_enable->setStyleSheet("background-color: rgb(186, 189, 182);");

        if(charge_state)  ui->btn_charge->setStyleSheet("background-color: rgb(0, 255, 0);");
        else ui->btn_charge->setStyleSheet("background-color: rgb(186, 189, 182);");

        nh->getParam("/speedlimit/obstacle_enable", obstacle_enable);
        if(obstacle_enable)  ui->btn_obstacle->setStyleSheet("background-color: rgb(0, 255, 0);");
        else ui->btn_obstacle->setStyleSheet("background-color: rgb(186, 189, 182);");

        // nh_local->getParam("/pathtrack/obs_stop_enable", obs_enable);
        // if(obs_enable)  ui->btn_obs->setStyleSheet("background-color: rgb(0, 255, 0);");
        // else ui->btn_obs->setStyleSheet("background-color: rgb(186, 189, 182);");

        // UpdateErrCode();

        // static TTimer tmr;
        // if(tmr.GetValue()>0.5)
        // {
        //    tmr.Clear();
        //    string str=ui->cb_load->currentText().toStdString();
        //    LoadPath(str);     
        // }

        geometry_msgs::PoseStamped zero_base;
        zero_base.header.frame_id="base_link";
        zero_base.pose.orientation.w=1;
        transformPose("map",zero_base,pose_map);
        float angle=GetYawFromPose(pose_map)*180/M_PI;
        sprintf(buf, "pose: x=%.2f y=%.2f angle=%.1f°", pose_map.pose.position.x,pose_map.pose.position.y, angle);
        ui->lab1->setText(QString::fromUtf8(buf));
        ContinueProcess();
    }

    void Panel_Global_Plan_Sim::mqttTaskCallback(const mqtt_comm::mqtt_task::ConstPtr &msg)
    {
        cur_task.taskId=msg->taskId;
        task_id=msg->taskId;
        LoadPath(msg->pathId);
        repeat_flag=1;

        cur_task.stamp = ros::Time::now();
        cur_task.final_path=true;
        task_pub.publish(cur_task);
        // cur_task = *msg;
    }
    
    void Panel_Global_Plan_Sim::FrontObsDisCallback(const std_msgs::Float32::ConstPtr &msg)
    {
        front_obs_dis=msg->data;
    }

    void Panel_Global_Plan_Sim::RemainPathCallback(const std_msgs::Float64::ConstPtr &msg)
    {
        remain_path_length=msg->data;
    }

    void Panel_Global_Plan_Sim::MqttCtrCallback(const mqtt_comm::mqtt_controls::ConstPtr &msg)
    {
        if(msg->msgType=="control" && msg->ctrlType=="HOME" && msg->value==1)
        {
            btn_home_onclick();
        }

    }


    void Panel_Global_Plan_Sim::BackObsDisCallback(const std_msgs::Float32::ConstPtr &msg)
    {
        back_obs_dis=msg->data;
    }

    void Panel_Global_Plan_Sim::LeftObsDisCallback(const std_msgs::Float32::ConstPtr &msg)
    {
        left_obs_dis=msg->data;
    }

    void Panel_Global_Plan_Sim::RightObsDisCallback(const std_msgs::Float32::ConstPtr &msg)
    {
        right_obs_dis=msg->data;
    }

    void Panel_Global_Plan_Sim::BatteryCallback(const data_comm::battery::ConstPtr &msg)
    {
        battery_info= *msg;

        char buf[200];
        sprintf(buf, "battery: V %.1fv I %.1fa SOC %d%% T %d charge %d ", battery_info.voltage, battery_info.current, battery_info.SOC, battery_info.temperature, battery_info.charge_state);
        ui->lab5->setText(QString::fromUtf8(buf));
        // ROS_INFO("%2f",battery_info);
    }

    void Panel_Global_Plan_Sim::ChargeStationCallback(const car_ctr::charge_station::ConstPtr &msg)
    {
        charge_station=*msg;

        char buf[200];
        sprintf(buf, "charge station: door_open %d  pole_out %d  charge_ready %d", msg->door_open, msg->pole_out, msg->charge_ready);
        ui->lab6->setText(QString::fromUtf8(buf));
        // ROS_INFO("%2f",battery_info);
    }

    void Panel_Global_Plan_Sim::CarStateCallback(const car_ctr::car_state::ConstPtr &msg)  //接收车辆状态
    {
        char buf[200];
        sprintf(buf, "car_state: enable=%d turn=%d ctr=%d vel=%.1f", msg->enable, msg->turnmode, msg->ctrmode, msg->speed[0]);
        ui->lab3->setText(QString::fromUtf8(buf));
        car_state = *msg;
    }

    void Panel_Global_Plan_Sim::CarCtrCallback(const car_ctr::car_ctr::ConstPtr &msg)      //接收车辆状态
    {
        char buf[200];
        sprintf(buf, "car_ctr: turnmode=%d vel=%.1f angle=%.1f", msg->turnmode, msg->speed, msg->angle);
        ui->lab4->setText(QString::fromUtf8(buf));
    }

    void Panel_Global_Plan_Sim::TargetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        char buf[200];
        float angle=GetYawFromPose(*msg)*180/M_PI;
        sprintf(buf, "target:x=%.2f y=%.2f angle=%.2f", msg->pose.position.x, msg->pose.position.y, angle);
        ui->lab2->setText(QString::fromUtf8(buf));
    }

    //  根据当前路径生成HOME路径
    void Panel_Global_Plan_Sim::GenHomePath()
    {
        if(cur_task.path.size()<2)  return;
        
        // 第一点是home，路径点反序排列，变为最后一点
        bool path_reverse=(cur_task.path.front().caption=="HOME");
        if(path_reverse)
        {
            // printf("%d ",cur_task.path.size());
            vector<mqtt_comm::path_point>  tmp_path=cur_task.path;
            cur_task.path.clear();
            for(int i=tmp_path.size()-1; i>=0; i--)
            {
                
                tmp_path[i].actions.clear();
                cur_task.path.push_back(tmp_path[i]);
            }
            // printf("%d \n",cur_task.path.size());
        }

        // 最后点是home，路径点正序不变
        bool path_positive=(cur_task.path.back().caption=="HOME");
        if(path_positive)
        {
            // printf("%d ",cur_task.path.size());
            vector<mqtt_comm::path_point>  tmp_path=cur_task.path;
            cur_task.path.clear();
            for(int i=0; i<tmp_path.size(); i++)
            {
                
                tmp_path[i].actions.clear();
                cur_task.path.push_back(tmp_path[i]);
            }
            // printf("%d \n",cur_task.path.size());
        }


        //  清除所有任务，倒数第二点加上充电任务
        int n=cur_task.path.size();
        // cur_task.path[n-2].actions.clear();
        mqtt_comm::path_point_action act;
        act.caption="charge";
        act.params.push_back(20);
        cur_task.path[n-2].actions.push_back(act);
    }

    void Panel_Global_Plan_Sim::LoadPath(string fn)
    {
        fn=pathfilepath+"/"+fn+".yaml";
        if(access(fn.c_str(), F_OK)!=0)  return;
        // printf("%s\n", fn.c_str());

        YAML::Node config = YAML::LoadFile(fn);
        cur_task.cmd="run task";
        cur_task.path.clear();

        if(config["next_path"].IsDefined())  next_path=config["next_path"].as<string>();
        else  next_path="";

        for(int i=0;i<1000;i++)
        {
            char posename[100];
            sprintf(posename,"pose%d",i);
            
            if (config[posename]["map_x"].IsDefined())
            {
                YAML::Node node=config[posename];
                mqtt_comm::path_point point;
                if(node["circle"].IsDefined())
                {
                    float c_x=node["map_x"].as<float>();
                    float c_y=node["map_y"].as<float>();
                    YAML::Node cnode=node["circle"];
                    float c_r=cnode[0].as<float>();
                    float c_a1=cnode[1].as<float>();
                    float c_a2=cnode[2].as<float>();
                    int n=cnode[3].as<int>();
                    int order=cnode[4].as<int>();
                    float c_da=(c_a2-c_a1)/n;
                    
                    point.vehSpeed = node["vel"].as<float>();
                    vector<mqtt_comm::path_point> points;
                    for(int i=0;i<=n;i++)
                    {
                        float angle=(c_a1+c_da*i)*M_PI/180;
                        point.pointX = c_x+c_r*cos(angle);
                        point.pointY = c_y+c_r*sin(angle);
                        points.push_back(point);
                    }
                    
                    for(int i=0;i<=n;i++)
                    {
                        float h_angle=0;
                        if(i==n)  h_angle=180/M_PI*atan2(points[i].pointY-points[i-1].pointY, points[i].pointX-points[i-1].pointX);
                        else h_angle=180/M_PI*atan2(points[i+1].pointY-points[i].pointY, points[i+1].pointX-points[i].pointX); 
                        if(order==-1)  h_angle+=180;
                        points[i].pointHA=h_angle;
                        point.pointHA = h_angle;
                        point.pointX=points[i].pointX;
                        point.pointY=points[i].pointY;
                        // if(node["lane_type"].IsDefined())  point.lane_type=node["lane_type"].as<int>();
                        // else point.lane_type=0;
                        cur_task.path.push_back(point);
                        // ROS_INFO("111111111111111111111");
                    }
                }
                else 
                {
                    point.pointX = config[posename]["map_x"].as<float>();
                    point.pointY = config[posename]["map_y"].as<float>();
                    point.pointHA = config[posename]["heading"].as<float>();
                    point.vehSpeed = config[posename]["vel"].as<float>();
                    point.caption="";
                    if(config[posename]["caption"].IsDefined())  
                        point.caption = config[posename]["caption"].as<string>();

                    for(int j=0;j<100;j++)
                    {
                        char actname[100];
                        sprintf(actname,"action%d",j);

                        if(config[posename][actname].IsDefined())
                        {
                            YAML::Node node=config[posename][actname];
                            mqtt_comm::path_point_action act;
                            for(int i=0;i<node.size();i++)
                                if(i==0)  act.caption=node[i].as<string>();
                                else act.params.push_back(node[i].as<float>());
                            point.actions.push_back(act);    
                            // ROS_INFO("%d",node.size());
                        }
                        else break;
                    }
                    cur_task.path.push_back(point); 
                }   
            }
            else break;
        }

        if(cur_task.path.size()<=1)  return;

        cur_task.only_akm=config["onlyAkm"].as<bool>();
        cur_task.accordingPathdir=config["accordingPathdir"].as<bool>();
        if(cur_task.accordingPathdir)
        {
            for(int i=0;i<cur_task.path.size();i++)
                if(i==cur_task.path.size()-1)  cur_task.path[i].pointHA=cur_task.path[i-1].pointHA;
                else
                {
                    float x1 = cur_task.path[i].pointX, y1 = cur_task.path[i].pointY;
                    float x2 = cur_task.path[i+1].pointX, y2 = cur_task.path[i+1].pointY;
                    cur_task.path[i].pointHA = atan2(y2 - y1, x2 - x1)*180/M_PI;
                }
        }
    }

    void Panel_Global_Plan_Sim::btn_load_onclick()    //  发布路径指令
    {
        btn_stop_onclick();
        usleep(50000);
        
        string str=ui->cb_load->currentText().toStdString();
        LoadPath(str);
        cur_task.stamp = ros::Time::now();
        cur_task.final_path=true;
        cur_task.taskId=task_id;
        task_pub.publish(cur_task);
        repeat_flag=1;
    }

    void Panel_Global_Plan_Sim::btn_home_onclick()    
    {
        GenHomePath();
        task_pub.publish(cur_task);
        repeat_flag=0;
    }
   
    void Panel_Global_Plan_Sim::btn_stop_onclick()    //  发送停车指令
    {
        cur_task.stamp = ros::Time::now();
        cur_task.cmd = "stop task";
        cur_task.path.clear();
        task_pub.publish(cur_task);
        repeat_flag=0;
    }

    void Panel_Global_Plan_Sim::btn_cleartrack_onclick()    //  清除跟踪轨迹
    {
        track_path.poses.clear();
    }

    void Panel_Global_Plan_Sim::btn_syscheck_onclick()     //  系统状态检查
    {
        d_nodecheck=new dialog_node_check(this);
        d_nodecheck->setModal(true);
        d_nodecheck->show();
    }

    void Panel_Global_Plan_Sim::btn_enable_onclick()      //  系统使能
    {
        nh_local->setParam("/pathtrack/run_enable", !run_enable);
    }

    void Panel_Global_Plan_Sim::btn_charge_ready_onclick()      
    {
        std_msgs::String cmd;
        cmd.data="open_out";
        charge_cmd_pub.publish(cmd);
    }

    void Panel_Global_Plan_Sim::btn_charge_close_onclick()      
    {
        std_msgs::String cmd;
        cmd.data="close_in";
        charge_cmd_pub.publish(cmd);
    }

    void Panel_Global_Plan_Sim::btn_charge_onclick()      
    {
        std_msgs::String cmd;
        charge_state=!charge_state;
        
        if(charge_state)  cmd.data="charge 1";
        else cmd.data="charge 0";
        charge_cmd_pub.publish(cmd);
    }

    void Panel_Global_Plan_Sim::btn_obstacle_onclick()      
    {
        nh->setParam("/speedlimit/obstacle_enable", !obstacle_enable);   
    }

    void Panel_Global_Plan_Sim::btn_sample_onclick()      //  采集数据
    {
        string fn=ui->cb_load->currentText().toStdString();
        string filename=pathfilepath+"/"+fn+".yaml";
        // ROS_INFO("%s",filename.c_str());

        YAML::Node config = YAML::LoadFile(filename);
        
        char posename[100];
        for(int i=0;i<10000;i++)
        {
            sprintf(posename,"pose%d",i);
            if (config[posename]["map_x"].IsDefined()==false)  break;
        }

        YAML::Node newnode=config[posename];
        newnode["map_x"]=pose_map.pose.position.x;
        newnode["map_y"]=pose_map.pose.position.y;
        newnode["heading"]=GetYawFromPose(pose_map)*180/M_PI;
        newnode["vel"]=0.3;
        newnode["caption"]="";
        
        ofstream ofs(filename);
        ofs << config;
        ofs.close();
    }

    void Panel_Global_Plan_Sim::ContinueProcess()  
    {
        // string paw_state="";
        // nh_local->getParam("/pawcontrol/paw_state", paw_state);

        static TTimer tmr;
        if(repeat_flag==1) 
        {
            tmr.Clear();
            repeat_flag++;
        }
        else if( repeat_flag==2 && tmr.GetValue()>3 && fabs(car_ctr.speed)<0.01 && remain_path_length<0.1)
        {
            repeat_flag++;
            tmr.Clear();
        }
        else if (repeat_flag==3 && tmr.GetValue()>5)
        {           
            if(next_path!="")  
            {
                ui->cb_load->setCurrentText(QString::fromStdString(next_path));
                tmr.Clear();  
                btn_load_onclick();
            }
            else repeat_flag = 0;
        } 
    }    


} // end namespace rviz_teleop_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_gui::Panel_Global_Plan_Sim, rviz::Panel)
// END_TUTORIAL
