#include <common/public.h>
#include <common/myudp.h>
#include <common/can.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <car_ctr/car_state.h>
#include <car_ctr/car_ctr.h>
#include <car_ctr/charge_station.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <data_comm/battery.h>
#include <mqtt_comm/mqtt_controls.h>
#include <common/public.h>

class TCharge 
{
private:
    TUDP *udp;
    ros::NodeHandle *nh;
    TTimer charge_check_tmr;
    TNodeCheck *nodecheck;
	
    ros::Subscriber charge_cmd_sub, battery_sub,control_sub;
    ros::Publisher charge_station_pub;

    bool ac220_charging=false, charge_ready=false;
    data_comm::battery battery_info;

    void ChargeCmdCallback(const std_msgs::String::ConstPtr &msg)
    {
        char buf[1000];
        strcpy(buf,msg->data.c_str());
        udp->Send(buf);
    }

    void StopCharge()
    {
        charge_check_tmr.Clear();
        udp->Send("charge 0");
        usleep(500000);
        udp->Send("close_in");
        ROS_INFO("STOP CHARGE!");
    }

    void BatteryCallback(const data_comm::battery::ConstPtr &msg)
    {
        //  充电完毕，关闭充电站
        if(battery_info.charge_state==true && msg->charge_state==false)
        {
            StopCharge();
        }

        battery_info= *msg;
    }

    void MqttCtrCallback(const mqtt_comm::mqtt_controls::ConstPtr &msg)
    {
        if(msg->msgType=="control" && msg->ctrlType=="Charge" && msg->value==0)
        {
            StopCharge();
        }
    }

public:
    TCharge()
    {
        nh=new ros::NodeHandle("~");
        charge_cmd_sub = nh->subscribe<std_msgs::String>("/charge_cmd", 10, &TCharge::ChargeCmdCallback, this);
        battery_sub = nh->subscribe<data_comm::battery>("/battery_info", 10, &TCharge::BatteryCallback, this);   
        control_sub = nh->subscribe<mqtt_comm::mqtt_controls>("/mqtt_controls", 10, &TCharge::MqttCtrCallback, this);
          
        charge_station_pub = nh->advertise<car_ctr::charge_station>("/charge_station", 10);
                
        string remote_ip="";
        nh->getParam("remote_ip",remote_ip);
        udp = new TUDP(9090);
        udp->AddRemoteIP(remote_ip, 9080);

        nodecheck=new TNodeCheck(nh, "node_rate");
	    nodecheck->Find("node_rate")->SetLimit(10);
    }
    
    void UDP_Proc() //  用于处理充电站上传的状态信息
    {
        if (udp->rec_count == 0)  return;
        udp->rec_count = 0;
    
        char recbuf[1000];
        strcpy(recbuf, udp->rec_buf);
        vector<string> strs = split(recbuf, " ");
        // ROS_INFO("%s", recbuf);
        
        if(strs.size()>=6)
        {
            car_ctr::charge_station msg;
            msg.door_open=atoi(strs[0].c_str())==1;
            msg.door_close=atoi(strs[1].c_str())==1;
            msg.pole_in=atoi(strs[2].c_str())==1;
            msg.pole_out=atoi(strs[3].c_str())==1;
            msg.charge_ready=atoi(strs[4].c_str())==1;
            charge_ready=msg.charge_ready;
            msg.ac220_charging=atoi(strs[5].c_str())==1;
            ac220_charging=msg.ac220_charging;
            charge_station_pub.publish(msg);

            if(!charge_ready) charge_check_tmr.Clear();
            else if(charge_check_tmr.GetValue()>5 && !ac220_charging)  
            {
                udp->Send("charge 1");
                ROS_INFO("START CHARGE!");
            }  

            nodecheck->Find("node_rate")->Beat();
        }
    }

    void run()
    {
        UDP_Proc();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Car_Charge");
    TCharge car_charge;

    ros::Rate rate(20);
    while (ros::ok())
    {
        car_charge.run();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};
