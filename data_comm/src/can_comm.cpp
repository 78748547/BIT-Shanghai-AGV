#include "ros/ros.h"
#include <sstream>
#include <common/can.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <common/public.h>
#include <data_comm/battery.h>

TCan *can;
TNodeCheck *nodecheck;

ros::NodeHandle *nh;
ros::Publisher battery_pub;

void ProcCanMsg()
{
	// ROS_INFO("size=%d\n", can->rec_frames.size());
    static data_comm::battery battery_info;

	for (int i = 0; i < can->rec_frames.size(); i++)
	{
		can_frame *frame = &can->rec_frames[i];
        frame->can_id&=0xFFFFFF;
		if(frame->can_id==0xFF28F4)  
		{
		    // for(int i=0;i<8;i++)  printf("%02x ", frame->data[i]);
 		    // printf("\n");

			battery_info.voltage=0.1*(frame->data[4]+frame->data[5]*256);
			battery_info.current=0.1*(frame->data[2]+frame->data[3]*256-5000);
			battery_info.charge_state=(frame->data[0] & 0x02)>0;
			battery_info.SOC=frame->data[1];
			battery_pub.publish(battery_info);

            nodecheck->Find("node_rate")->Beat();			
        } 
        else if(frame->can_id==0xFE28F4)
		{
			battery_info.temperature=frame->data[4]-40;
			// ROS_INFO("%d",battery_info.temperature);
		}

		// ROS_INFO("%x", frame->can_id);
		
		frame->can_id = 0;
	}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "plus1");

	nh = new ros::NodeHandle("~");

	nodecheck = new TNodeCheck(nh, "node_rate");
	nodecheck->Find("node_rate")->SetLimit(10);
	int can_ch = 1;
	can = InitCan(can_ch, 250000);
	
	battery_pub = nh->advertise<data_comm::battery>("/battery_info", 10);

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ProcCanMsg();
     // if(nodecheck->Find("node_rate")->value<5)  ROS_ERROR_STREAM_THROTTLE(1, "CAN comm error!");

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}