#include "node_check.h"

dialog_node_check::dialog_node_check(QWidget *parent) :
    QDialog(parent),
    ui(new Ui_Dialog_NodeCheck)
{
    ui->setupUi(this);
    nh=new ros::NodeHandle("~");

    qtmr.start(100);
    connect(&qtmr, SIGNAL(timeout()), this, SLOT(qtmrfunc()));
}

void dialog_node_check::update_rate(string param, QLabel *lab)
{
    char buf[200];
    float value = 0;
    nh->getParam(param, value);
    sprintf(buf, "%.1f", value);
    lab->setText(buf);
}

void dialog_node_check::qtmrfunc()
{
    update_rate("/gps_pro/check/node_rate", ui->label_slam_hb);
    update_rate("/cloud_tf/check/rslidar_rate", ui->label_lidar_hb);
    update_rate("/read_if_cam/check/node_rate", ui->label_if_hb);
    update_rate("/cap_image1/check/node_rate", ui->label_ccd_hb);
    update_rate("/car_ctr/check/node_rate", ui->label_carctr_hb);
    update_rate("/car_charge/check/node_rate", ui->label_station_hb);
    update_rate("/can_comm/check/node_rate", ui->label_battery_hb);
    update_rate("/turntable/check/node_rate", ui->label_turntable_hb);
    
}

dialog_node_check::~dialog_node_check()
{
    delete ui;
}
