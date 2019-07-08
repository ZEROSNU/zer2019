#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "std_msgs/Header.h"
#include "core_msgs/Control.h"
#include <sstream>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include "async_comm/serial.h"

std::string password = "";

std::string port = "/dev/ttyUSB0";
std::string command = "echo " + password + " | sudo -S chmod 777 " + port;

uint baudrate=115200;
async_comm::Serial s(port, baudrate);

uint8_t alive;

void ctrlmsg_to_data(/*std_msgs::Header header,*/ int16_t control_mode, bool is_auto, bool estop, int8_t gear, int16_t brake, float speed, float steer, int64_t encoder){
  uint8_t data[14];
  data[0] = (uint8_t)0x53;
  data[1] = (uint8_t)0x54;
  data[2] = (uint8_t)0x58;
  data[3] = (uint8_t)is_auto;
  data[4] = (uint8_t)estop;
  data[5] = (uint8_t)gear;
  int speed_int = (int)round(speed*10);
  data[6] = (uint8_t)(speed_int >> 8);
  data[7] = (uint8_t)(speed_int & 255);
  int steer_int = (int)round(steer*71);
  data[8] = (uint8_t)(steer_int >> 8);
  data[9] = (uint8_t)(steer_int & 255);
  data[10] = (uint8_t)brake;
  data[11] = alive;
  data[12] = (uint8_t)0x0d;
  data[13] = (uint8_t)0x0a;
  s.send_bytes(data, 14);
}

void input_callback (const uint8_t* data, size_t buff) {
  static int curr_byte = 0, curr_data[18];

  for(int i=0;i<buff;i++){
    curr_data[curr_byte] = data[i];
    curr_byte++;
  }
  if(curr_byte == 18){
    printf("0x%.2x ",curr_data[0]);
    printf("0x%.2x ",curr_data[1]);
    printf("0x%.2x ",curr_data[2]);
    printf("0x%.2x ",curr_data[3]);
    printf("0x%.2x ",curr_data[4]);
    printf("%4d ",curr_data[5]);
    printf("%6d ",curr_data[6]*256 + curr_data[7]);
    printf("%3d %3d ",curr_data[8], curr_data[9]);
    printf("%4d ",curr_data[10]);
    printf("%10d ",(signed int)(curr_data[11]<<24 + curr_data[12]<<16 + curr_data[13]<<8 + curr_data[14]));
    printf("%4d ",curr_data[15]);
    printf("0x%.2x ",curr_data[16]);
    printf("0x%.2x ",curr_data[17]);
    printf("\n");

    bool is_auto = (bool) curr_data[3];
    bool estop = (bool) curr_data[4];
    int8_t gear = (int8_t) curr_data[5];
    int16_t brake = (int16_t) curr_data[10];
    float speed = (float)(curr_data[6]*256 + curr_data[7]) / 10.;
    float steer = (float)(curr_data[8]*256 + curr_data[9]) / 71.;
    int64_t encoder = (signed int)(curr_data[11]<<24 + curr_data[12]<<16 + curr_data[13]<<8 + curr_data[14]);
    alive = curr_data[15];

    std_msgs::Header h();

    ctrlmsg_to_data(/*h,*/ 0, 1, 0, gear, brake, 0.1, steer, encoder);

    curr_byte = 0;
  }
}
int main ( int argc, char **argv )
{
  freopen("output.txt","w",stdout);
  printf("  S    T    X  AorM Estp gear  speed   steer  brk     enc     alv etx0 etx1\n");

  std::system(command.c_str());

  s.register_receive_callback(input_callback);

  bool tf = s.init();

  if (tf == false) {
    std::cout << "not initiated" << "\n";
    return 0;
  }
  ros::init(argc, argv, "serial_communicator");

  ros::NodeHandle n;

  ros::Publisher pub_cont = n.advertise<core_msgs::Control> ("/Control", 1000);

  ros::Rate loop_rate(10);

  int count =0;
  while (ros::ok()) {
    core_msgs::Control msg;
    pub_cont.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
