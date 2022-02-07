#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <sl/Camera.hpp>
#include <string.h>
#include <chrono>
#include <math.h>

#include <stdlib.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <unistd.h>

using namespace std;

void* create_shared_mem(int *id, int *size){
    
    key_t key = ftok("imageMat", 0);
    *id = shmget(key, *size, 0666|IPC_CREAT);
    std::cout << "Pub - shmid 1 : " << *id << std::endl;
    if(*id<0) { printf("shmid() error"); return 0; }
    return (void*)shmat(*id, (void*)0, 0);
}

int main(int argc, char **argv)
{
  sl::Camera zed;

  // Set configuration parameters
  sl::InitParameters init_parameters;
  init_parameters.camera_resolution = sl::RESOLUTION::HD720; // Use HD1080 video mode
  init_parameters.camera_fps = 60; // Set fps
  init_parameters.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
  init_parameters.coordinate_units = sl::UNIT::METER;

  // Open the camera
  auto returned_state = zed.open(init_parameters);
  if (returned_state != sl::ERROR_CODE::SUCCESS) {
      cout << "Error " << returned_state << ", exit program." << endl;
      return EXIT_FAILURE;
  }

  // Initializing node
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter_1", 1000);
  // ros::Rate loop_rate(100);

  std_msgs::String msg;
  msg.data = "GO!!";

  // Creating shared memory block of image size(720p)
  int shmid;
  int block_size = 1280*4*720 + 20480*720;

  char (*mPtr)[720][1280][4] = (char(*)[720][1280][4])create_shared_mem(&shmid, &block_size);
  void *mPtr2 = (void*)((char*)mPtr + 5120*720 );
  sl::Mat image, point_cloud;
  // sl::Mat image(1280, 720, sl::MAT_TYPE::U8_C4, (sl::uchar1*)mPtr, 5120, sl::MEM::CPU);
  // sl::Mat point_cloud(1280, 720, sl::MAT_TYPE::F32_C4, ((sl::uchar1*)mPtr + 5120), 20480, sl::MEM::CPU); 
  
  // Defining clock for time measurements
  std::chrono::steady_clock::time_point begin, end;


  while (ros::ok()) 
  { 
    
    returned_state = zed.grab();
 
    if (returned_state == sl::ERROR_CODE::SUCCESS) {
      zed.retrieveImage(image, sl::VIEW::LEFT);
      zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZBGRA, sl::MEM::CPU);
      }
    
    
    std::memcpy((void*)mPtr, (void*)image.getPtr<sl::uchar4>(), 5120*720);
    std::memcpy((void*)mPtr2, (void*)point_cloud.getPtr<sl::uchar4>(), 20480*720);
    
    
    // Publishing data
    chatter_pub.publish(msg);

    // Time measurement and display
    end = std::chrono::steady_clock::now();
    std::cout << "Talker publish speed : " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    begin = std::chrono::steady_clock::now(); 

    ros::spinOnce();
    // loop_rate.sleep();
    
  }

  // Close the camera
  zed.close();

  return 0;
}
