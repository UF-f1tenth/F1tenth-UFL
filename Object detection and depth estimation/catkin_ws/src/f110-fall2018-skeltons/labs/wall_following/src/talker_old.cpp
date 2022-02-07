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
    
    key_t key = ftok("leftImage", 0);
    *id = shmget(key, *size, 0666|IPC_CREAT);
    if(*id<0) { printf("shmid() error"); return 0; }
    return (void*)shmat(*id, NULL, 0);
}

int main(int argc, char **argv)
{
  sl::Camera zed;

  // Set configuration parameters
  sl::InitParameters init_parameters;
  init_parameters.camera_resolution = sl::RESOLUTION::HD720; // Use HD1080 video mode
  init_parameters.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
  init_parameters.coordinate_units = sl::UNIT::METER;
  init_parameters.camera_fps = 60; // Set fps

  // Open the camera
  auto returned_state = zed.open(init_parameters);
  if (returned_state != sl::ERROR_CODE::SUCCESS) {
      cout << "Error " << returned_state << ", exit program." << endl;
      return EXIT_FAILURE;
  }

  // Initializing node
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  // ros::Rate loop_rate(100);
  
  sl::Mat image, point_cloud;
  std_msgs::String msg;
  msg.data = "GO!!";

  sl::uchar4 leftCenter;
  sl::float4 point3D; 
  static int image_int[720][1280][4];

  // Creating shared memory block of image size(720p)
  int shmid;
  int block_size = sizeof(image_int);

  // Creating custom pointer
  typedef int myType[720][1280][4];
  myType *mPtr = (myType*)create_shared_mem(&shmid, &block_size); 
  
  // Defining clock for time measurements
  std::chrono::steady_clock::time_point begin, end;
  
  float x, y, z;

  while (ros::ok()) 
  {   
      begin = std::chrono::steady_clock::now();
      
      returned_state = zed.grab();
      if (returned_state == sl::ERROR_CODE::SUCCESS) {
      zed.retrieveImage(image, sl::VIEW::LEFT);
      zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);
      }
   
    
    //Retrieving data from sl::Mat to array
    for(int i=0;i<720;i++){
        for(int j=0;j<1280;j++){
        
            // Unpacking image
            image.getValue<sl::uchar4>(j, i, &leftCenter);
            (*mPtr)[i][j][0] = (int)leftCenter[0];
            (*mPtr)[i][j][1] = (int)leftCenter[1];
            (*mPtr)[i][j][2] = (int)leftCenter[2];
            
            // Unpacking depth measure
            point_cloud.getValue(j, i, &point3D);
            x = point3D.x;y = point3D.y;z = point3D.z;
            (*mPtr)[i][j][3] = 100*sqrt(x*x + y*y + z*z);
      }  
    }
    
    //std::cout << "Distance : " << 100*sqrt(point3D.x*point3D.x + point3D.y*point3D.y + point3D.z*point3D.z) << std::endl;
    //std::memcpy(mPtr, &image_int, block_size);
    

    // Publishing data
    chatter_pub.publish(msg);
    // ROS_INFO("I sent : " + msg.data.c_str());
    end = std::chrono::steady_clock::now();
    // Time measurement and display
    std::cout << "Publish speed : " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    // std::cout << "From pub : " << (*mPtr)[300][600][0] << std::endl;
    ros::spinOnce();
    // loop_rate.sleep();
    
  }

  // Close the camera
  zed.close();

  return 0;
}
