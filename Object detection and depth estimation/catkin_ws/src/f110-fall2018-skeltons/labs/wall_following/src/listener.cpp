#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <sl/Camera.hpp>
#include <boost/make_shared.hpp>

#include <stdlib.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <unistd.h>

using namespace std;

typedef int myType[720][1280][3];
myType *ptr;

void* create_shared_mem(int *id, int *size){
    
    key_t key = ftok("leftImage", 0);
    *id = shmget(key, *size, 0644|IPC_CREAT);
    if(*id<0) { printf("shmid() error. Error code : %d",*id); return 0; }
    return (void*)shmat(*id,NULL,0);
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{  
  // ROS_INFO( msg->data.c_str());
  cout <<  " From Sub : " << (*ptr)[300][600][0] << endl; 
}

int main(int argc, char **argv)
{
  int shmid = 0;
  int block_size = 1280*720*4*3;

  ptr = (myType*)create_shared_mem(&shmid, &block_size);

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  
  ros::spin();
  
  return 0;
}
