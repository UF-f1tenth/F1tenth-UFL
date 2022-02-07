#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <sl/Camera.hpp>
#include <chrono>

#include <stdlib.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>

class matExt{
  private:
    ros::NodeHandle n;
    ros::Publisher chatter2_pub = n.advertise<std_msgs::String>("chatter_2", 1000);

    int shmid = 0;
    int block_size1 = 5120*720;
    int block_size2 = 1280*720*3*4;
    int (*ptr)[720][1280][3];

    // Image mat
    sl::Mat image;

    sl::uchar4 pixel;
    std_msgs::String msg;

    // // Defining clock for time measurements
    std::chrono::steady_clock::time_point begin, end;

    void* create_shared_mem(int *id, int *size){
      key_t key = ftok("imageMat", 1);
      *id = shmget(key, *size, 0644|IPC_CREAT);
      std::cout << "ext - shmid 1 : " << *id << std::endl;
      if(*id<0) { printf("shmid() error. Error code : %d",*id); return 0; }
      return (void*)shmat(*id, (void*)0, 0);
    }

    void* create_shared_mem2(int *id, int *size){
      key_t key2;
      if (-1 != open("/tmp/foo", O_CREAT, 0777)) {
          key2 = ftok("/tmp/foo", 0);
      } else {
          perror("open");
          exit(1);
      }

      *id = shmget(key2, *size, 0644|IPC_CREAT);
      std::cout << "ext - shmid 2 : " << *id << std::endl;
      if(*id<0) { printf("shmid() error. Error code : %d",*id); return 0; }
      return (void*)shmat(*id, (void*)0, 0);
    }


  public:

    // Constructor
    matExt(){
      msg.data = "GO!!";
      std::cout << "Created matExt Class with msg.data = " << msg.data << std::endl;
      }

    void initialize_mems(){
      void *temp = create_shared_mem(&shmid, &block_size1);
      size_t width=1280, height=720, step = 5120;
      sl::Mat m(width, height, sl::MAT_TYPE::U8_C4, (sl::uchar1*)temp, step, sl::MEM::CPU);
      
      image = m;
      ptr = (int(*)[720][1280][3])create_shared_mem2(&shmid, &block_size2);
      // std::cout << "HERE : " << (*ptr)[0][0][0] << std::endl;
    }

    void chatterCallback(const std_msgs::String::ConstPtr& data){
      // ROS_INFO("I heard: [%s]", data->data.c_str());

      // Extracting image data
      for(auto i=0;i<720;i++){
        for(auto j=0;j<1280;j++){
          image.getValue(j, i, &pixel);
          (*ptr)[i][j][0] = (int)pixel[0];
          (*ptr)[i][j][1] = (int)pixel[1];
          (*ptr)[i][j][2] = (int)pixel[2];
        }
      }
      
      // Publishing data
      chatter2_pub.publish(msg);
      
      // Time measurement
      end = std::chrono::steady_clock::now();
      std::cout << "Ext publish speed : " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
      begin = std::chrono::steady_clock::now();
    }

    void run(){
      ros::Subscriber sub = n.subscribe("chatter_1", 1000, &matExt::chatterCallback, this);
      ros::spin();
    }

// end of class
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Mat_Extractor");
  matExt myClass;
  myClass.initialize_mems();
  myClass.run();
  return 0;
}

