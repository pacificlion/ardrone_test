//
// main.cpp
// udp
//
// Created by Yang Chen on 1/11/23.
//
#include <chrono>
#include <sys/socket.h>
#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <memory.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <errno.h>
#include <stdlib.h>
#include <thread>
using namespace std::chrono;


class ARDrone{
  private:
  //UDP
   //std::chrono::microseconds start;
   std::string hostname;
   uint16_t port;
   int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
   sockaddr_in destination;

   //time
   milliseconds start;
   milliseconds now;
   int sequence_number=1;
   int loop_rate;
   int takeoffBits = (1 << 18) | (1 << 20) | (1 << 22) | (1 << 24) | (1 << 28) |(1<<9);
   int landingBits = (1 << 18) | (1 << 20) | (1 << 22) | (1 << 24) | (1 << 28);
   int emergencyBits = (1 << 18) | (1 << 20) | (1 << 22) | (1 << 24) | (1 << 28) |(1<<8);
  public:
    ARDrone(std::string host, int loop_rate){
      this->loop_rate = loop_rate;
      this->hostname = host;
      this->port = 5556;
      this->destination.sin_family = AF_INET;
      this->destination.sin_port = htons(this->port);
      this->destination.sin_addr.s_addr = inet_addr(this->hostname.c_str());
      this->sequence_number =1;
    }


    void takeoff(){
      this->start = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
      this->now = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
      long long diff = 0;
      char msg[100];
      while((this->now.count() - this->start.count()) < 4000){
          this->now = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
          snprintf(msg,100,"AT*REF=%d,%d\r",this->sequence_number,takeoffBits);
          //std::cout<<msg<<std::endl;
          this->sequence_number++;
          ssize_t n_bytes = ::sendto(this->sock, msg, 100, 0, reinterpret_cast<sockaddr*>(&this->destination), sizeof(this->destination));
          std::this_thread::sleep_for(std::chrono::milliseconds(this->loop_rate));
      }
      std::cout<<"taken off"<<std::endl;
    }

    void emergency(){

      this->start = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
      this->now = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
      long long diff = 0;
      char msg[100];
      while((this->now.count() - this->start.count()) < 4000){
          this->now = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
          snprintf(msg,100,"AT*REF=%d,%d\r",this->sequence_number,290717952);
          //std::cout<<msg<<std::endl;
          this->sequence_number++;
          ssize_t n_bytes = ::sendto(this->sock, msg, 100, 0, reinterpret_cast<sockaddr*>(&this->destination), sizeof(this->destination));
          std::this_thread::sleep_for(std::chrono::milliseconds(this->loop_rate));
      }
      std::cout<<"emergency done"<<std::endl;
      
    }

    void land(){
      this->start = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
      this->now = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
      long long diff = 0;
      char msg[100];
      while((this->now.count() - this->start.count()) < 4000){
          this->now = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
          snprintf(msg,100,"AT*REF=%d,%d\r",this->sequence_number,landingBits);
          //std::cout<<msg<<std::endl;
          this->sequence_number++;
          ssize_t n_bytes = ::sendto(this->sock, msg, 100, 0, reinterpret_cast<sockaddr*>(&this->destination), sizeof(this->destination));
          std::this_thread::sleep_for(std::chrono::milliseconds(this->loop_rate));
      }
      std::cout<<"landed"<<std::endl;
    }

    void zSpeed(float speed,int time){
      this->start = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
      this->now = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
      long long diff = 0;
      char msg[100];
      while((this->now.count() - this->start.count()) < time){
          this->now = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
          snprintf(msg,100,"AT*PCMD=%d,%d,%d,%d,%d,%d,%d\r",
           this->sequence_number,
           7,
           0,
           0,
           0,
           *(int*)(&speed),
           0);
           std::cout<<msg<<std::endl;
          this->sequence_number++;
          ssize_t n_bytes = ::sendto(this->sock, msg, 100, 0, reinterpret_cast<sockaddr*>(&this->destination), sizeof(this->destination));
          std::this_thread::sleep_for(std::chrono::milliseconds(this->loop_rate));
      }
    }

    void speed(float x,float y,float z, float a, float time){
      this->start = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
      this->now = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
      long long diff = 0;
      char msg[100];
      while((this->now.count() - this->start.count()) < time){
          this->now = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
          snprintf(msg,100,"AT*PCMD=%d,%d,%d,%d,%d,%d\r",
           this->sequence_number,
           7,
           *(int*)(&x),
           *(int*)(&y),
           *(int*)(&z),
           0);
           std::cout<<msg<<std::endl;
          this->sequence_number++;
          ssize_t n_bytes = ::sendto(this->sock, msg, 100, 0, reinterpret_cast<sockaddr*>(&this->destination), sizeof(this->destination));
          std::this_thread::sleep_for(std::chrono::milliseconds(this->loop_rate));
      }
    }

};

int main(int argc, const char * argv[]) {
  
  auto drone = ARDrone("192.168.1.1",30);
  drone.takeoff();


  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  
  /**
  for(int i=0; i< 50; i++){
    drone.zSpeed(1.0,200);
  }
  */

std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    for(int i=0; i< 100; i++){
    drone.zSpeed(-1.0,200);
  }
  
  
  //drone.speed(0.2,0,0,0,2000);
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  
  // drone.land();
  
  return 0;
}