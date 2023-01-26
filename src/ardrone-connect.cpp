//
// main.cpp
// udp
//
// Created by Yang Chen on 1/11/23.
//
#include "ros/ros.h"
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
#include "geometry_msgs/TransformStamped.h"
using namespace std::chrono;



ros::Subscriber vicon_sub;
geometry_msgs::TransformStamped vicon_statedata;  


void ViconStateCallback(const geometry_msgs::TransformStamped::ConstPtr & vicon_state_message); //callback function for vicon data of ardrone1

void ViconStateCallback(const geometry_msgs::TransformStamped::ConstPtr & vicon_state_message)
{
	vicon_statedata.transform.translation.x	= 	vicon_state_message->transform.translation.x;
	vicon_statedata.transform.translation.y	= 	vicon_state_message->transform.translation.y;
	vicon_statedata.transform.translation.z	= 	vicon_state_message->transform.translation.z;
	
	vicon_statedata.transform.rotation.x	= 	vicon_state_message->transform.rotation.x;//quaternion orientation x,y,z,w
	vicon_statedata.transform.rotation.y	= 	vicon_state_message->transform.rotation.y;
	vicon_statedata.transform.rotation.z	= 	vicon_state_message->transform.rotation.z;
	vicon_statedata.transform.rotation.w	= 	vicon_state_message->transform.rotation.w;
	
}
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

    void speed(float &x,float &y,float &z,  float time){
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
void xy_pid(double x,double y,double z,ARDrone drone){
	ros::Rate loop_rate(250);
	float kpx = 0.1, kdx = 0,kpy = 0.1, kdy = 0,kpz = 0.4, kdz = 0.1;
	float x0=0,y0=0,z0=0,qx0,qy0,qz0,qw0,theta0,phi0,psi0,ex=0.0,ey=0.0,dx=0.0,dy=0.0,ox=0.0,oy=0.0,ez=0.0,dz=0.0,oz=0.0;
	float xi,yi,zi,qxi,qyi,qzi,qwi,thetai,phii,psii;

	
	int ctr = 0;
	 
	while(ctr<5){
		xi = vicon_statedata.transform.translation.x, yi = vicon_statedata.transform.translation.y, zi = vicon_statedata.transform.translation.z;
		qxi=vicon_statedata.transform.rotation.x, qyi=vicon_statedata.transform.rotation.y, qzi=vicon_statedata.transform.rotation.z, qw0=vicon_statedata.transform.rotation.w;
		thetai = atan2(+2.0 * (qwi*qxi + qyi*qzi) , +1.0 - 2.0 * (qxi*qxi + qyi*qyi));
		phii = asin(+2.0 * (qwi*qyi - qzi*qxi)); 
		psii = 0;
		ctr = ctr+1;

		ros::spinOnce(); 
		loop_rate.sleep();
	}
	//ctr = 0;
	while(ros::ok()){
		dz = vicon_statedata.transform.translation.z - z0;
		dx = vicon_statedata.transform.translation.x - x0;
		dy = vicon_statedata.transform.translation.y - y0;
		//dz = vicon_statedata.transform.translation.z - z0;
		x0 = vicon_statedata.transform.translation.x-xi, y0 = vicon_statedata.transform.translation.y-yi, z0 = vicon_statedata.transform.translation.z - zi;
		qx0=vicon_statedata.transform.rotation.x, qy0=vicon_statedata.transform.rotation.y, qz0=vicon_statedata.transform.rotation.z, qw0=vicon_statedata.transform.rotation.w;
		theta0 = atan2(+2.0 * (qw0*qx0 + qy0*qz0) , +1.0 - 2.0 * (qx0*qx0 + qy0*qy0));
		phi0 = asin(+2.0 * (qw0*qy0 - qz0*qx0)); 
		psi0 = 0;
		//ctr = ctr+1;
		//print_vicon_coordinates();
		
		/*ez = height - z0;
		oz = kp * ez + kd * dz; */
		ex = x - x0;
		ox = kpx * ex + kdx * dx; 
		ey = y - y0;
		oy = kpy * ey + kdy * dy; 
		ez = z - z0;
		oz = kpz * ez + kdz * dz; 
		ox = ox > 1 ? 1 : ox;
		ox = ox < -1 ? -1 : ox;
		oy = oy > 1 ? 1: oy;
		oy = oy  <-1 ? -1 : oy;
		std::cout<<"error: " << ex<<","<<ey<<","<<ez<<","<<dx<<","<<dy<<","<<dz<<","<<ox<<","<<oy<<","<<oz<<std::endl;
		drone.speed(ox,oy,oz,0.2);
		// cout<<x0<<","<<y0<<","<<z0<<","<<qx0<<","<<qy0<<","<<qz0<<endl;
		ros::spinOnce(); 
		loop_rate.sleep();
	}
	std::cout<<"before landing\n";
  drone.land();
	//move(0,0,0,0,0,0);
	//land();
	


}
int main(int argc, char ** argv) {
  
  ros::init(argc, argv, "waypoint_nav");
  ros::NodeHandle n; 
  std::cout<<"running from UDP way point\n";
  auto drone = ARDrone("192.168.1.211",30);
  drone.takeoff();
  vicon_sub = n.subscribe("/vicon/ardrone1/cf1",200,ViconStateCallback);
	

//   std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  
//   /**
//   for(int i=0; i< 50; i++){
//     drone.zSpeed(1.0,200);
//   }
//   */

// std::this_thread::sleep_for(std::chrono::milliseconds(2000));
//     for(int i=0; i< 100; i++){
//     drone.zSpeed(-1.0,200);
//   }
  
  
//   //drone.speed(0.2,0,0,0,2000);
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  
  drone.land();
  
  return 0;
}