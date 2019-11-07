/*
 * rosserial DVRK foot pedals
 */

#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;


std_msgs::Float32 gas_msg;
std_msgs::Float32 brake_msg;
ros::Publisher pub_gas("/dvrk_carla/control/pedal_gas", &gas_msg);
ros::Publisher pub_brake("/dvrk_carla/control/pedal_brake", &brake_msg);


double offset_gas, offset_brake;
double freq = 10.0;


void setup()
{

  pinMode (A0, INPUT);
  pinMode (A1, INPUT);
      
  nh.initNode();
  nh.advertise(pub_gas);

  offset_gas = 0;
  offset_brake = 0;

  for (int i = 0; i < 10; i++)
  {
    offset_gas += analogRead(A0);
    offset_brake += analogRead(A1);
  }
  
  offset_gas = offset_gas / 10.0;
  offset_brake = offset_brake / 10.0;
  
}

long publisher_timer;

void loop()
{
  
  
  if (millis() > publisher_timer) {
  // step 1: request reading from sensor 
      int gas, brake;

      gas = analogRead(A0) - offset_gas;
      brake = analogRead(A1) - offset_brake;
  
   

      gas_msg.data = gas;
      brake_msg.data = brake;
      
      pub_gas.publish(&gas_msg);
      pub_brake.publish(&brake_msg);
    
  
  
  publisher_timer = millis() + (long)(1000/freq);
  }
  
  nh.spinOnce();
}
