/*
 * rosserial DVRK foot pedals
 */

#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

std_msgs::UInt16 gas_msg;
std_msgs::UInt16 brake_msg;
ros::Publisher pub_gas("/dvrk_carla/control/pedal_gas", &gas_msg);
ros::Publisher pub_brake("/dvrk_carla/control/pedal_brake", &brake_msg);


int offset_gas, offset_brake;
double freq = 10.0;
#define n_samples 100

void setup()
{

  pinMode (A0, INPUT);
  pinMode (A1, INPUT);
      
  nh.initNode();
  nh.advertise(pub_gas);
  nh.advertise(pub_brake);

  offset_gas = 0;
  offset_brake = 0;

  delay(1000);

  for (int i = 0; i < n_samples; i++)
  {
    offset_gas += analogRead(A0);
    offset_brake += analogRead(A1);
  }
  
  offset_gas = offset_gas / n_samples;
  // No idea what is going on, but add 20
  offset_gas += 20;
  offset_brake = offset_brake / n_samples;
  
}

long publisher_timer;

void loop()
{  
  if (millis() > publisher_timer) {
      // step 1: request reading from sensor 
      int gas, brake;

      gas = analogRead(A0) - offset_gas;
      gas = max(0, gas);
      brake = analogRead(A1) - offset_brake;
      brake = max(0, brake);
  
      gas_msg.data = gas;
      brake_msg.data = brake;
      
      pub_gas.publish(&gas_msg);
      pub_brake.publish(&brake_msg);  
  
      publisher_timer = millis() + (long)(1000/freq);
  }
  
  nh.spinOnce();
}
