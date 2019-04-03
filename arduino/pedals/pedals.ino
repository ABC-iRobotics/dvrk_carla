/*
 * rosserial DVRK foot pedals
 */

#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;


std_msgs::Float32 gas_msg;
ros::Publisher pub_gas("/dvrk_carla/control/pedal_gas", &gas_msg);


int offset1, offset2;


void setup()
{

  pinMode (A0, INPUT);
  pinMode (A1, INPUT);
      
  nh.initNode();
  nh.advertise(pub_gas);

  offset1 = 0;
  offset2 = 0;

  for (int i = 0; i < 10; i++)
  {
    offset1 += analogRead(A0);
    offset2 += analogRead(A1);
  }
  
  offset1 = offset1 / 10;
  offset2 = offset2 / 10;
  
}

long publisher_timer;

void loop()
{
  
  
  if (millis() > publisher_timer) {
  // step 1: request reading from sensor 
      int pedal1, pedal2;

      pedal1 = analogRead(A0) - offset1;
      pedal2 = analogRead(A1) - offset2;
  
   

      gas_msg.data = pedal1;
      pub_gas.publish(&gas_msg);
    
  
  publisher_timer = millis() + 1000;
  }
  
  nh.spinOnce();
}
