/*
 * rosserial DVRK foot pedals
 * This code reads analog values from gas and brake pedals, applies an offset correction,
 * and publishes the values to ROS topics for use in a DVRK (Da Vinci Research Kit) simulation.
 */

#include <ros.h>
#include <std_msgs/UInt16.h>

// Initialize ROS node handle
ros::NodeHandle  nh;

// Define messages for gas and brake pedal values
std_msgs::UInt16 gas_msg;
std_msgs::UInt16 brake_msg;

// Define ROS publishers for the pedal values
ros::Publisher pub_gas("/dvrk_carla/control/pedal_gas", &gas_msg);
ros::Publisher pub_brake("/dvrk_carla/control/pedal_brake", &brake_msg);

// Variables to store pedal offset values
int offset_gas, offset_brake;
double freq = 10.0; // Frequency for publishing pedal values in Hz

#define n_samples 100 // Number of samples to calculate offset

void setup()
{
  // Configure analog input pins for gas and brake pedals
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
      
  // Initialize ROS node and advertise publishers
  nh.initNode();
  nh.advertise(pub_gas);
  nh.advertise(pub_brake);

  // Initialize offset values
  offset_gas = 0;
  offset_brake = 0;

  delay(1000); // Allow some time for stability before reading initial values

  // Compute offset by averaging multiple readings
  for (int i = 0; i < n_samples; i++)
  {
    offset_gas += analogRead(A0);
    offset_brake += analogRead(A1);
  }
  
  offset_gas = offset_gas / n_samples;
  offset_gas += 20; // Additional calibration adjustment for gas pedal
  offset_brake = offset_brake / n_samples;
}

// Timer to control publishing frequency
long publisher_timer;

void loop()
{  
  if (millis() > publisher_timer) {
      // Read pedal values and apply offset correction
      int gas, brake;

      gas = analogRead(A0) - offset_gas;
      gas = max(0, gas); // Ensure values are non-negative
      brake = analogRead(A1) - offset_brake;
      brake = max(0, brake);
  
      // Assign values to messages
      gas_msg.data = gas;
      brake_msg.data = brake;
      
      // Publish pedal values to ROS topics
      pub_gas.publish(&gas_msg);
      pub_brake.publish(&brake_msg);  
  
      // Set next publishing time based on frequency
      publisher_timer = millis() + (long)(1000 / freq);
  }
  
  // Handle ROS communication
  nh.spinOnce();
}
