/*
 * Ejemplo para publicar un entero que se incrementa en uno cada 500 ms
 */
#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;

std_msgs::Int32 msgCounter;
ros::Publisher pubCounter("/counter", &msgCounter);

void setup()
{
  nh.initNode();
  nh.advertise(pubCounter);
  msgCounter.data = 0;
}

void loop()
{
  msgCounter.data++;
  pubCounter.publish( &msgCounter );
  nh.spinOnce();
  delay(500);
}
