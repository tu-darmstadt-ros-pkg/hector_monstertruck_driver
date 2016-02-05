#include <ros/ros.h>
#include <hector_monstertruck_driver/hector_monstertruck_driver.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_monstertruck_driver");

  HectorMonstertruckDriver monstertruck_driver;

  if(!monstertruck_driver.configure()){
      ROS_ERROR("Unable to configure Monstertruck Driver");
      exit(-1);
  }

  if(!monstertruck_driver.start()){
      ROS_ERROR("Unable to start Monstertruck Driver");
      exit(-1);
  }

  ros::Rate loop_rate(50);

  while (ros::ok()){
      ros::spinOnce();

      monstertruck_driver.spin();

      ros::spinOnce();

      loop_rate.sleep();
  }

  monstertruck_driver.stop();

  monstertruck_driver.cleanup();

  exit(0);
}
