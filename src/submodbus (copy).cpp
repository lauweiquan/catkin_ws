#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include "examples_common.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>


int Arr[4];
int j=0;

void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array);

int main(int argc, char **argv)
{

	ros::init(argc, argv, "submodbus");

	ros::NodeHandle n;	

	ros::Subscriber sub = n.subscribe("modbus_wrapper/input", 1, arrayCallback);
	

for(;;){
for(j = 0; j < 4; j++)
	{
		printf("b4 enter %d ", Arr[j]);
	}
	printf("\n");
    franka::Robot robot(argv[1]);
    franka::Gripper gripper(argv[1]);
    setDefaultBehavior(robot);
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot.setFilters(1000,1000,1000,1000,1000)
    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{M_PI_2, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(1, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    gripper.move(80,0.1);
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

try 
{
ros::spinOnce();
while (Arr[0]==0)
{
  robot.stop();
  ros::spinOnce();
}
for(j = 0; j < 4; j++)
	{
		printf("b4 1st if else %d ", Arr[j]);
	}
	printf("\n");
if (Arr[0]==1 && Arr[1]==1) //prox=1,light=1
{

//move to cover
    
    std::array<double, 7> q_goal = {{1.25433,0.606685,-0.0010334,-1.85255,-0.00013017,2.43791,0.471117}};
    MotionGenerator motion_generator(1, q_goal);
    robot.control(motion_generator);

//move down to grab cover
    
    std::array<double, 7> q_goal1 = {{1.25262,0.750407,-0.000549145,-1.82856,-0.00012407,2.57698,0.468173}};
    MotionGenerator motion_generator1(0.5, q_goal1);
    robot.control(motion_generator1);

//grab cover

    
    // Check for the maximum grasping width.
    franka::GripperState gripper_state = gripper.readOnce();
  
    // Grasp the object.
    if (!gripper.grasp(0, 0.1, 20,0,0.5)) {
      std::cout << "Failed to grasp object." << std::endl;
      return -1;
    }
    // Wait 0s and check afterwards, if the object is still grasped.
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(0));
    gripper_state = gripper.readOnce();
    if (!gripper_state.is_grasped) {
      std::cout << "Object lost." << std::endl;
      return -1;
    }
    



//move up after grabbing

    std::array<double, 7> q_goal2 = {{1.25551,0.606791,0.00262695,-1.8559,-0.000706866,2.43168,0.469491}};
    MotionGenerator motion_generator2(1, q_goal2);
    robot.control(motion_generator2);

//move and rotate to moulding

    std::array<double, 7> q_goal3 = {{1.7125,0.60439,0.0233414,-1.89127,-0.000607447,2.45238,-0.69383}};
    MotionGenerator motion_generator3(1.3, q_goal3);
    robot.control(motion_generator3);

//move closer to moulding

    std::array<double, 7> q_goal8 = {{1.71851,0.676809,0.0220158,-1.89155,-0.00106561,2.52842,-0.693798}};
    MotionGenerator motion_generator8(0.5, q_goal8);
    robot.control(motion_generator8);
 }


else if (Arr[0]==1 && Arr[1]==0) //prox=1,light=0
{
  //move to cover
    std::array<double, 7> q_goal4 = {{1.25433,0.606685,-0.0010334,-1.85255,-0.00013017,2.43791,0.471117}};
    MotionGenerator motion_generator4(1, q_goal4);
    robot.control(motion_generator4);

//move down to grab cover
    
    std::array<double, 7> q_goal5 = {{1.25262,0.750407,-0.000549145,-1.82856,-0.00012407,2.57698,0.468173}};
    MotionGenerator motion_generator5(0.5, q_goal5);
    robot.control(motion_generator5);

//grab cover
    
    // Check for the maximum grasping width.
    franka::GripperState gripper_state = gripper.readOnce();
  
    // Grasp the object.
    if (!gripper.grasp(0, 0.1, 20,0,0.5)) {
      std::cout << "Failed to grasp object." << std::endl;
      return -1;
    }
    // Wait 0s and check afterwards, if the object is still grasped.
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(0));
    gripper_state = gripper.readOnce();
    if (!gripper_state.is_grasped) {
      std::cout << "Object lost." << std::endl;
      return -1;
    }
    

//move up after grabbing

    std::array<double, 7> q_goal6 = {{1.25551,0.606791,0.00262695,-1.8559,-0.000706866,2.43168,0.469491}};
    MotionGenerator motion_generator6(1, q_goal6);
    robot.control(motion_generator6);

//move and rotate to moulding

    std::array<double, 7> q_goal7 = {{1.7125,0.60439,0.0233414,-1.89127,-0.000607447,2.45238,2.45473}};
    MotionGenerator motion_generator7(1.1, q_goal7);
    robot.control(motion_generator7);

//move closer to moulding

    std::array<double, 7> q_goal8 = {{1.71851,0.676809,0.0220158,-1.89155,-0.00106561,2.52842,2.45473}};
    MotionGenerator motion_generator8(0.5, q_goal8);
    robot.control(motion_generator8); 
}


//gripper release
    std::cout << "Grasped object, will release it now." << std::endl;
    gripper.stop();
    gripper.move(80,0.1);


//move up from moulding after gripper release

    std::array<double, 7> q_goal9 = {{1.73946,0.521047,-0.00252165,-1.90048,-0.000723994,2.38443,-0.686917}};
    MotionGenerator motion_generator9(1, q_goal9);
    robot.control(motion_generator9);

//move to base (base's start of curve is 10.5cm on the ruler)

    std::array<double, 7> q_goal10 = {{2.1743,0.787139,-0.0182482,-1.59376,0.0249934,2.36077,-0.238332}};
    MotionGenerator motion_generator10(1.5, q_goal10);
    robot.control(motion_generator10);

ros::spinOnce();
while (Arr[2]==0)
{
  robot.stop();
  ros::spinOnce();
}
for(j = 0; j < 4; j++)
	{
		printf("b4 2nd if else %d ", Arr[j]);
	}
	printf("\n");
if (Arr[2]==1 && Arr[3]==1)
{
//move down to grab base

    std::array<double, 7> q_goal11 = {{2.17461,0.870496,-0.0198777,-1.58799,0.0248815,2.4503,-0.238377}};
    MotionGenerator motion_generator11(0.5, q_goal11);
    robot.control(motion_generator11);

//grab base

    // Check for the maximum grasping width.
    franka::GripperState gripper_state = gripper.readOnce();
  
    // Grasp the object.
    if (!gripper.grasp(0, 0.1, 20,0,0.5)) {
      std::cout << "Failed to grasp object." << std::endl;
      return -1;
    }
    // Wait 0s and check afterwards, if the object is still grasped.
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(0));
    gripper_state = gripper.readOnce();
    if (!gripper_state.is_grasped) {
      std::cout << "Object lost." << std::endl;
      return -1;
    }
    

//move up after grabbing base

    std::array<double, 7> q_goal12 = {{2.1743,0.787139,-0.0182482,-1.59376,0.0249934,2.36077,-0.238332}};
    MotionGenerator motion_generator12(1, q_goal12);
    robot.control(motion_generator12);

//move to mould

    std::array<double, 7> q_goal13 = {{1.81725,0.614627,-0.0688318,-1.93725,0.0209639,2.4798,-2.22059}};
    MotionGenerator motion_generator13(1.1, q_goal13);
    robot.control(motion_generator13);
 
//gripper release
    std::cout << "Grasped object, will release it now." << std::endl;
    gripper.stop();
    gripper.move(80,0.1);
    

//move to top of handle

    std::array<double, 7> q_goal14 = {{1.81209,0.668589,-0.0727104,-1.84347,0.0210647,2.43546,-2.22038}};
    MotionGenerator motion_generator14(0.5, q_goal14);
    robot.control(motion_generator14);

//push handle

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set the joint impedance.
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    // Set the collision behavior.
    std::array<double, 7> lower_torque_thresholds_nominal{
        {0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> upper_torque_thresholds_nominal{
        {100, 100, 100, 100, 100, 100, 100}};
    std::array<double, 7> lower_torque_thresholds_acceleration{
        {0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> upper_torque_thresholds_acceleration{
        {100, 100, 100, 100, 100, 100, 100}};
    std::array<double, 6> lower_force_thresholds_nominal{{0, 0, 0, 0, 0, 0}};
    std::array<double, 6> upper_force_thresholds_nominal{{100, 100, 100, 100, 100, 100}};
    std::array<double, 6> lower_force_thresholds_acceleration{{0, 0, 0, 0, 0, 0}};
    std::array<double, 6> upper_force_thresholds_acceleration{{100, 100, 100, 100, 100, 100}};
    robot.setCollisionBehavior(
        lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
        lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
        lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
        lower_force_thresholds_nominal, upper_force_thresholds_nominal);
    double time_max = 1.2;
    double v_max = 0.1;
    double time = 0.0;
    robot.control([=, &time](const franka::RobotState&,
                             franka::Duration period) -> franka::CartesianVelocities {
      time += period.toSec();
      double cycle = std::floor(pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
      double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));
    
      franka::CartesianVelocities output = {{0, 0.0, -v/2, 0.0, 0.0, 0.0}};
      if (time >= 2 * time_max) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
    });

//move to mould

    std::array<double, 7> q_goal15 = {{1.81674,0.609307,-0.0694842,-1.9373,0.0201987,2.48056,-2.2213}};
    MotionGenerator motion_generator15(1, q_goal15);
    robot.control(motion_generator15);


//move down to grab combined base and cover

    std::array<double, 7> q_goal16 = {{1.79969,0.68405,-0.0591957,-1.90347,0.0600053,2.54121,-2.27056}};
    MotionGenerator motion_generator16(1, q_goal16);
    robot.control(motion_generator16);

//grab combined
    
    // Check for the maximum grasping width.
    franka::GripperState gripper_state1 = gripper.readOnce();
  
    // Grasp the object.
    if (!gripper.grasp(0, 0.1, 20,0,0.5)) {
      std::cout << "Failed to grasp object." << std::endl;
      return -1;
    }
    // Wait 0s and check afterwards, if the object is still grasped.
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(0));
    gripper_state1 = gripper.readOnce();
    if (!gripper_state1.is_grasped) {
      std::cout << "Object lost." << std::endl;
      return -1;
    }

//move up from mould

    std::array<double, 7> q_goal17 = {{1.80885,0.47281,-0.065823,-1.90509,0.017731,2.32334,-2.24508}};
    MotionGenerator motion_generator17(1, q_goal17);
    robot.control(motion_generator17);

//place assembly at finished zone

    std::array<double, 7> q_goal18 = {{-0.91407,0.247921,0.1498,-2.46705,-0.22217,2.69779,-1.42168}};
    MotionGenerator motion_generator18(1, q_goal18);
    robot.control(motion_generator18);

//gripper release
    std::cout << "Grasped object, will release it now." << std::endl;
    gripper.stop();
    gripper.move(80,0.1);
    

//move up finished zone

    std::array<double, 7> q_goal19 = {{-0.905599,0.126754,0.163983,-2.46893,-0.193703,2.577,-1.42442}};
    MotionGenerator motion_generator19(1, q_goal19);
    robot.control(motion_generator19);
} 

else if (Arr[2]==1 && Arr[3]==0)
{
//move down to grab base

    std::array<double, 7> q_goal11 = {{2.17461,0.870496,-0.0198777,-1.58799,0.0248815,2.4503,-0.238377}};
    MotionGenerator motion_generator11(0.5, q_goal11);
    robot.control(motion_generator11);

//grab base

    // Check for the maximum grasping width.
    franka::GripperState gripper_state = gripper.readOnce();
  
    // Grasp the object.
    if (!gripper.grasp(0, 0.1, 20,0,0.5)) {
      std::cout << "Failed to grasp object." << std::endl;
      return -1;
    }
    // Wait 0s and check afterwards, if the object is still grasped.
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(0));
    gripper_state = gripper.readOnce();
    if (!gripper_state.is_grasped) {
      std::cout << "Object lost." << std::endl;
      return -1;
    }
    

//move up after grabbing base

    std::array<double, 7> q_goal12 = {{2.1743,0.787139,-0.0182482,-1.59376,0.0249934,2.36077,-0.238332}};
    MotionGenerator motion_generator12(1, q_goal12);
    robot.control(motion_generator12);

//move to mould

    std::array<double, 7> q_goal13 = {{1.81029,0.683761,-0.0722363,-1.80374,0.0243715,2.39198,-2.22059+M_PI}};
    MotionGenerator motion_generator13(1.1, q_goal13);
    robot.control(motion_generator13);

//gripper release
    std::cout << "Grasped object, will release it now." << std::endl;
    gripper.stop();
    gripper.move(80,0.1);


//push handle

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set the joint impedance.
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    // Set the collision behavior.
    std::array<double, 7> lower_torque_thresholds_nominal{
        {0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> upper_torque_thresholds_nominal{
        {100, 100, 100, 100, 100, 100, 100}};
    std::array<double, 7> lower_torque_thresholds_acceleration{
        {0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> upper_torque_thresholds_acceleration{
        {100, 100, 100, 100, 100, 100, 100}};
    std::array<double, 6> lower_force_thresholds_nominal{{0, 0, 0, 0, 0, 0}};
    std::array<double, 6> upper_force_thresholds_nominal{{100, 100, 100, 100, 100, 100}};
    std::array<double, 6> lower_force_thresholds_acceleration{{0, 0, 0, 0, 0, 0}};
    std::array<double, 6> upper_force_thresholds_acceleration{{100, 100, 100, 100, 100, 100}};
    robot.setCollisionBehavior(
        lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
        lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
        lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
        lower_force_thresholds_nominal, upper_force_thresholds_nominal);
    double time_max = 1.2;
    double v_max = 0.1;
    double time = 0.0;
    robot.control([=, &time](const franka::RobotState&,
                             franka::Duration period) -> franka::CartesianVelocities {
      time += period.toSec();
      double cycle = std::floor(pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
      double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));
    
      franka::CartesianVelocities output = {{0, 0.0, -v/2, 0.0, 0.0, 0.0}};
      if (time >= 2 * time_max) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
    });

//move to mould

    std::array<double, 7> q_goal15 = {{1.81029,0.683761,-0.0722363,-1.80374,0.0243715,2.39198,-2.22059+M_PI}};
    MotionGenerator motion_generator15(0.5, q_goal15);
    robot.control(motion_generator15);


//move down to grab combined base and cover

    std::array<double, 7> q_goal16 = {{1.79969,0.68405,-0.0591957,-1.90347,0.0600053,2.54121,-2.27056+M_PI}};
    MotionGenerator motion_generator16(1, q_goal16);
    robot.control(motion_generator16);

//grab combined
    
    // Check for the maximum grasping width.
    franka::GripperState gripper_state2 = gripper.readOnce();
  
    // Grasp the object.
    if (!gripper.grasp(0, 0.1, 20,0,0.5)) {
      std::cout << "Failed to grasp object." << std::endl;
      return -1;
    }
    // Wait 0s and check afterwards, if the object is still grasped.
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(0));
    gripper_state2 = gripper.readOnce();
    if (!gripper_state2.is_grasped) {
      std::cout << "Object lost." << std::endl;
      return -1;
    }

//move up from mould

    std::array<double, 7> q_goal17 = {{1.80094,0.553863,-0.0618479,-1.91678,0.0570503,2.41129,-2.22059+M_PI}};
    MotionGenerator motion_generator17(1, q_goal17);
    robot.control(motion_generator17);

//place assembly at finished zone

    std::array<double, 7> q_goal18 = {{-0.908813,0.22462,0.149498,-2.46261,-0.211834,2.66035,-1.42168+M_PI}};
    MotionGenerator motion_generator18(1, q_goal18);
    robot.control(motion_generator18);

//gripper release
    std::cout << "Grasped object, will release it now." << std::endl;
    gripper.stop();
    gripper.move(80,0.1);

//move up finished zone

    std::array<double, 7> q_goal19 = {{-0.905599,0.126754,0.163983,-2.46893,-0.193703,2.577,-1.42442+M_PI}};
    MotionGenerator motion_generator19(1, q_goal19);
    robot.control(motion_generator19);
 } 




//return home

    std::array<double, 7> q_goal20 = {{M_PI_2, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator20(1, q_goal20);
    robot.control(motion_generator20);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
}
  return 0;
}

void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{

	int i = 0;
	// print all the remaining numbers
	for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		Arr[i] = *it;
		i++;
	}

	return;
}
