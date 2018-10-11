// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include "examples_common.h"


int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

for(;;){
    int Arr[4];
    int prox;
    int light;
    prox=Arr[0];
    light=Arr[1];
    franka::Robot robot(argv[1]);
    franka::Gripper gripper(argv[1]);
    setDefaultBehavior(robot);
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{M_PI_2, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(1.3, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    gripper.move(80,0.1);
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

   
if (prox==1 && light==1) //prox=1,light=1
{

//move to cover
try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    
    std::array<double, 7> q_goal = {{1.25433,0.606685,-0.0010334,-1.85255,-0.00013017,2.43791,0.471117}};
    MotionGenerator motion_generator(1, q_goal);
    robot.control(motion_generator);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//move down to grab cover
try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    
    std::array<double, 7> q_goal = {{1.25262,0.750407,-0.000549145,-1.82856,-0.00012407,2.57698,0.468173}};
    MotionGenerator motion_generator(0.5, q_goal);
    robot.control(motion_generator);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//grab cover
 try {
    franka::Gripper robot(argv[1]);
    
    // Check for the maximum grasping width.
    franka::GripperState robot_state = robot.readOnce();
  
    // Grasp the object.
    if (!robot.grasp(0, 0.1, 20,0,0.5)) {
      std::cout << "Failed to grasp object." << std::endl;
      return -1;
    }
    // Wait 0s and check afterwards, if the object is still grasped.
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(0));
    robot_state = robot.readOnce();
    if (!robot_state.is_grasped) {
      std::cout << "Object lost." << std::endl;
      return -1;
    }
    
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }


//move up after grabbing
try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    std::array<double, 7> q_goal = {{1.25551,0.606791,0.00262695,-1.8559,-0.000706866,2.43168,0.469491}};
    MotionGenerator motion_generator(1, q_goal);
    robot.control(motion_generator);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//move and rotate to moulding
try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    std::array<double, 7> q_goal = {{1.7125,0.60439,0.0233414,-1.89127,-0.000607447,2.45238,-0.69383}};
    MotionGenerator motion_generator(1.3, q_goal);
    robot.control(motion_generator);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
}

else if (prox==1 && light==0) //prox=1,light=0
{
  //move to cover
try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    
    std::array<double, 7> q_goal = {{1.25433,0.606685,-0.0010334,-1.85255,-0.00013017,2.43791,0.471117}};
    MotionGenerator motion_generator(1, q_goal);
    robot.control(motion_generator);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//move down to grab cover
try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    
    std::array<double, 7> q_goal = {{1.25262,0.750407,-0.000549145,-1.82856,-0.00012407,2.57698,0.468173}};
    MotionGenerator motion_generator(0.5, q_goal);
    robot.control(motion_generator);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//grab cover
 try {
    franka::Gripper robot(argv[1]);
    
    // Check for the maximum grasping width.
    franka::GripperState robot_state = robot.readOnce();
  
    // Grasp the object.
    if (!robot.grasp(0, 0.1, 20,0,0.5)) {
      std::cout << "Failed to grasp object." << std::endl;
      return -1;
    }
    // Wait 0s and check afterwards, if the object is still grasped.
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(0));
    robot_state = robot.readOnce();
    if (!robot_state.is_grasped) {
      std::cout << "Object lost." << std::endl;
      return -1;
    }
    
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }


//move up after grabbing
try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    std::array<double, 7> q_goal = {{1.25551,0.606791,0.00262695,-1.8559,-0.000706866,2.43168,0.469491}};
    MotionGenerator motion_generator(1, q_goal);
    robot.control(motion_generator);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//move and rotate to moulding
try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    std::array<double, 7> q_goal = {{1.7125,0.60439,0.0233414,-1.89127,-0.000607447,2.45238,-0.69383}};
    MotionGenerator motion_generator(1.3, q_goal);
    robot.control(motion_generator);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
}

else
robot.stop();
 //end of prox=1 light=1 if loop



//move closer to moulding
try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    std::array<double, 7> q_goal = {{1.71851,0.676809,0.0220158,-1.89155,-0.00106561,2.52842,-0.693798}};
    MotionGenerator motion_generator(0.5, q_goal);
    robot.control(motion_generator);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//gripper release
try {
    franka::Gripper robot(argv[1]);
    std::cout << "Grasped object, will release it now." << std::endl;
    robot.stop();
    robot.move(80,0.1);
    
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//move up from moulding after gripper release
try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    std::array<double, 7> q_goal = {{1.73946,0.521047,-0.00252165,-1.90048,-0.000723994,2.38443,-0.686917}};
    MotionGenerator motion_generator(1, q_goal);
    robot.control(motion_generator);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//move to base (base's start of curve is 10.5cm on the ruler)
try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    std::array<double, 7> q_goal = {{2.1743,0.787139,-0.0182482,-1.59376,0.0249934,2.36077,-0.238332}};
    MotionGenerator motion_generator(1.5, q_goal);
    robot.control(motion_generator);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//if proxsen=1 && lightsen=1
//{

//move down to grab base
try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    std::array<double, 7> q_goal = {{2.17461,0.870496,-0.0198777,-1.58799,0.0248815,2.4503,-0.238377}};
    MotionGenerator motion_generator(0.5, q_goal);
    robot.control(motion_generator);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//grab base
 try {
    franka::Gripper robot(argv[1]);
    
    // Check for the maximum grasping width.
    franka::GripperState robot_state = robot.readOnce();
  
    // Grasp the object.
    if (!robot.grasp(0, 0.1, 20,0,0.5)) {
      std::cout << "Failed to grasp object." << std::endl;
      return -1;
    }
    // Wait 0s and check afterwards, if the object is still grasped.
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(0));
    robot_state = robot.readOnce();
    if (!robot_state.is_grasped) {
      std::cout << "Object lost." << std::endl;
      return -1;
    }
    
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//move up after grabbing base
try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    std::array<double, 7> q_goal = {{2.1743,0.787139,-0.0182482,-1.59376,0.0249934,2.36077,-0.238332}};
    MotionGenerator motion_generator(1, q_goal);
    robot.control(motion_generator);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//move to mould
try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    std::array<double, 7> q_goal = {{1.81725,0.614627,-0.0688318,-1.93725,0.0209639,2.4798,-2.22059}};
    MotionGenerator motion_generator(1.1, q_goal);
    robot.control(motion_generator);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//}

//gripper release
try {
    franka::Gripper robot(argv[1]);
    std::cout << "Grasped object, will release it now." << std::endl;
    robot.stop();
    robot.move(80,0.1);
    
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//move to top of handle
try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    std::array<double, 7> q_goal = {{1.81209,0.668589,-0.0727104,-1.84347,0.0210647,2.43546,-2.22038}};
    MotionGenerator motion_generator(0.5, q_goal);
    robot.control(motion_generator);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//push handle
try {
    franka::Robot robot(argv[1]);
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
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//move to mould
try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    std::array<double, 7> q_goal = {{1.81674,0.609307,-0.0694842,-1.9373,0.0201987,2.48056,-2.2213}};
    MotionGenerator motion_generator(1, q_goal);
    robot.control(motion_generator);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }


//move down to grab combined base and cover
try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    std::array<double, 7> q_goal = {{1.79969,0.68405,-0.0591957,-1.90347,0.0600053,2.54121,-2.27056}};
    MotionGenerator motion_generator(1, q_goal);
    robot.control(motion_generator);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//grab combined
 try {
    franka::Gripper robot(argv[1]);
    
    // Check for the maximum grasping width.
    franka::GripperState robot_state = robot.readOnce();
  
    // Grasp the object.
    if (!robot.grasp(0, 0.1, 20,0,0.5)) {
      std::cout << "Failed to grasp object." << std::endl;
      return -1;
    }
    // Wait 0s and check afterwards, if the object is still grasped.
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(0));
    robot_state = robot.readOnce();
    if (!robot_state.is_grasped) {
      std::cout << "Object lost." << std::endl;
      return -1;
    }
    
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//move up from mould
try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    std::array<double, 7> q_goal = {{1.80885,0.47281,-0.065823,-1.90509,0.017731,2.32334,-2.24508}};
    MotionGenerator motion_generator(1, q_goal);
    robot.control(motion_generator);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//place assembly at finished zone
try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    std::array<double, 7> q_goal = {{-0.91407,0.247921,0.1498,-2.46705,-0.22217,2.69779,-1.42168}};
    MotionGenerator motion_generator(1, q_goal);
    robot.control(motion_generator);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//gripper release
try {
    franka::Gripper robot(argv[1]);
    std::cout << "Grasped object, will release it now." << std::endl;
    robot.stop();
    robot.move(80,0.1);
    
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//move up finished zone
try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    std::array<double, 7> q_goal = {{-0.905599,0.126754,0.163983,-2.46893,-0.193703,2.577,-1.42442}};
    MotionGenerator motion_generator(1, q_goal);
    robot.control(motion_generator);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

//return home
try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    std::array<double, 7> q_goal = {{M_PI_2, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(1.1, q_goal);
    robot.control(motion_generator);
 } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
}
  return 0;
}
