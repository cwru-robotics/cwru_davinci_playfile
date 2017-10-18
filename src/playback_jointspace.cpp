/******************************************************************************
**
**   cwru davinci Joint Space playfile reader for playing a robotic motion on a dvrk.
**   Copyright (C) 2017  Tom Shkurti, Russell Jackson, & Wyatt Newman
**   Case Western Reserve University
**
**   This program is free software: you can redistribute it and/or modify
**   it under the terms of the GNU General Public License as published by
**   the Free Software Foundation, either version 3 of the License, or
**   (at your option) any later version.
**
**   This program is distributed in the hope that it will be useful,
**   but WITHOUT ANY WARRANTY; without even the implied warranty of
**   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**   GNU General Public License for more details.
**
**   You should have received a copy of the GNU General Public License
**   along with this program.  If not, see <http://www.gnu.org/licenses/>.
**
******************************************************************************/
#include <string>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <cwru_davinci_control/psm_controller.h>
#include <cwru_davinci_playfile/playfile_format_joint.h>

int main(int argc, char **argv)
{
  // Locate and load the input file.
  std::vector<std::vector<double> > data = std::vector<std::vector<double> >();
  if (argc == 2)
  {
    joint_format::read_file(std::string(argv[1]), data);
  }
  else if (argc == 3)
  {
    joint_format::read_file(std::string(argv[1]), std::string(argv[2]), data);
  }
  else
  {
    ROS_ERROR("Missing file or package location. Aborting.");
    return 0;
  }

  // Set up our node.
  ros::init(argc, argv, "playback_joint");
  ros::NodeHandle nh;
  psm_controller psm_1(1, nh);
  psm_controller psm_2(2, nh);

  // Grab the current position, which will be turned into the first point in the trajectory
  std::vector<trajectory_msgs::JointTrajectoryPoint>
    joint_trajectories_1(std::vector<trajectory_msgs::JointTrajectoryPoint>(data.size()));
  std::vector<trajectory_msgs::JointTrajectoryPoint>
    joint_trajectories_2(std::vector<trajectory_msgs::JointTrajectoryPoint>(data.size()));


  // Fill in the other positions in the trajectory, creating velocity goals between them.
  for (int n  = 0; n < data.size(); n++)
  {
    joint_trajectories_1[n].positions.clear();
    joint_trajectories_1[n].positions.insert(
      joint_trajectories_1[n].positions.end(), data[n].begin(), data[n].begin() + 7);

    joint_trajectories_1[n].velocities.clear();
    joint_trajectories_1[n].effort.clear();
    if (data[n].size() == 15)
    {
      joint_trajectories_1[n + 1].velocities.resize(7, 0.0);
      joint_trajectories_1[n + 1].effort.resize(7, 0.0);
    }
    else
    {
      joint_trajectories_1[n].velocities.insert(
        joint_trajectories_1[n].velocities.end(), data[n].begin() + 7, data[n].begin() + 14);

      joint_trajectories_1[n].effort.insert(
        joint_trajectories_1[n].effort.end(), data[n].begin() + 14, data[n].begin() + 21);
    }

    // allocated time information
    if (data[n].size() == 15)
    {
      joint_trajectories_1[n].time_from_start = ros::Duration(data[n][14]);
      joint_trajectories_2[n].time_from_start = ros::Duration(data[n][14]);
    }
    else
    {
      joint_trajectories_1[n].time_from_start = ros::Duration(data[n][42]);
      joint_trajectories_2[n].time_from_start = ros::Duration(data[n][42]);
    }


    // TODO(tes77) redefine the velocity model generation.
    // float dt = joint_trajectories_1[n].time_from_start.toSec() - joint_trajectories_1[n + 1].time_from_start.toSec();

    /*joint_trajectories_1[n + 1].velocities = {
      (joint_trajectories_1[n].positions[0] - joint_trajectories_1[n + 1].positions[0]) / dt,
      (joint_trajectories_1[n].positions[1] - joint_trajectories_1[n + 1].positions[1]) / dt,
      (joint_trajectories_1[n].positions[2] - joint_trajectories_1[n + 1].positions[2]) / dt, 
      (joint_trajectories_1[n].positions[3] - joint_trajectories_1[n + 1].positions[3]) / dt, 
      (joint_trajectories_1[n].positions[4] - joint_trajectories_1[n + 1].positions[4]) / dt, 
      (joint_trajectories_1[n].positions[5] - joint_trajectories_1[n + 1].positions[5]) / dt, 
      (joint_trajectories_1[n].positions[6] - joint_trajectories_1[n + 1].positions[6]) / dt
    };*/

    /*for(int m = 0; m < 7; m++){
      if(joint_trajectories_1[n].positions[m] == joint_trajectories_1[n + 1].positions[m]){
        joint_trajectories_1[n + 1].velocities[m] = 0.0;
      }
    }*/
    joint_trajectories_2[n].velocities.clear();
    joint_trajectories_2[n].effort.clear();
    joint_trajectories_2[n].positions.clear();
    if (data[n].size() == 15)
    {
      joint_trajectories_2[n].positions.insert(
        joint_trajectories_2[n].positions.end(), data[n].begin() + 7, data[n].begin() + 14);

      joint_trajectories_2[n].velocities.resize(7, 0.0);
      joint_trajectories_2[n].effort.resize(7, 0.0);
    }
    else
    {
      joint_trajectories_2[n].positions.insert(
        joint_trajectories_2[n].positions.end(), data[n].begin() + 21, data[n].begin() + 28);

      joint_trajectories_2[n].velocities.insert(
        joint_trajectories_2[n].velocities.end(), data[n].begin() + 28, data[n].begin() + 35);

      joint_trajectories_2[n].effort.insert(
        joint_trajectories_2[n].effort.end(), data[n].begin() + 35, data[n].begin() + 42);
    }
  }

  /*std::cout << "PSM 1 Joint commands:\n";
  for (int i = 0; i < joint_trajectories_1.size(); i++)
  {
    std::cout << "At time: ";
    std::cout << joint_trajectories_1[i].time_from_start;
    std::cout << "\n\t";
    for (int j = 0; j < 7; j++)
    {
      std::cout << joint_trajectories_1[i].positions[j] << " ";
      std::cout << joint_trajectories_1[i].velocities[j] << " ";
      std::cout << joint_trajectories_1[i].effort[j];
    }
    std::cout << "\n";
  }

  std::cout << "PSM 2 Joint commands:\n";
  for (int i = 0; i < joint_trajectories_2.size(); i++)
  {
    std::cout << "At time: ";
    std::cout << joint_trajectories_2[i].time_from_start;
    std::cout << "\n\t";
    for (int j = 0; j < 7; j++)
    {
      std::cout << joint_trajectories_2[i].positions[j] << " ";
      std::cout << joint_trajectories_2[i].velocities[j] << " ";
      std::cout << joint_trajectories_2[i].effort[j];
    }
    std::cout << "\n";
  } */


  psm_1.move_psm(joint_trajectories_1[0]);
  if (psm_1.wait_psm())
  {
    ROS_INFO("Finished setting up PSM 1");
  }
  else
  {
    ROS_WARN("PSM 1 not set up");
  }
  psm_2.move_psm(joint_trajectories_2[0]);
  if (psm_2.wait_psm())
  {
    ROS_INFO("Finished setting up PSM 2");
  }
  else
  {
    ROS_WARN("PSM 2 not set up");
  }

  psm_1.move_psm(joint_trajectories_1);
  psm_2.move_psm(joint_trajectories_2);

  return 0;
}
