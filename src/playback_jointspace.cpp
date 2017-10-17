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

  // Grab the current position, which will be turned into the first point in the trajectory.
  sensor_msgs::JointState js_1;
  sensor_msgs::JointState js_2;
  psm_1.get_fresh_psm_state(js_1);
  psm_2.get_fresh_psm_state(js_2);

  std::vector<trajectory_msgs::JointTrajectoryPoint>
    joint_trajectories_1(std::vector<trajectory_msgs::JointTrajectoryPoint>(data.size() + 1));
  std::vector<trajectory_msgs::JointTrajectoryPoint>
    joint_trajectories_2(std::vector<trajectory_msgs::JointTrajectoryPoint>(data.size() + 1));

  joint_trajectories_1[0].time_from_start = ros::Duration(0.0);
  joint_trajectories_1[0].velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  joint_trajectories_1[0].positions = js_1.position;

  joint_trajectories_2[0].time_from_start = ros::Duration(0.0);
  joint_trajectories_2[0].velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  joint_trajectories_2[0].positions = js_2.position;



  // Fill in the other positions in the trajectory, creating velocity goals between them.
  for (int n  = 0; n < data.size(); n++)
  {
    joint_trajectories_1[n+1].time_from_start = ros::Duration(data[n][14] + 0.0);
    joint_trajectories_1[n+1].positions =
    {
      data[n][0],
      data[n][1],
      data[n][2],
      data[n][3],
      data[n][4],
      data[n][5],
      data[n][6]
    };

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
    joint_trajectories_1[n + 1].velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    /*for(int m = 0; m < 7; m++){
      if(joint_trajectories_1[n].positions[m] == joint_trajectories_1[n + 1].positions[m]){
        joint_trajectories_1[n + 1].velocities[m] = 0.0;
      }
    }*/

    joint_trajectories_2[n+1].time_from_start = ros::Duration(data[n][14] + 0.0);
    joint_trajectories_2[n+1].positions =
    {
      data[n][7 ],
      data[n][8 ],
      data[n][9 ],
      data[n][10],
      data[n][11],
      data[n][12],
      data[n][13]
    };
    /*joint_trajectories_2[n + 1].velocities = {
      (joint_trajectories_2[n].positions[0] - joint_trajectories_2[n + 1].positions[0]) / dt,
      (joint_trajectories_2[n].positions[1] - joint_trajectories_2[n + 1].positions[1]) / dt,
      (joint_trajectories_2[n].positions[2] - joint_trajectories_2[n + 1].positions[2]) / dt, 
      (joint_trajectories_2[n].positions[3] - joint_trajectories_2[n + 1].positions[3]) / dt, 
      (joint_trajectories_2[n].positions[4] - joint_trajectories_2[n + 1].positions[4]) / dt, 
      (joint_trajectories_2[n].positions[5] - joint_trajectories_2[n + 1].positions[5]) / dt, 
      (joint_trajectories_2[n].positions[6] - joint_trajectories_2[n + 1].positions[6]) / dt
    };*/
    joint_trajectories_2[n + 1].velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    /*for(int m = 7; m < 14; m++){
      if(joint_trajectories_2[n].positions[m] == joint_trajectories_2[n + 1].positions[m]){
        std::cout << "CLIPPING";
        joint_trajectories_2[n + 1].velocities[m] = 0.0;
      }
    }*/
  }
  std::cout << "positions:\n";
  for (int i = 0; i < joint_trajectories_2.size(); i++)
  {
    std::cout << "  ";
    for (int j = 0; j < 7; j++)
    {
      std::cout << joint_trajectories_2[i].positions[j] << " ";
    }
    std::cout << "\n";
  }

  std::cout << "\nvelocities:\n";
  for (int i = 0; i < joint_trajectories_2.size(); i++)
  {
    std::cout << "  ";
    for (int j = 0; j < 7; j++)
    {
      std::cout << joint_trajectories_2[i].velocities[j] << " ";
    }
    std::cout << "\n";
  }

  std::cout << "\ntimes:\n";
  for (int i = 0; i <joint_trajectories_2.size(); i++)
  {
    std::cout << "  ";
    std::cout << joint_trajectories_2[i].time_from_start;
    std::cout << "\n";
  }

  // Publish the trajectories.
  psm_1.move_psm(joint_trajectories_1);
  psm_2.move_psm(joint_trajectories_2);

  return 0;
}
