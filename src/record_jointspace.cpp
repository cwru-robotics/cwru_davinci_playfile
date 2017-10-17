/******************************************************************************
**
**   cwru davinci Joint Space playfile recorder for storing a robotic motion on a dvrk.
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
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <cwru_davinci_control/psm_controller.h>
#include <cwru_davinci_playfile/playfile_format_joint.h>

int main(int argc, char **argv)
{
  // TODO(rcj, tes) define a new input for setup time.
  double CAPTURE_TIME = -1.0;
  double CAPTURE_PER = -1.0;

  double setup_time(5.0);

  std::string CAPTURE_FILE;

  if (argc > 1 && argc < 5)
  {
    CAPTURE_FILE = argv[1];
    if (argc > 2)
    {
      CAPTURE_PER = strtod(argv[2], NULL);
      if (CAPTURE_PER <= 0.0)
      {
        ROS_ERROR("Custom capture period is %f, it must be greater than zero!", CAPTURE_PER);
        return 0;
      }
      if (argc > 3)
      {
        CAPTURE_TIME = strtod(argv[3], NULL);
        if (CAPTURE_PER <= 0.0)
        {
          ROS_ERROR("Capture time is %f, it must be greater than zero!", CAPTURE_TIME);
          return 0;
        }
        if (CAPTURE_TIME < CAPTURE_PER)
        {
          ROS_ERROR("Capture time is shorter than capture period, nothing will be recorded!");
          return 0;
        }
      }
    }
    else
    {
      CAPTURE_PER = 0.1;
    }
  }
  else
  {
    printf("Da Vinci playfile recorder. Usage:\n");
    std::string help_info;
    help_info += "  playfile_record [filename] \n [optional record period (default- 10 Hz)] \n";
    help_info += " [optional record duration (default- record until killed)]\n";
    printf("%s", help_info.c_str());
    return 0;
  }

  std::ofstream outfile;
  outfile.open(CAPTURE_FILE.c_str());
  if (!outfile.is_open())
  {
    ROS_ERROR("Unable to open record file %s", CAPTURE_FILE.c_str());
    return 1;
  }

  ros::init(argc, argv, "playfile_writer");
  ros::NodeHandle nh;

  psm_controller psm1(1, nh);
  psm_controller psm2(2, nh);

  ROS_INFO("Ready to start capturing- enter '1' to begin.");
  char ans = '0';
  while (ans != '1' && ros::ok())
  {
    std::cin >> ans;
  }

  sensor_msgs::JointState pose_1, pose_2;
  std::vector<double> pose_both;
  psm1.get_fresh_psm_state(pose_1);
  psm2.get_fresh_psm_state(pose_2);
  pose_both.insert(pose_both.end(), pose_1.position.begin(), pose_1.position.end());
  pose_both.insert(pose_both.end(), pose_2.position.begin(), pose_2.position.end());
  pose_both.push_back(setup_time);

  outfile << joint_format::write_line(pose_both);

  ros::spinOnce();

  ROS_INFO("Recording. Press Ctrl-C to end.");

  double total_time = 0.0;
  ros::Time starting = ros::Time::now();
  double start_time = starting.toSec();
  // TODO(rcj, tes) make the joint space recording reflect a real time operation.
  while (ros::ok() && (CAPTURE_TIME == -1.0 || total_time < CAPTURE_TIME))
  {
    ros::Duration sl(CAPTURE_PER);

    ros::Time current = ros::Time::now();

    total_time = total_time + CAPTURE_PER;

    if (psm1.get_psm_state(pose_1) || psm2.get_psm_state(pose_2))
    {
      pose_both.clear();
      pose_both.insert(pose_both.end(), pose_1.position.begin(), pose_1.position.end());
      pose_both.insert(pose_both.end(), pose_2.position.begin(), pose_2.position.end());
      pose_both.push_back(setup_time + current.toSec() - start_time);
      outfile << "\n";
      outfile << joint_format::write_line(pose_both);
    }
    ros::spinOnce();
    sl.sleep();
  }
  outfile.close();
  return 0;
}
