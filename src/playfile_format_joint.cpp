/******************************************************************************
**
**   cwru davinci Joint Space playfile io library.
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
#include <ros/package.h>
#include <cwru_davinci_playfile/playfile_format_joint.h>

void joint_format::read_line(const std::string & line, std::vector<double> & data)
{
  data.resize(43);
  std::string joint_info("%lf, %lf, %lf, %lf, %lf, %lf, %lf, ");
  std::string line_list;
  // PSM1 recording 7 doubles, 3 times.
  line_list += joint_info;
  line_list += joint_info;
  line_list += joint_info;
  // PSM2 recording 7 doubles, 3 times.
  line_list += joint_info;
  line_list += joint_info;
  line_list += joint_info;
  line_list += std::string("%lf");
  int code = sscanf(
    line.c_str(),
    line_list.c_str(),
    &data[ 0], &data[ 1], &data[ 2], &data[ 3], &data[ 4], &data[ 5], &data[ 6],
    &data[ 7], &data[ 8], &data[ 9], &data[10], &data[11], &data[12], &data[13],
    &data[14], &data[15], &data[16], &data[17], &data[18], &data[19], &data[20],
    &data[21], &data[22], &data[23], &data[24], &data[25], &data[26], &data[27],
    &data[28], &data[29], &data[30], &data[31], &data[32], &data[33], &data[34],
    &data[35], &data[36], &data[37], &data[38], &data[39], &data[40], &data[41],
    &data[42]);

  if (code != 15 && code != 43)
  {
    throw code;
  }
  if (code == 15)
  {
    data.resize(15);
  }
}

void joint_format::read_file(const std::string & filename, std::vector<std::vector<double> > & data)
{
  ROS_INFO("Absolute file location: %s", filename.c_str());

  std::fstream infile(filename.c_str());
  if (!infile)
  {
    ROS_ERROR("Error: file %s could not be opened.", filename.c_str());
    throw 1;
  }

  std::string line;
  while (getline(infile, line))
  {
    try
    {
      std::vector<double> l(15);
      read_line(line, l);
      data.push_back(l);
    }
    catch(int e)
    {
      ROS_ERROR("Bad JSP line: %s \n Line had %d recognizeable values.", line.c_str(), e);
      throw 2;
    }
  }
}

void joint_format::read_file(const std::string & package_name, const std::string & filename,
  std::vector<std::vector<double> > & data)
{
  ROS_INFO("Package file location: %s/play/jsp/%s", package_name.c_str(), filename.c_str());
  std::string fullname = ros::package::getPath(package_name) + "/play/jsp/" + filename;
  read_file(fullname, data);
}

std::string joint_format::write_line(const std::vector<double> & data)
{
  if (data.size() != 43 && data.size() != 15)
  {
    ROS_ERROR("Attempting to print malformed joint description of size %lu.", data.size());
  }

  std::string accum;
  for (int i = 0; i < (data.size()-1); i++)
  {
    accum = accum + std::to_string(data[i]) + ", ";
  }
  accum = accum + std::to_string(data.back());
  return accum;
}
