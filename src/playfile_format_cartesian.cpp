/******************************************************************************
**
**   cwru davinci Cartesian Space playfile io library.
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
#include <cwru_davinci_playfile/playfile_format_cartesian.h>

void cartesian_format::read_line(const std::string & line, std::vector<double> & data)
{
  std::string data_format;
  // add the string formats of the two PSM's
  data_format += "%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, ";
  data_format += "%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, ";
  data.resize(21);

  int code = sscanf(
    line.c_str(),
    data_format.c_str(),

    &data[0 ], &data[1 ], &data[2 ],
    &data[3 ], &data[4 ], &data[5 ],
    &data[6 ], &data[7 ], &data[8 ],
    &data[9 ],

    &data[10], &data[11], &data[12],
    &data[13], &data[14], &data[15],
    &data[16], &data[17], &data[18],
    &data[19],

    &data[20]);

  if (code != 21)
  {
    throw code;
  }
}

void cartesian_format::read_file(const std::string & filename, std::vector<std::vector<double> > & data)
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
      std::vector<double> l(21);
      read_line(line, l);
      data.push_back(l);
    }
    catch(int e)
    {
      ROS_ERROR("Bad PSP line: %s \n Line had %d recognizeable values.", line.c_str(), e);
      throw 2;
    }
  }
}

void cartesian_format::read_file(const std::string & package_name,
  const std::string & filename, std::vector<std::vector<double> > & data)
{
  ROS_INFO("Package file location: %s/play/psp/%s", package_name.c_str(), filename.c_str());

  std::string fullname = ros::package::getPath(package_name) + "/play/jsp/" + filename;

  read_file(fullname, data);
}

std::string cartesian_format::write_line(const std::vector<double> & data)
{
  if (data.size() != 21)
  {
    ROS_ERROR("Attempting to print malformed cartesian description of size %lu.", data.size());
  }

  std::string accum = "";
  for (int i = 0; i < 9; i++)
  {
    accum = accum + std::to_string(data[i]) + ", ";
  }
  accum = accum + std::to_string(data[9]) + ",\t";
  for (int i = 9; i < 19; i++)
  {
    accum = accum + std::to_string(data[i]) + ", ";
  }
  accum = accum + std::to_string(data[19]) + ",\t";
  accum = accum + std::to_string(data[20]);

  return accum;
}
