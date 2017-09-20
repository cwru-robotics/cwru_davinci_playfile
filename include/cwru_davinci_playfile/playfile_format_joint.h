/******************************************************************************
**
**   cwru davinci Joint Space playfile format header for playfile io.
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
#include <fstream>
#include <iostream>
#include <sstream>


#ifndef CWRU_DAVINCI_PLAYFILE_PLAYFILE_FORMAT_JOINT_H
#define CWRU_DAVINCI_PLAYFILE_PLAYFILE_FORMAT_JOINT_H

namespace joint_format
{
  void read_line(const std::string & line, std::vector<double> & data);
  void read_file(const std::string & filename, std::vector<std::vector<double> > & data);
  void read_file(const std::string & package_name, const std::string & filename,
    std::vector<std::vector<double> > & data);
  std::string write_line(const std::vector<double> & data);
};  // namespace joint_format

#endif  // CWRU_DAVINCI_PLAYFILE_PLAYFILE_FORMAT_JOINT_H
