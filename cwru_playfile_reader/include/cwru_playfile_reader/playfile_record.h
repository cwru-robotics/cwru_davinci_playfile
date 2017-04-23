#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
/** \file playfile_record.h
*@brief Utility types and functions for the playfile reader.
*/

/**
*@brief A list of joint positions.
*
*One record contains all 14 degrees of freedom used by the two-armed DaVinci configuration.
*/
typedef std::vector <double> record_t;
/**
*@brief A list of records.
*
*A working trajectory must contain at least two records (start and finish), out to any arbitrary number.
*/
typedef std::vector <record_t> data_t;

/**
*@brief Creates a record from a line in an ASCII file
*
*@param ins input: istream contining a line following the JSP or CSP protocols
*@param data output: record_t created from that file
*/
std::istream& operator >> (std::istream & ins, record_t & record);

/**
*@brief Creates a data_t from an ASCII file
*
*@param ins input: istream pointing to an ASCII file following the JSP or CSP protocols
*@param data output: data_t created from that file
*/
std::istream& operator >> (std::istream & ins, data_t & data);
