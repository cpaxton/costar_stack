#ifndef STRINGVECTORARGSREADER_H_
#define STRINGVECTORARGSREADER_H_

#include "sp_segmenter/features.h"

// ros header for roslaunch capability
#include <ros/ros.h>

std::vector<std::string> stringVectorArgsReader (ros::NodeHandle nh, 
    const std::string param_name, const std::string default_value)
{
// The function will read string arguments passed from nodehandle, 
// separate it for every ',' character, and return it to vector<string> variable
	std::string tmp;
	std::vector<std::string> result;
	nh.param(param_name,tmp,default_value);

	char splitOperator(',');
	std::size_t found = tmp.find(splitOperator);
	std::size_t pos = 0;

	while (found!=std::string::npos)
	{
		std::string buffer;
		buffer.assign(tmp, pos, found - pos);
		result.push_back(buffer);
		pos = found + 1;
		found = tmp.find(splitOperator,pos);
	}

	std::string buffer;
	buffer.assign(tmp, pos, tmp.length() - pos);
	result.push_back(buffer);
	return result;
}

#endif