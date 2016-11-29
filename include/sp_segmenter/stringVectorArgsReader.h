#ifndef STRINGVECTORARGSREADER_H_
#define STRINGVECTORARGSREADER_H_

inline
std::vector<std::string> stringVectorArgsReader (const std::string &input_string)
{
	std::vector<std::string> result;
	char splitOperator(',');
	std::size_t found = input_string.find(splitOperator);
	std::size_t pos = 0;

	while (found!=std::string::npos)
	{
		std::string buffer;
		buffer.assign(input_string, pos, found - pos);
		result.push_back(buffer);
		pos = found + 1;
		found = input_string.find(splitOperator,pos);
	}

	std::string buffer;
	buffer.assign(input_string, pos, input_string.length() - pos);
	result.push_back(buffer);
	return result;
}
#ifdef BUILD_ROS_BINDING
inline
std::vector<std::string> stringVectorArgsReader (const ros::NodeHandle &nh,
    const std::string &param_name, const std::string &default_value)
{
// The function will read string arguments passed from nodehandle, 
// separate it for every ',' character, and return it to vector<string> variable
	std::string tmp;
	nh.param(param_name,tmp,default_value);
	return stringVectorArgsReader(tmp);
}
#endif


#endif