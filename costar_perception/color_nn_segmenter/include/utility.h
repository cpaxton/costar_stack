#ifndef UTILITY
#define UTILITY

#include <cmath>

// Based on opencv imgproc color conversion 
static const float ref_x = 0.950456;
static const float ref_z = 1.088754;

template<typename T>
void convertRgbToXyzColorSpace(T &ori_r, T &ori_g, T &ori_b, bool normalized = false)
{
	// r,g,b is a float number 0-255 if not normalized
	float r = ori_r,
		g = ori_g,
		b = ori_b;

	if (!normalized)
	{
		r /= 255; g /= 255; b /= 255;
	}

	float x,y,z;

	x = r * 0.412453 + g * 0.357580 + b * 0.180423;
	y = r * 0.212671 + g * 0.715160 + b * 0.072169;
	z = r * 0.019334 + g * 0.119193 + b * 0.950227;

	ori_r = x, ori_g = y, ori_b = z;
}

template<typename T> 
inline T f_t(const T &t)
{
	return t > 0.00856 ? pow(t,1./3) : 7.787 * t + 16./116;
}

template<typename T>
void convertXyzColorSpaceToLab(T &x, T &y, T &z)
{
	x /= ref_x;
	z /= ref_z;

	// CIE x,y,z = L*a*b
	T L, a, b;
	L = y > 0.008856 ? 116 * pow(y, (1./3)) - 16 : 903.3 * y;
	
	a = 500 * (f_t(x) - f_t(y));
	b = 200 * (f_t(y) - f_t(z));

	x = L;
	y = a;
	z = b;
}

template<typename T>
void convertRgbToLab(T &r, T &g, T &b, bool normalized = false)
{
	convertRgbToXyzColorSpace(r,g,b,normalized);
	convertXyzColorSpaceToLab(r,g,b);
}

bool isFileExist(const std::string &filename)
{
	std::ifstream file_tmp(filename.c_str());
	if (!file_tmp.is_open())
	{
		return false;
	}
	file_tmp.close();
	return true;
}


#endif