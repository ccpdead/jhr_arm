#ifdef WIN32

	#include <Windows.h>
	#include <direct.h>
	#include <psapi.h>
	#include <io.h>

	#define R_OK 4
	#define W_OK 2
	#define X_OK 1
	#define F_OK 0

#elif defined LINUX 

	#include <unistd.h>
	#include <sys/stat.h>
	#include <sys/time.h>
	#include <sys/sysinfo.h>

#endif
#include <time.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <iostream>
#include <thread>
#include <list>
#include <mutex>
#include <vector>
#include <chrono>
#include <algorithm>
#include "tof_error.h"
#include "typedef.h"
#include "tof_dev_sdk.h"

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


ros::Publisher pub_info;
ros::Publisher pub_pointCloud;
ros::Publisher pub_depthz;
ros::Publisher pub_gray;
ros::Publisher pub_rgbd;
ros::Publisher pub_rgb;

sensor_msgs::CameraInfoPtr camInfoMsg;
sensor_msgs::PointCloud2 cloudMsg;
sensor_msgs::ImagePtr depthzMsg;
sensor_msgs::ImagePtr grayMsg;
sensor_msgs::ImagePtr rgbdMsg;
sensor_msgs::ImagePtr rgbMsg;

pcl::PointCloud<pcl::PointXYZ> pclCloud;


//
#define SAFE_DELETE(p) if(p){delete p; p=NULL;}
#define SAFE_DELETE_ARRY(p) if(p){delete [] p; p=NULL;}
#define SAFE_FREE(p) if(p){free(p); p=NULL;}
#define SAFE_CLOSE_FP(fp) if(fp){fclose(fp); fp=NULL;}



#ifdef WIN32
static int gettimeofday(struct timeval *tp, void *tzp)
{
	time_t clock;
	struct tm tm;
	SYSTEMTIME wtm;
	GetLocalTime(&wtm);
	tm.tm_year = wtm.wYear - 1900;
	tm.tm_mon = wtm.wMonth - 1;
	tm.tm_mday = wtm.wDay;
	tm.tm_hour = wtm.wHour;
	tm.tm_min = wtm.wMinute;
	tm.tm_sec = wtm.wSecond;
	tm.tm_isdst = -1;
	clock = mktime(&tm);
	tp->tv_sec = (long)clock;
	tp->tv_usec = wtm.wMilliseconds * 1000;
	return (0);
}
#endif

static unsigned long long Utils_GetTickCount(void)
{
	unsigned long long tick = 0;

#ifdef WIN32
	//tick = GetTickCount();//实际精度只有15ms左右; 返回的是一个32位的无符号整数，Windows连续运行49.710天后，它将再次从零开始计时; 
	//tick = GetTickCount64();//返回一个64位的无符号整数。Windows连续运行5.8亿年后，其计时才会归零; 
	//tick = clock();//该程序从启动到函数调用占用CPU的时间, 是C/C++中的计时函数

	//struct timeval tv;
	//gettimeofday(&tv, 0);
	//tick = (tv.tv_sec * 1000 + tv.tv_usec / 1000);
		
	auto timePoint = std::chrono::steady_clock::now(); // std::chrono::time_point
	tick = std::chrono::duration_cast<std::chrono::milliseconds>(timePoint.time_since_epoch()).count();

#elif defined LINUX
	//struct timeval tv;
	//gettimeofday(&tv, 0);
	//tick = (tv.tv_sec * 1000 + tv.tv_usec/1000);
	
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
	tick = (tv.tv_sec * 1000 + tv.tv_nsec/1000000);
	
#else
	printf("unknown platform in getting tick cnt, error!\n");
#endif // WIN32

	return tick;
}


static long Utils_GetFileLen(const char * filename)
{
	if (NULL == filename)
	{
		printf("NULL == filename\n");
		return 0;
	}

	FILE * fd = fopen(filename, "rb");
	if (NULL == fd)
	{
		printf("open file (%s) failed, errno=%d(%s).\n", filename, errno, strerror(errno));
		return 0;
	}

	fseek(fd, 0L, SEEK_END); /* 定位到文件末尾 */
	const long len = ftell(fd);
	fclose(fd);

	return len;

}

static void Utils_SaveBufToFile(void* pData, const unsigned int nDataLen, const char* pFile, const bool bAppend)
{
	if ((NULL == pData) || (0 >= nDataLen) || (NULL == pFile))
	{
		return;
	}

	FILE* fp = fopen(pFile, (bAppend ? "ab" : "wb"));
	if (NULL == fp)
	{
		printf("open file(%s) failed, error=%d(%s).\n", pFile, errno, strerror(errno));
		return;
	}

	fwrite(pData, 1, nDataLen, fp);
	fclose(fp);
}

template <class T>
T Utils_FindMaxValue(T* pData, const int nCnt)
{
	T max = pData[0];

	for (int i = 0; i < nCnt; i++)
	{
		if (max < pData[i])
		{
			max = pData[i];
		}
	}

	return max;
}

template <class T>
T Utils_FindMinValue(T* pData, const int nCnt)
{
	T min = pData[0];

	for (int i = 0; i < nCnt; i++)
	{
		if (min > pData[i])
		{
			min = pData[i];
		}
	}

	return min;
}

static int CaculateGrayPixelBytes(const GRAY_FORMAT format)
{
	int grayByte = 1;

	switch (format)
	{
	case GRAY_FORMAT_UINT8: grayByte = 1; break;
	case GRAY_FORMAT_UINT16: grayByte = 2; break;
	case GRAY_FORMAT_FLOAT: grayByte = 4; break;
	case GRAY_FORMAT_BGRD: grayByte = 4; break;
	default:break;
	}

	return grayByte;

}
static const char* StringGrayFormat(const GRAY_FORMAT format)
{
	const char* pStr = "UnknownGRAY";

	switch (format)
	{
	case GRAY_FORMAT_UINT8: pStr = "U8"; break;
	case GRAY_FORMAT_UINT16: pStr = "U16"; break;
	case GRAY_FORMAT_FLOAT: pStr = "FLOAT"; break;
	case GRAY_FORMAT_BGRD: pStr = "BGRD"; break;

	default: break;
	}

	return pStr;
}
static float CalCenterPointDataZAvg(PointData *pPointData, const UINT32 width, const UINT32 height)
{
	if (NULL == pPointData)
	{
		return 0;
	}

	const int start_h = (10<height) ? ((height / 2) - 5) : 0;
	const int end_h = (10<height) ? ((height / 2) + 5) : (height);
	const int start_w = (10<width) ? ((width / 2) - 5) : 0;
	const int end_w = (10<width) ? ((width / 2) + 5) : (width);


	float sum = 0.0;
	int cnt = 0;
	for (int h = start_h; h < end_h; h++)
	{
		PointData *pTmp = pPointData + h*width;
		for (int w = start_w; w < end_w; w++)
		{
			if (0.00001 < pTmp[w].z)
			{
				sum += pTmp[w].z;
				cnt++;
			}
		}
	}

	return ((0 < cnt) ? (sum / cnt) : 0);
}

static const char* StringColorFormat(const COLOR_FORMAT type)
{
	const char* pStr = "UnknownRGB";

	switch (type)
	{
	case COLOR_FORMAT_MJPG: pStr = "JPG"; break;

	case COLOR_FORMAT_H264: pStr = "H264"; break;

	case COLOR_FORMAT_YUV422: pStr = "YUV422"; break;
	case COLOR_FORMAT_YUYV: pStr = "YUYV"; break;
	case COLOR_FORMAT_I420: pStr = "I420"; break;
	case COLOR_FORMAT_YV12: pStr = "YV12"; break;
	case COLOR_FORMAT_NV12: pStr = "NV12"; break;
	case COLOR_FORMAT_NV21: pStr = "NV21"; break;

	case COLOR_FORMAT_BGR: pStr = "BGR"; break;
	case COLOR_FORMAT_RGB: pStr = "RGB"; break;
	case COLOR_FORMAT_BGRA: pStr = "BGRA"; break;
	case COLOR_FORMAT_RGBA: pStr = "RGBA"; break;

	default: break;
	}

	return pStr;
}
static const char* TofMode2Str(const TOF_MODE mode)
{
	const char* pStr = "Unknown";

	switch (mode)
	{
	case TOF_MODE_STERO_5FPS: pStr = "STERO_5FPS"; break;
	case TOF_MODE_STERO_10FPS: pStr = "STERO_10FPS"; break;
	case TOF_MODE_STERO_15FPS: pStr = "STERO_15FPS"; break;
	case TOF_MODE_STERO_30FPS: pStr = "STERO_30FPS"; break;
	case TOF_MODE_STERO_45FPS: pStr = "STERO_45FPS"; break;
	case TOF_MODE_STERO_60FPS: pStr = "STERO_60FPS"; break;

	case TOF_MODE_MONO_5FPS: pStr = "MONO_5FPS"; break;
	case TOF_MODE_MONO_10FPS: pStr = "MONO_10FPS"; break;
	case TOF_MODE_MONO_15FPS: pStr = "MONO_15FPS"; break;
	case TOF_MODE_MONO_30FPS: pStr = "MONO_30FPS"; break;
	case TOF_MODE_MONO_45FPS: pStr = "MONO_45FPS"; break;
	case TOF_MODE_MONO_60FPS: pStr = "MONO_60FPS"; break;

	case TOF_MODE_HDRZ_5FPS: pStr = "HDRZ_5FPS"; break;
	case TOF_MODE_HDRZ_10FPS: pStr = "HDRZ_10FPS"; break;
	case TOF_MODE_HDRZ_15FPS: pStr = "HDRZ_15FPS"; break;
	case TOF_MODE_HDRZ_30FPS: pStr = "HDRZ_30FPS"; break;
	case TOF_MODE_HDRZ_45FPS: pStr = "HDRZ_45FPS"; break;
	case TOF_MODE_HDRZ_60FPS: pStr = "HDRZ_60FPS"; break;

	case TOF_MODE_5FPS: pStr = "5FPS"; break;
	case TOF_MODE_10FPS: pStr = "10FPS"; break;
	case TOF_MODE_20FPS: pStr = "20FPS"; break;
	case TOF_MODE_30FPS: pStr = "30FPS"; break;
	case TOF_MODE_45FPS: pStr = "45FPS"; break;
	case TOF_MODE_60FPS: pStr = "60FPS"; break;

	case TOF_MODE_ADI_1M5: pStr = "ADI_1M5"; break;
	case TOF_MODE_ADI_5M: pStr = "ADI_5M"; break;

	case TOF_MODE_CUSTOM_1: pStr = "CUSTOM_1"; break;
	case TOF_MODE_CUSTOM_2: pStr = "CUSTOM_2"; break;
	case TOF_MODE_CUSTOM_3: pStr = "CUSTOM_3"; break;
	case TOF_MODE_CUSTOM_4: pStr = "CUSTOM_4"; break;
	case TOF_MODE_CUSTOM_5: pStr = "CUSTOM_5"; break;

	case TOF_MODE_DEBUG: pStr = "DEBUG"; break;


	default: break;
	}

	return pStr;
}


class CGrayConvert
{
	CGrayConvert();
	~CGrayConvert();

public:
	static bool Gray_2_Bgr32(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned int* pBgr32);
	static bool Gray_2_U16(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned short* pU16);
	static bool Gray_2_U8(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned char* pU8);

private:
	static bool ToBgr32(unsigned char* pGray, const int width, const int height, unsigned int* pBgr32);
	static bool ToBgr32(unsigned short* pGray, const int width, const int height, unsigned int* pBgr32);
	static bool ToBgr32(float* pGray, const int width, const int height, unsigned int* pBgr32);
	static bool ToBgr32(unsigned int* pGray, const int width, const int height, unsigned int* pBgr32);
private:
	static bool ToU16(unsigned char* pGray, const int width, const int height, unsigned short* pU16);
	static bool ToU16(unsigned short* pGray, const int width, const int height, unsigned short* pU16);
	static bool ToU16(float* pGray, const int width, const int height, unsigned short* pU16);
	static bool ToU16(unsigned int* pGray, const int width, const int height, unsigned short* pU16);
private:
	static bool ToU8(unsigned char* pGray, const int width, const int height, unsigned char* pU8);
	static bool ToU8(unsigned short* pGray, const int width, const int height, unsigned char* pU8);
	static bool ToU8(float* pGray, const int width, const int height, unsigned char* pU8);
	static bool ToU8(unsigned int* pGray, const int width, const int height, unsigned char* pU8);
};
CGrayConvert::CGrayConvert()
{
}
CGrayConvert::~CGrayConvert()
{
}

bool CGrayConvert::Gray_2_Bgr32(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned int* pBgr32)
{
	bool retVal = false;

	switch (format)
	{
	case GRAY_FORMAT_UINT8:  retVal = ToBgr32((unsigned char*)pGray, width, height, pBgr32); break;
	case GRAY_FORMAT_UINT16: retVal = ToBgr32((unsigned short*)pGray, width, height, pBgr32); break;
	case GRAY_FORMAT_FLOAT:  retVal = ToBgr32((float*)pGray, width, height, pBgr32); break;
	case GRAY_FORMAT_BGRD:   retVal = ToBgr32((unsigned int*)pGray, width, height, pBgr32); break;
	default: break;
	}

	return retVal;
}

bool CGrayConvert::Gray_2_U16(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned short* pU16)
{
	bool retVal = false;

	switch (format)
	{
	case GRAY_FORMAT_UINT8:  retVal = ToU16((unsigned char*)pGray, width, height, pU16); break;
	case GRAY_FORMAT_UINT16: retVal = ToU16((unsigned short*)pGray, width, height, pU16); break;
	case GRAY_FORMAT_FLOAT:  retVal = ToU16((float*)pGray, width, height, pU16); break;
	case GRAY_FORMAT_BGRD:   retVal = ToU16((unsigned int*)pGray, width, height, pU16); break;
	default: break;
	}

	return retVal;
}

bool CGrayConvert::Gray_2_U8(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned char* pU8)
{
	bool retVal = false;

	switch (format)
	{
	case GRAY_FORMAT_UINT8:  retVal = ToU8((unsigned char*)pGray, width, height, pU8); break;
	case GRAY_FORMAT_UINT16: retVal = ToU8((unsigned short*)pGray, width, height, pU8); break;
	case GRAY_FORMAT_FLOAT:  retVal = ToU8((float*)pGray, width, height, pU8); break;
	case GRAY_FORMAT_BGRD:   retVal = ToU8((unsigned int*)pGray, width, height, pU8); break;
	default: break;
	}

	return retVal;
}
bool CGrayConvert::ToBgr32(unsigned char* pGray, const int width, const int height, unsigned int* pBgr32)
{
	const int pixel_cnt = width*height;

	for (int i = 0; i < pixel_cnt; i++)
	{
		pBgr32[i] = ((pGray[i] << 24) | (pGray[i] << 16) | (pGray[i] << 8) | pGray[i]);
	}

	return true;
}
bool CGrayConvert::ToBgr32(unsigned short* pGray, const int width, const int height, unsigned int* pBgr32)
{
#if 0
	const int pixel_cnt = width*height;
	//const unsigned short min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const unsigned short max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0 >= max)
	{
		memset(pBgr32, 0, pixel_cnt * sizeof(pBgr32[0]));
		return true;
	}

	const float K = (255 * 1.0f / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		const unsigned char tmp = (unsigned char)(pGray[i] * K);
		pBgr32[i] = ((tmp << 24) | (tmp << 16) | (tmp << 8) | tmp);
	}
#endif

#if 1
	const int pixel_cnt = width*height;

	//
	unsigned short* pGrayTmp = new unsigned short[pixel_cnt];
	memcpy(pGrayTmp, pGray, pixel_cnt * sizeof(pGray[0]));
	const int nth = (int)(0.995f * pixel_cnt);
	std::nth_element(pGrayTmp, pGrayTmp + nth, pGrayTmp + pixel_cnt);//找到第K小的数据
	const unsigned short max = pGrayTmp[nth];
	delete[] pGrayTmp;

	//
	if (0 >= max)
	{
		memset(pBgr32, 0, pixel_cnt * sizeof(pBgr32[0]));
		return true;
	}

	const float K = (255 * 1.0f / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		const unsigned char tmp = (unsigned char)((max < pGray[i]) ? 255 : (pGray[i] * K));
		pBgr32[i] = ((tmp << 24) | (tmp << 16) | (tmp << 8) | tmp);
	}
#endif

	return true;
}
bool CGrayConvert::ToBgr32(float* pGray, const int width, const int height, unsigned int* pBgr32)
{
#if 0
	const int pixel_cnt = width*height;
	//const float min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const float max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0.001f >= max)//0值用黑色表示
	{
		memset(pBgr32, 0, pixel_cnt * sizeof(pBgr32[0]));
		return true;
	}

	const float K = (255 * 1.0f / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned char tmp = 0;//0值用黑色表示
		if (0.001f < pGray[i])
		{
			tmp = (unsigned char)(pGray[i] * K);
		}
		pBgr32[i] = ((tmp << 24) | (tmp << 16) | (tmp << 8) | tmp);
	}
#endif

#if 1
	const int pixel_cnt = width*height;

	//
	float* pGrayTmp = new float[pixel_cnt];
	memcpy(pGrayTmp, pGray, pixel_cnt * sizeof(pGray[0]));
	const int nth = (int)(0.995f * pixel_cnt);
	std::nth_element(pGrayTmp, pGrayTmp + nth, pGrayTmp + pixel_cnt);//找到第K小的数据
	const float max = pGrayTmp[nth];
	delete[] pGrayTmp;

	//
	if (0.001f >= max)//0值用黑色表示
	{
		memset(pBgr32, 0, pixel_cnt * sizeof(pBgr32[0]));
		return true;
	}

	const float K = (255 * 1.0f / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		const unsigned char tmp = (unsigned char)((max < pGray[i]) ? 255 : (pGray[i] * K));
		pBgr32[i] = ((tmp << 24) | (tmp << 16) | (tmp << 8) | tmp);
	}
#endif

	return true;
}
bool CGrayConvert::ToBgr32(unsigned int* pGray, const int width, const int height, unsigned int* pBgr32)
{
	const int pixel_cnt = width*height;

	memcpy(pBgr32, pGray, pixel_cnt * sizeof(pBgr32[0]));

	return true;
}

bool CGrayConvert::ToU16(unsigned char* pGray, const int width, const int height, unsigned short* pU16)
{
	const int pixel_cnt = width*height;
	//const unsigned char min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const unsigned char max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0 >= max)
	{
		memset(pU16, 0, pixel_cnt * sizeof(pU16[0]));
		return true;
	}

	const float K = (65535 * 1.0f / max);//最大值是65535的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		const unsigned short tmp = (unsigned short)(pGray[i] * K);
		pU16[i] = tmp;
	}

	return true;
}
bool CGrayConvert::ToU16(unsigned short* pGray, const int width, const int height, unsigned short* pU16)
{
	const int pixel_cnt = width*height;

	memcpy(pU16, pGray, pixel_cnt * sizeof(pU16[0]));

	return true;
}
bool CGrayConvert::ToU16(float* pGray, const int width, const int height, unsigned short* pU16)
{
#if 1
	//方法1：灰度直接数据类型强转

	const int pixel_cnt = width*height;

	for (int i = 0; i < pixel_cnt; i++)
	{
		pU16[i] = (unsigned short)(((65535.0f < pGray[i]) ? 65535 : pGray[i]));
	}

#else
	//方法2：灰度按照等比例压缩

	const int pixel_cnt = width*height;
	//const float min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const float max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0.001f >= max)//0值用黑色表示
	{
		memset(pU16, 0, pixel_cnt * sizeof(pU16[0]));
		return true;
	}

	const float K = (65535 * 1.0f / max);//最大值是65535的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned short tmp = 0;//0值用黑色表示
		if (0.001f < pGray[i])
		{
			tmp = (unsigned short)(pGray[i] * K);
		}
		pU16[i] = tmp;
	}
#endif
	return true;
}
bool CGrayConvert::ToU16(unsigned int* pGray, const int width, const int height, unsigned short* pU16)
{
	const int pixel_cnt = width*height;

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned char* pTmp = (unsigned char*)(pGray + i);//BGRD排列
		pU16[i] = (pTmp[0] << 8);//放大到65535
	}

	return true;
}

bool CGrayConvert::ToU8(unsigned char* pGray, const int width, const int height, unsigned char* pU8)
{
	const int pixel_cnt = width*height;

	memcpy(pU8, pGray, pixel_cnt * sizeof(pU8[0]));

	return true;
}
bool CGrayConvert::ToU8(unsigned short* pGray, const int width, const int height, unsigned char* pU8)
{
#if 0
	const int pixel_cnt = width*height;
	//const unsigned short min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const unsigned short max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0 >= max)
	{
		memset(pU8, 0, pixel_cnt * sizeof(pU8[0]));
		return true;
	}

	const float K = (255 * 1.0f / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		const unsigned char tmp = (unsigned char)(pGray[i] * K);
		pU8[i] = tmp;
	}
#endif

#if 1
	const int pixel_cnt = width*height;

	//
	unsigned short* pGrayTmp = new unsigned short[pixel_cnt];
	memcpy(pGrayTmp, pGray, pixel_cnt * sizeof(pGray[0]));
	const int nth = (int)(0.995f * pixel_cnt);
	std::nth_element(pGrayTmp, pGrayTmp + nth, pGrayTmp + pixel_cnt);//找到第K小的数据
	const unsigned short max = pGrayTmp[nth];
	delete[] pGrayTmp;

	//
	if (0 >= max)
	{
		memset(pU8, 0, pixel_cnt * sizeof(pU8[0]));
		return true;
	}

	const float K = (255 * 1.0f / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		pU8[i] = (unsigned char)((max < pGray[i]) ? 255 : (pGray[i] * K));
	}
#endif

	return true;
}
bool CGrayConvert::ToU8(float* pGray, const int width, const int height, unsigned char* pU8)
{
#if 0
	const int pixel_cnt = width*height;
	//const float min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const float max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0.001f >= max)//0值用黑色表示
	{
		memset(pU8, 0, pixel_cnt * sizeof(pU8[0]));
		return true;
	}

	const float K = (255 * 1.0f / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned char tmp = 0;//0值用黑色表示
		if (0.001f < pGray[i])
		{
			tmp = (unsigned char)(pGray[i] * K);
		}
		pU8[i] = tmp;
	}
#endif

#if 1
	const int pixel_cnt = width*height;

	//
	float* pGrayTmp = new float[pixel_cnt];
	memcpy(pGrayTmp, pGray, pixel_cnt * sizeof(pGray[0]));
	const int nth = (int)(0.995f * pixel_cnt);
	std::nth_element(pGrayTmp, pGrayTmp + nth, pGrayTmp + pixel_cnt);//找到第K小的数据
	const float max = pGrayTmp[nth];
	delete[] pGrayTmp;

	//
	if (0.001f >= max)//0值用黑色表示
	{
		memset(pU8, 0, pixel_cnt * sizeof(pU8[0]));
		return true;
	}

	const float K = (255 * 1.0f / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		pU8[i] = (unsigned char)((max < pGray[i]) ? 255 : (pGray[i] * K));
	}

#endif

	return true;
}
bool CGrayConvert::ToU8(unsigned int* pGray, const int width, const int height, unsigned char* pU8)
{
	const int pixel_cnt = width*height;

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned char* pTmp = (unsigned char*)(pGray + i);//BGRD排列
		pU8[i] = pTmp[0];
	}

	return true;
}

static bool SaveGray_2_BGR32(void* pGray, const UINT32 width, const UINT32 height, const GRAY_FORMAT format, char* pFile)
{
	if ((NULL == pGray) || (0 >= width) || (0 >= height) || (NULL == pFile))
	{
		return false;
	}

	unsigned int* pData = new unsigned int[width * height];//bgra
	CGrayConvert::Gray_2_Bgr32(pGray, width, height, format, pData);
	Utils_SaveBufToFile(pData, width * height * sizeof(pData[0]), (const char*)pFile, false);
	SAFE_DELETE_ARRY(pData);

	return true;

}
static bool SaveGray_2_U16(void* pGray, const UINT32 width, const UINT32 height, const GRAY_FORMAT format, char* pFile)
{
	if ((NULL == pGray) || (0 >= width) || (0 >= height) || (NULL == pFile))
	{
		return false;
	}

	unsigned short* pData = new unsigned short[width * height];//U16
	CGrayConvert::Gray_2_U16(pGray, width, height, format, pData);
	Utils_SaveBufToFile(pData, width * height * sizeof(pData[0]), (const char*)pFile, false);
	SAFE_DELETE_ARRY(pData);

	return true;

}
static bool SaveGray_2_U8(void* pGray, const UINT32 width, const UINT32 height, const GRAY_FORMAT format, char* pFile)
{
	if ((NULL == pGray) || (0 >= width) || (0 >= height) || (NULL == pFile))
	{
		return false;
	}

	unsigned char* pData = new unsigned char[width * height];//U8
	CGrayConvert::Gray_2_U8(pGray, width, height, format, pData);
	Utils_SaveBufToFile(pData, width * height * sizeof(pData[0]), (const char*)pFile, false);
	SAFE_DELETE_ARRY(pData);

	return true;

}

static bool SaveDepthText(float* pDepthData, const UINT32 width, const UINT32 height, char* pTxtFile, const bool bWH)
{
	if ((NULL == pDepthData) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	if (bWH)//1：W列、H行排列
	{
		UINT32 nPos = 0;
		for (UINT32 h = 0; h < height; h++)
		{
			for (UINT32 w = 0; w < (width - 1); w++)
			{
				fprintf(fp, "%0.6f,", pDepthData[nPos]);
				nPos++;
			}
			fprintf(fp, "%0.6f\n", pDepthData[nPos]);
			nPos++;
		}
	}
	else//2：1行、W*H行排列
	{
		const UINT32 nCnt = width *height;
		for (UINT32 nPos = 0; nPos < nCnt; nPos++)
		{
			fprintf(fp, "%0.6f\n", pDepthData[nPos]);
		}
	}

	fclose(fp);
	return true;
}
static bool SavePointDataXYZText(PointData *pPointData, const UINT32 width, const UINT32 height, char* pTxtFile)
{
	if ((NULL == pPointData) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	const UINT32 nCnt = width *height;
	for (UINT32 nPos = 0; nPos < nCnt; nPos++)
	{
		fprintf(fp, "%0.6f;%0.6f;%0.6f\n", pPointData[nPos].x, pPointData[nPos].y, pPointData[nPos].z);
	}

	fclose(fp);
	return true;
}
static bool SavePointDataZWHText(PointData *pPointData, const UINT32 width, const UINT32 height, char* pTxtFile)
{
	if ((NULL == pPointData) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	UINT32 nPos = 0;
	for (UINT32 h = 0; h < height; h++)
	{
		for (UINT32 w = 0; w < width; w++)
		{
			fprintf(fp, "%0.6f", pPointData[nPos].z);
			nPos++;
		}
		fprintf(fp, "\n");
	}

	fclose(fp);
	return true;
}


static void CaptureTofFrame(const std::string& strDir, const unsigned int nCaptureIndex, TofFrameData *tofFrameData)
{
	const unsigned int nPixelCnt = tofFrameData->frameWidth * tofFrameData->frameHeight;
	char szFile[512] = { 0 };

	//
	if (NULL != tofFrameData->pDepthData)
	{
		sprintf(szFile, "%s/%u-DepthData.dat", strDir.c_str(), nCaptureIndex);
		Utils_SaveBufToFile(tofFrameData->pDepthData, nPixelCnt  * sizeof(tofFrameData->pDepthData[0]), szFile, false);

		sprintf(szFile, "%s/%u-DepthData.txt", strDir.c_str(), nCaptureIndex);
		SaveDepthText(tofFrameData->pDepthData, tofFrameData->frameWidth, tofFrameData->frameHeight, szFile, false);
	}

	//
	if (NULL != tofFrameData->pDepthDataFilter)
	{
		sprintf(szFile, "%s/%u-DepthDataFilter.dat", strDir.c_str(), nCaptureIndex);
		Utils_SaveBufToFile(tofFrameData->pDepthDataFilter, nPixelCnt  * sizeof(tofFrameData->pDepthDataFilter[0]), szFile, false);

		sprintf(szFile, "%s/%u-DepthDataFilter.txt", strDir.c_str(), nCaptureIndex);
		SaveDepthText(tofFrameData->pDepthDataFilter, tofFrameData->frameWidth, tofFrameData->frameHeight, szFile, false);
	}

	//
	if (NULL != tofFrameData->pPointData)
	{
		sprintf(szFile, "%s/%u-PointData.dat", strDir.c_str(), nCaptureIndex);
		Utils_SaveBufToFile(tofFrameData->pPointData, nPixelCnt  * sizeof(tofFrameData->pPointData[0]), szFile, false);

		sprintf(szFile, "%s/%u-PointData.txt", strDir.c_str(), nCaptureIndex);
		SavePointDataXYZText(tofFrameData->pPointData, tofFrameData->frameWidth, tofFrameData->frameHeight, szFile);
	}

	//
	if (NULL != tofFrameData->pPointDataUnfilter)
	{
		sprintf(szFile, "%s/%u-PointDataUnfilter.dat", strDir.c_str(), nCaptureIndex);
		Utils_SaveBufToFile(tofFrameData->pPointDataUnfilter, nPixelCnt  * sizeof(tofFrameData->pPointDataUnfilter[0]), szFile, false);

		sprintf(szFile, "%s/%u-PointDataUnfilter.txt", strDir.c_str(), nCaptureIndex);
		SavePointDataXYZText(tofFrameData->pPointDataUnfilter, tofFrameData->frameWidth, tofFrameData->frameHeight, szFile);
	}

	//
	if (NULL != tofFrameData->pGrayData)
	{
		sprintf(szFile, "%s/%u-Gray.%s", strDir.c_str(), nCaptureIndex, StringGrayFormat(tofFrameData->grayFormat));
		Utils_SaveBufToFile(tofFrameData->pGrayData, nPixelCnt * CaculateGrayPixelBytes(tofFrameData->grayFormat), szFile, false);

		sprintf(szFile, "%s/%u-Gray.u8", strDir.c_str(), nCaptureIndex);
		SaveGray_2_U8(tofFrameData->pGrayData, tofFrameData->frameWidth, tofFrameData->frameHeight, tofFrameData->grayFormat, szFile);
	}
	//
	if (NULL != tofFrameData->pConfidence)
	{
		sprintf(szFile, "%s/%u-Confidence.dat", strDir.c_str(), nCaptureIndex);
		Utils_SaveBufToFile(tofFrameData->pConfidence, nPixelCnt  * sizeof(tofFrameData->pConfidence[0]), szFile, false);

		sprintf(szFile, "%s/%u-Confidence.txt", strDir.c_str(), nCaptureIndex);
		SaveDepthText(tofFrameData->pConfidence, tofFrameData->frameWidth, tofFrameData->frameHeight, szFile, false);

		sprintf(szFile, "%s/%u-Confidence.u8", strDir.c_str(), nCaptureIndex);
		SaveGray_2_U8(tofFrameData->pConfidence, tofFrameData->frameWidth, tofFrameData->frameHeight, GRAY_FORMAT_FLOAT, szFile);
	}
	//
	if (NULL != tofFrameData->pIntensity)
	{
		sprintf(szFile, "%s/%u-Intensity.dat", strDir.c_str(), nCaptureIndex);
		Utils_SaveBufToFile(tofFrameData->pIntensity, nPixelCnt  * sizeof(tofFrameData->pIntensity[0]), szFile, false);

		sprintf(szFile, "%s/%u-Intensity.txt", strDir.c_str(), nCaptureIndex);
		SaveDepthText(tofFrameData->pIntensity, tofFrameData->frameWidth, tofFrameData->frameHeight, szFile, false);
	}
	//
	if (NULL != tofFrameData->pIntensityU8)
	{
		sprintf(szFile, "%s/%u-IntensityU8.u8", strDir.c_str(), nCaptureIndex);
		Utils_SaveBufToFile(tofFrameData->pIntensityU8, nPixelCnt  * sizeof(tofFrameData->pIntensityU8[0]), szFile, false);
	}

	//
	if ((NULL != tofFrameData->pRawData) && (0 < tofFrameData->nRawDataLen))
	{
		sprintf(szFile, "%s/%u-Tof.%s", strDir.c_str(), nCaptureIndex, "raw");
		Utils_SaveBufToFile(tofFrameData->pRawData, tofFrameData->nRawDataLen, szFile, false);
	}
}

static void CallBackTofDeviceStatus(TOFDEV_STATUS tofDevStatus, void *pUserData)
{
	printf("device status: 0x08%x.\n", tofDevStatus);
}

//发布点云图像
static void publishPointCloud(TofFrameData *tofFrameData, UINT32 frame_count)
{
	const UINT32 width = tofFrameData->frameWidth;
	const UINT32 height = tofFrameData->frameHeight;

	pclCloud.points.clear();
	pcl::PointXYZ singlePoint;

	for (int i = 0; i < width * height; ++i)
	{
		singlePoint.x = tofFrameData->pPointData[i].x;
		singlePoint.y = tofFrameData->pPointData[i].y;
		singlePoint.z = tofFrameData->pPointData[i].z;

		pclCloud.points.push_back(singlePoint);
	}

	pcl::toROSMsg(pclCloud, cloudMsg);//将pcl数据类型转化为ros数据类型
	cloudMsg.header.frame_id = "camera_point_cloud_frame";
	cloudMsg.header.stamp = ros::Time::now();
	pub_pointCloud.publish(cloudMsg);
}

//发布深度图像
static void publishDepthzImage(TofFrameData *tofFrameData, UINT32 frame_count)
{
	const UINT32 width = tofFrameData->frameWidth;
	const UINT32 height = tofFrameData->frameHeight;

	cv::Mat depthzMat = cv::Mat(height, width, CV_16UC1);
	for (UINT32 i = 0; i < height; ++i)
	{
		for (UINT32 j = 0; j < width; ++j)
		{
			depthzMat.at<UINT16>(i, j) = static_cast<unsigned short>(tofFrameData->pPointData[i * width + j].z * 1000);
		}
	}
	depthzMsg.reset(new sensor_msgs::Image);
	depthzMsg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depthzMat).toImageMsg();
	depthzMsg->header.frame_id = "camera_depthz_frame";
	depthzMsg->header.stamp = ros::Time::now();
	depthzMsg->height = height;
	depthzMsg->width = width;
	depthzMsg->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
	depthzMsg->is_bigendian = false;
	depthzMsg->step = depthzMsg->width * 2; // 2 bytes per pixel
	pub_depthz.publish(depthzMsg);
}

//发布灰度图像
static void publishGrayImage(TofFrameData *tofFrameData, UINT32 frame_count)
{
	const UINT32 width = tofFrameData->frameWidth;
	const UINT32 height = tofFrameData->frameHeight;
	const GRAY_FORMAT format = tofFrameData->grayFormat;
	void* pGray = tofFrameData->pGrayData;

	unsigned char* pGrayU8 = new unsigned char[width * height];//U8
	CGrayConvert::Gray_2_U8(pGray, width, height, format, pGrayU8);

	cv::Mat grayMat = cv::Mat(height, width, CV_8UC1);
	for (UINT32 i = 0; i < height; ++i)
	{
		for (UINT32 j = 0; j < width; ++j)
		{
			grayMat.at<UINT8>(i, j) = static_cast<unsigned char>(pGrayU8[i * width + j]);
		}
	}
	SAFE_DELETE_ARRY(pGrayU8);


	grayMsg.reset(new sensor_msgs::Image);
	grayMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", grayMat).toImageMsg();
	grayMsg->header.frame_id = "camera_gray_frame";
	grayMsg->header.stamp = ros::Time::now();
	grayMsg->height = height;
	grayMsg->width = width;
	grayMsg->encoding = sensor_msgs::image_encodings::TYPE_8UC1;
	grayMsg->is_bigendian = false;
	grayMsg->step = grayMsg->width * 1; // 1 bytes per pixel
	pub_gray.publish(grayMsg);


}

static void PublishTofFrame(const unsigned int nCaptureIndex, TofFrameData *tofFrameData)
{
	if (NULL != tofFrameData->pPointData)
	{
		publishPointCloud(tofFrameData, nCaptureIndex);//发布点云
		publishDepthzImage(tofFrameData, nCaptureIndex);//发布深度图
	}

	if (NULL != tofFrameData->pGrayData)
	{
		publishGrayImage(tofFrameData, nCaptureIndex);//发布灰度图
	}

}

static void fnTofStream(TofFrameData *tofFrameData, void* pUserData)
{
	UINT32* frame_count = (UINT32*)pUserData;
	const UINT32 width = tofFrameData->frameWidth;
	const UINT32 height = tofFrameData->frameHeight;

	(*frame_count)++;

	//
	const float fDepthZAvg = CalCenterPointDataZAvg(tofFrameData->pPointData, tofFrameData->frameWidth, tofFrameData->frameHeight);
	// printf("[%u], one TOF frame, time=%llu, ts=%llu, center depthZ = %0.3f m.\n", *frame_count, Utils_GetTickCount(), tofFrameData->timeStamp, fDepthZAvg);

	PublishTofFrame((*frame_count), tofFrameData);
	return;

	CaptureTofFrame(std::string("./Capture"), (*frame_count), tofFrameData);

}


static void PrintDevInfo(TofDeviceInfo *pTofDeviceInfo)
{
	printf("Dev Info:==================================\n");
	printf(">>  szDevName=%s.\n", pTofDeviceInfo->szDevName);
	printf(">>  szDevId=%s.\n", pTofDeviceInfo->szDevId);
	printf(">>  szFirmwareVersion=%s.\n", pTofDeviceInfo->szFirmwareVersion);
	printf("Dev Info==================================\n\n");
}

static void GetTofModeList(const UINT32 supportedTOFMode, std::vector<TOF_MODE>& modeList)
{
	const int bitCnt = (sizeof(supportedTOFMode) * 8);
	int tofMode = 0;

	modeList.clear();
	for (int i = 0; i < bitCnt; i++)
	{
		tofMode = 1 << i;
		if (supportedTOFMode & tofMode)
		{
			modeList.push_back((TOF_MODE)tofMode);
		}
	}
}

static TOF_MODE ChoseTofMode(const UINT32 supportedTOFMode)
{
	std::vector<TOF_MODE> tofModeList;
	GetTofModeList(supportedTOFMode, tofModeList);

/*changed*/
	//打印出所有支持的TOF模式，供用户选择
	// printf("chose tof mode from list: \n");
	// printf(">>  number: mode.\n");
	// for (UINT32 i = 0; i < tofModeList.size(); i++)
	// {
	// 	printf(">>  %u: %s.\n", i, TofMode2Str(tofModeList[i]));
	// }

	// if (1 == tofModeList.size())//只有一种模式，直接返回，省去输入的过程
	// {
	// 	return tofModeList[0];
	// }

#if 1
	//这里简单处理，选择第一种支持的模式
	return tofModeList[0];
#else
	//用于选择哪一种tof模式
	while (1)
	{
		printf("input mode (number) >>");

		std::string strInput;
		std::cin >> strInput;

		const UINT32 i = (UINT32)strtol(strInput.c_str(), NULL, 10);
		if (tofModeList.size() > i)
		{
			return tofModeList[i];
		}
		else
		{
			printf("the number is invalid.\n");
		}
	}
#endif

}


static bool OpenStream(HTOFD hTofD, TofDeviceInfo* pCaps, const TOF_MODE tofMode, 
	UINT32* tof_frame_count, UINT32* rgb_frame_count, UINT32* imu_frame_count)
{
	(*tof_frame_count) = 0;
	TOFRET retVal = TOFD_StartTofStream(hTofD, tofMode, fnTofStream, tof_frame_count);
	if (TOFRET_SUCCESS != retVal)
	{
		printf("start TOF stream, [ FAILED ], retVal=0x%08x!!!!!!!!!!!!!!!!!!!!!\n\n", retVal);
		return false;
	}

	return true;

}

static void CloseStream(HTOFD hTofD, TofDeviceInfo* pCaps)
{
	TOFD_StopTofStream(hTofD);
}

static void PrintfTofLensParameter(TofModuleLensParameterV20* pTofLens)
{
	if (NULL == pTofLens)  return;

	const UINT32 nIndex = pTofLens->nIndex;

	if (1 == nIndex)
	{
		TofModuleLensGeneral* pTmp = &(pTofLens->uParam.general);

		printf("Tof Lens Paramter (general):...............................\n");
		printf(">>   fx = %f.\n", pTmp->fx);
		printf(">>   fy = %f.\n", pTmp->fy);
		printf(">>   cx = %f.\n", pTmp->cx);
		printf(">>   cy = %f.\n", pTmp->cy);
		printf(">>   k1 = %f.\n", pTmp->k1);
		printf(">>   k2 = %f.\n", pTmp->k2);
		printf(">>   p1 = %f.\n", pTmp->p1);
		printf(">>   p2 = %f.\n", pTmp->p2);
		printf(">>   k3 = %f.\n", pTmp->k3);
	}
	else if (2 == nIndex)
	{
		TofModuleLensFishEye* pTmp = &(pTofLens->uParam.fishEye);

		printf("Tof  Lens Paramter (fishEye):...............................\n");
		printf(">>   fx = %f.\n", pTmp->fx);
		printf(">>   fy = %f.\n", pTmp->fy);
		printf(">>   cx = %f.\n", pTmp->cx);
		printf(">>   cy = %f.\n", pTmp->cy);
		printf(">>   k1 = %f.\n", pTmp->k1);
		printf(">>   k2 = %f.\n", pTmp->k2);
		printf(">>   k3 = %f.\n", pTmp->k3);
		printf(">>   k4 = %f.\n", pTmp->k4);
	}
	else
	{
		printf("Tof Lens Paramter (index=%u):...............................\n", nIndex);
		printf(">>   unknown, not supported.\n");
	}
}

static void PrintfDepthCalRoi(DepthCalRoi* Roi)
{
	if (NULL == Roi)  return;

	printf("Depth Cal Roi:...............................\n");
	printf(">>    struMax.left=%u.\n", Roi->struMax.left);
	printf(">>    struMax.right=%u.\n", Roi->struMax.right);
	printf(">>    struMax.top=%u.\n", Roi->struMax.top);
	printf(">>    struMax.bottom=%u.\n", Roi->struMax.bottom);
	printf(">>    struDefault.left=%u.\n", Roi->struDefault.left);
	printf(">>    struDefault.right=%u.\n", Roi->struDefault.right);
	printf(">>    struDefault.top=%u.\n", Roi->struDefault.top);
	printf(">>    struDefault.bottom=%u.\n", Roi->struDefault.bottom);
	printf(">>    struCurrent.left=%u.\n", Roi->struCurrent.left);
	printf(">>    struCurrent.right=%u.\n", Roi->struCurrent.right);
	printf(">>    struCurrent.top=%u.\n", Roi->struCurrent.top);
	printf(">>    struCurrent.bottom=%u.\n", Roi->struCurrent.bottom);

}


static void GetOrSetSomeParam(HTOFD hTofD, TofDeviceInfo* pCaps, const TOF_MODE tofMode)
{
	TOFRET retVal = TOFRET_ERROR_OTHER;

	TofDeviceParamV20 struParam;

	//打印出TOF内参
	memset(&struParam, 0, sizeof(struParam));
	struParam.type = TOF_DEV_PARAM_TofLensParameterV20;
	if (TOFRET_SUCCESS == (retVal = TOFD_GetDeviceParamV20(hTofD, &struParam)))
	{
		PrintfTofLensParameter(&(struParam.uParam.struTofLensParameterV20));
	}

	//打印深度计算的区域
	memset(&struParam, 0, sizeof(struParam));
	struParam.type = TOF_DEV_PARAM_DepthCalRoi;
	if (TOFRET_SUCCESS == (retVal = TOFD_GetDeviceParamV20(hTofD, &struParam)))
	{
		PrintfDepthCalRoi(&(struParam.uParam.struDepthCalRoi));
	}
}

static void SaveSomeData(HTOFD hTofD, std::string& strSaveDir)
{
	TOFRET retVal = TOFRET_ERROR_OTHER;
	TofDeviceParamV20 struParam;
	std::string strPath;

	//
	TofDeviceInfo struCaps;
	memset(&struCaps, 0, sizeof(struCaps));
	TOFD_GetDeviceInfo(hTofD, &struCaps);

	//保存标定文件
	memset(&struParam, 0, sizeof(struParam));
	struParam.type = TOF_DEV_PARAM_TofCalibData;
	if (TOFRET_SUCCESS == (retVal = TOFD_GetDeviceParamV20(hTofD, &struParam)))
	{
		TofCalibData* pTmp = &(struParam.uParam.struTofCalibData);
		strPath = (strSaveDir + std::string("/") + std::string(struCaps.szDevId) + std::string(".bin"));
		Utils_SaveBufToFile(pTmp->pData, pTmp->nDataLen, strPath.c_str(), false);
	}


}

static void PrintfCmdUsage()
{
	printf("\ncmd usage:==================================\n");
	printf(">> s: exit demo.\n");
	printf("\n");
}


static void ThreadPublishCameraInfo(HTOFD hTofD, bool* bPublish)
{
	TOFRET retVal = TOFRET_ERROR_OTHER;

	TofDeviceInfo struCaps;
	memset(&struCaps, 0, sizeof(struCaps));
	TOFD_GetDeviceInfo(hTofD, &struCaps);

	TofDeviceParamV20 struParam;
	memset(&struParam, 0, sizeof(struParam));
	struParam.type = TOF_DEV_PARAM_TofLensParameterV20;
	if (TOFRET_SUCCESS != (retVal = TOFD_GetDeviceParamV20(hTofD, &struParam)))
	{
		printf("TOFD_GetDeviceParamV20(TOF_DEV_PARAM_TofLensParameterV20) failed, retVal=0x%08x.\n", retVal);
		return;
	}

	TofModuleLensParameterV20* pTofLens = &(struParam.uParam.struTofLensParameterV20);
	const UINT32 nIndex = pTofLens->nIndex;


	camInfoMsg.reset(new sensor_msgs::CameraInfo);
	camInfoMsg->header.frame_id = "camera_info_frame";
	camInfoMsg->width = struCaps.tofResWidth;
	camInfoMsg->height =  struCaps.tofResHeight;
	camInfoMsg->distortion_model = "plumb_bob";//指定了相机畸变模型，对于大多数相机,"plumb_bob"简单的径向和切向畸变模型就足够了


	if (1 == nIndex)
	{
		TofModuleLensGeneral* pTmp = &(pTofLens->uParam.general);

		camInfoMsg->D = std::vector<double>{pTmp->k1, pTmp->k2, pTmp->p1, pTmp->p2, pTmp->k3};
		camInfoMsg->K = boost::array<double, 9ul>{pTmp->fx, 0, pTmp->cx, 0, pTmp->fy, pTmp->cy, 0, 0, 1};
		camInfoMsg->R = boost::array<double, 9ul>{1, 0, 0, 0, 1, 0, 0, 0, 1};
		camInfoMsg->P = boost::array<double, 12ul>{pTmp->fx, 0, pTmp->cx, 0, 0, pTmp->fy, pTmp->cy, 0, 0, 0, 1, 0};
	}
	else if (2 == nIndex)
	{
		TofModuleLensFishEye* pTmp = &(pTofLens->uParam.fishEye);

		camInfoMsg->D = std::vector<double>{pTmp->k1, pTmp->k2, pTmp->k3, pTmp->k4, 0};
		camInfoMsg->K = boost::array<double, 9ul>{pTmp->fx, 0, pTmp->cx, 0, pTmp->fy, pTmp->cy, 0, 0, 1};
		camInfoMsg->R = boost::array<double, 9ul>{1, 0, 0, 0, 1, 0, 0, 0, 1};
		camInfoMsg->P = boost::array<double, 12ul>{pTmp->fx, 0, pTmp->cx, 0, 0, pTmp->fy, pTmp->cy, 0, 0, 0, 1, 0};
	}
	else
	{
		printf("Lens Paramter (index=%u):...............................\n", nIndex);
		printf(">>   unknown, not supported.\n");
		return;
	}

	while (*bPublish)
	{
		camInfoMsg->header.stamp = ros::Time::now();
		pub_info.publish(camInfoMsg);

		const unsigned long long tick = Utils_GetTickCount();
		do
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(50)); //单位是毫秒
		} while((*bPublish) && (1000 >= (Utils_GetTickCount() - tick)));	
	}

}


static void ThreadTestDemo(HTOFD hTofD, std::string strSaveDir)
{
	TofDeviceInfo struCaps;
	memset(&struCaps, 0, sizeof(struCaps));
	TOFD_GetDeviceInfo(hTofD, &struCaps);
	PrintDevInfo(&struCaps);

	const TOF_MODE tofMode = ChoseTofMode(struCaps.supportedTOFMode);//选择其中一种TOF模式出TOF数据

	GetOrSetSomeParam(hTofD, &struCaps, tofMode);
	
	//
	UINT32 tof_frame_count = 0, rgb_frame_count = 0, imu_frame_count = 0;
	const bool bSuc = OpenStream(hTofD, &struCaps, tofMode, &tof_frame_count, &rgb_frame_count, &imu_frame_count);
	if (bSuc)
	{
		
	}

	//线程里定时发布camerainfo,需要在开流之后才行
	bool bPublishCameraInfo = true;
	std::thread thread_publish_camera_info = std::thread(ThreadPublishCameraInfo, hTofD, &bPublishCameraInfo);


	//处理键盘输入命令
	const std::string strExitCode = "s";
	while (1)
	{
		PrintfCmdUsage();
	    std::string strInput;
		std::cin >> strInput;
		if (strExitCode == strInput)
		{
			bPublishCameraInfo = false;
			break;
		}
		else
		{
			printf("unknown cmd: %s, skip.\n", strInput.c_str());
		}
	}

	//等待线程退出
	CloseStream(hTofD, &struCaps);

	thread_publish_camera_info.join();

	if ("" != strSaveDir)
	{
		SaveSomeData(hTofD, strSaveDir);//保存一些文件，便于问题排查
	}
}


static void DoTestDemo(ros::NodeHandle& nh, TofDeviceDescriptor* pDevsDesc, std::string& strSaveDir)
{
	HTOFD hTofD = TOFD_OpenDevice(pDevsDesc, CallBackTofDeviceStatus, NULL);
	if (NULL == hTofD)
	{
		printf("Open Tof Device failed.\n");
		return;
	}

	TofDeviceInfo struCaps;
	memset(&struCaps, 0, sizeof(struCaps));
	TOFD_GetDeviceInfo(hTofD, &struCaps);


	//init camera info publisher
	pub_info = nh.advertise<sensor_msgs::CameraInfo>("sunny_topic/camera_info", 5);
	//init point clouds publisher
	pub_pointCloud = nh.advertise<sensor_msgs::PointCloud2>("sunny_topic/tof_frame/pointcloud", 5); 
	//init depthz publisher
	pub_depthz = nh.advertise<sensor_msgs::Image>("sunny_topic/tof_frame/depthz", 5);
	//init gray publisher
	pub_gray = nh.advertise<sensor_msgs::Image>("sunny_topic/tof_frame/gray", 5);

	if(struCaps.bRgbDSupported)
	{
		//init rgbd publisher
		pub_rgbd = nh.advertise<sensor_msgs::Image>("sunny_topic/tof_frame/rgbd", 5);
	}

	if(struCaps.bRgbSupported)
	{
		 //init rgb publisher
		pub_rgb = nh.advertise<sensor_msgs::Image>("sunny_topic/rgb_frame/rgb", 5);
	}


	std::thread thread_test = std::thread(ThreadTestDemo, hTofD, strSaveDir);
	thread_test.join();

	TOFD_CloseDevice(hTofD);

}

static UINT32 ChoseDev(const UINT32  dev_cnt)
{
	const UINT32 min_index = 1;
	const UINT32 max_index = dev_cnt;

	if (1 == max_index)//只有一个设备的话就不用选择了
	{
		return max_index;
	}

	UINT32 dev_index = 1;
	while (1)
	{
		printf("please chose a dev (min=%d, max:%d):\n", min_index, max_index);
		printf(">>");

		std::string strInput;
		std::cin >> strInput;

		dev_index = (UINT32)strtol(strInput.c_str(), NULL, 10);
		if ((min_index <= dev_index) && (max_index >= dev_index))
		{
			break;
		}
		printf("invalid dev index:%d.\n", dev_index);
	}

	return dev_index;
}

/*
用例功能简述：选择某一个设备的某一种模式后，进行取流测试，适用linux下的ROS系统。
*/
int main(int argc, char **argv)
{
	printf("*********************start test*************************\n");

	//初始化节点
	ros::init(argc, argv, "publisher_node");

	ros::NodeHandle node_handle;

	std::string strSaveDir = ("");//(".");//用于保存文件的目录，可以自己指定，为空则表示不保存文件

	TofDevInitParam struInitParam;
	memset(&struInitParam, 0, sizeof(struInitParam));
	strncpy(struInitParam.szDepthCalcCfgFileDir, "/home/sunny/catkin_ws/devel/lib/tof_dev_sdk_demo/parameter", sizeof(struInitParam.szDepthCalcCfgFileDir) - 1);
	struInitParam.bSupUsb = true;
	struInitParam.bSupNetWork = false;
	struInitParam.bSupSerialCOM = false;
	if (struInitParam.bSupSerialCOM)
	{
#ifdef WIN32
		//strncpy(struInitParam.szSerialDev, "COM1", sizeof(struInitParam.szSerialDev));//windows下可以不用赋值
#elif defined LINUX 
		strncpy(struInitParam.szSerialDev, "/dev/ttyUSB0", sizeof(struInitParam.szSerialDev));//linux下必须赋值一个实际使用的串口设备，这里随便写了一个
#endif
	}
	struInitParam.bWeakAuthority = false;
	struInitParam.bDisablePixelOffset = false;
	strncpy(struInitParam.szLogFile, "./tof_dev_sdk_log.txt", sizeof(struInitParam.szLogFile));//不赋值则不记录日志到文件
	TOFD_Init(&struInitParam);

	printf("SDK Version: %s.\n", TOFD_GetSDKVersion());

	TofDeviceDescriptor* pDevsDescList = NULL;
	UINT32 dev_num = 0;
	TOFD_SearchDevice(&pDevsDescList, &dev_num);
	if (0 < dev_num )
	{
		const UINT32 dev_index = ChoseDev(dev_num) - 1;//决定测试哪一个设备
		DoTestDemo(node_handle, pDevsDescList + dev_index, strSaveDir);
	}
	else
	{
		printf("can not find tof device!\n");
	}

	TOFD_Uninit();

	printf("*********************stop test*********************\n");

#ifdef WIN32 //防止控制台自动退出而看不到历史日志信息
	printf("please input anything to finish....");
	system("pause");
#endif

	return 0;
}




