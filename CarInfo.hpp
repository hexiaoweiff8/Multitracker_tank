#ifndef CARTRACKER_TYPE_HPP_
#define CARTRACKER_TYPE_HPP_
#include <opencv2/opencv.hpp>
#include <string>
#include "MultiTracker2.h"

namespace CarTracker{


struct CarBaseInfo {
    std::string type;
    std::string ID;
};

struct LocInfo {
    double  locX;
    double  locY;
    double  dir;
};

struct LocInfoHis{
	bool mark_flag = 1;
	std::vector<double> dirHis;
	double relativeX = 200;
	double relativeY = 0;
	int frameNo = 0;
	STCTracker stctracker;
	Rect rectRes;
};

struct CarAllInfo : public CarBaseInfo, public LocInfo {
    CarAllInfo() {}
    CarAllInfo(const CarBaseInfo& _cbi) : CarBaseInfo(_cbi) {}
};

struct CarwithHistory :public CarAllInfo,public LocInfoHis{
	CarwithHistory(){}
	CarwithHistory(const CarAllInfo& _cai) :CarAllInfo(_cai){}
};

struct _tracker{
	std::vector<CarwithHistory> cars;
	std::vector<CarAllInfo> _Cars;
	cv::Mat                frame;
	MultiTracker2          tracker;
	//Tracker                 tracker;
	_tracker() :cars(std::vector<CarwithHistory>()), 
				_Cars(std::vector<CarAllInfo>()),
				frame(cv::Mat()),
				tracker(MultiTracker2())
	{}
};

typedef void* algHandle;

algHandle trackerInit();
int registerCar(const CarBaseInfo &_car, const cv::Mat &frame, const cv::Rect &rect, algHandle _h);
Mat findCar(cv::Mat& inputImages, std::vector<CarAllInfo> &out, algHandle _h);
void trackerDestroy(algHandle _h);
//int distributeMark(cv::RotatedRect rRects,void* _h);

}

#endif
