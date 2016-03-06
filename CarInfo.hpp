#ifndef CARTRACKER_TYPE_HPP_
#define CARTRACKER_TYPE_HPP_
#include <opencv2/opencv.hpp>
#include <string>
#include "MultiTracker2.h"

namespace CarTracker{


struct CarBaseInfo {
    std::string type;
    std::string ID;
	CarBaseInfo(std::string ty, std::string id){
		type = ty; 
		ID = id; 
	}
};

struct LocInfo {
    double  locX;
    double  locY;
    double  dir;
};

struct LocInfoHis{
	bool mark_flag = 1;
	std::vector<double> dirHis;
	double relativeX = 500;
	double relativeY = -500;
};

struct CarAllInfo : public CarBaseInfo, public LocInfo {
    //CarAllInfo() {}
    CarAllInfo(const CarBaseInfo& _cbi) : CarBaseInfo(_cbi) {}
};

struct CarwithHistory :public CarAllInfo,public LocInfoHis{
	//CarwithHistory(){}
	CarwithHistory(const CarAllInfo& _cai) :CarAllInfo(_cai){}
};

struct _tracker{
	std::vector<CarwithHistory> cars;
	std::vector<CarAllInfo> _Cars;
	cv::Mat                frame;
	MultiTracker2          tracker;
	//Tracker                 tracker;
};

typedef void* algHandle;

algHandle trackerInit();
int registerCar(const CarBaseInfo &_car, const cv::Mat &frame, const cv::Rect &rect, algHandle _h);
int findCar(cv::Mat& inputImages, std::vector<CarAllInfo>** out, algHandle _h);
void trackerDestroy(algHandle _h);
//int distributeMark(cv::RotatedRect rRects,void* _h);

}

#endif
