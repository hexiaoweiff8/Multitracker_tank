#pragma once

#ifndef _MULTITRACKER2_H
#define _MULTITRACKER2_H

#include "GMMTracker.h"
#include "STCTracker.h"
#include "CarInfo.hpp"

class MultiTracker2
{
public:
	MultiTracker2();
	~MultiTracker2();
	std::vector<cv::RotatedRect> process(Mat &frame,string alg);

public:
	//MultiTracker obj;
	GMMTracker gTracker;
	STCTracker sTracker[5];//circle to track
	Rect res[5];//stc detect consequnce
	Rect res2[5];//correct the res[5] to display
	bool init;//the signal of stc
	long frameNo;//current frame number
};

namespace CarTracker{

struct LocInfoHis{
	bool mark_flag = 1;
	std::vector<double> dirHis;
	std::deque<Point2f> locatHis;

	double lastdir = 0;
	double relativeX = 200;
	double relativeY = 0;

	int frameNo = 0;
	STCTracker stctracker;
	Rect rectRes;
	Scalar color = Scalar(rand() % 255, rand() % 255, rand()%128+128);
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
Mat findCar(std::vector<cv::Mat>& inputImages, std::vector<CarAllInfo>** out, algHandle _h);
void trackerDestroy(algHandle _h);

} // CarTracker

#endif
