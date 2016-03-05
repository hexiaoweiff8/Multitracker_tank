#ifndef CARTRACKER_TYPE_HPP_
#define CARTRACKER_TYPE_HPP_
#include <opencv2/opencv.hpp>
#include <string>

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

typedef void* algHandle;

algHandle trackerInit();
int registerCar(const CarBaseInfo &_car, const cv::Mat &frame, const cv::Rect &rect, algHandle _h);
int findCar(std::vector<cv::Mat>& inputImages, std::vector<CarAllInfo>** out, algHandle _h);
void trackerDestroy(algHandle _h);

}

#endif
