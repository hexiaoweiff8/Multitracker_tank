#include "CarInfo.hpp"
//#include "Tracker3.hpp"
#include "MultiTracker2.h"
//#include "GMMTracker.h"

using namespace cv;

namespace CarTracker {

    struct _tracker{
		std::vector<CarwithHistory> cars;
        std::vector<CarAllInfo> _Cars;
        cv::Mat                frame;
        MultiTracker2          tracker;
        //Tracker                 tracker;
    };

    algHandle trackerInit();
    int registerCar(const CarBaseInfo &_car, const cv::Mat &frame, const cv::Rect &rect, algHandle _h);
    int findCar(std::vector<cv::Mat>& inputImages, std::vector<CarAllInfo>** out, algHandle _h);
    void trackerDestroy(algHandle _h);

    void* trackerInit() {
        return new _tracker();
    }

    int registerCar(const CarBaseInfo &_car, const cv::Mat &frame, const cv::Rect &rect, void* _h) {
        _tracker* p = static_cast<_tracker*>(_h);
		Mat img = frame.clone();
		vector<RotatedRect>rRects = p->tracker.gTracker.id_Mark(img, rect);
		if (rRects.size()!=1)
		{
			cout << "regist failed! Mark " ;
			if (rRects.size() == 0)
				cout << "no mark fund" << endl;
			return -1;
		}
		img = frame.clone();
		vector<RotatedRect>rRects_tank = p->tracker.gTracker.tracking(img, Rect(Point(0,0),Point(frame.cols/3,frame.rows)));
		if (rRects_tank.size()!=1)
		{
			cout << "regist failed! tank";
			if (rRects_tank.size() == 0)
				cout << "no tank fund" << endl;
			return -1;
		}

        //printf("registerCar (%d,%d,%d,%d)\n", rect.x, rect.y, rect.width, rect.height);

        CarAllInfo car(_car);
        car.locX = rRects[0].center.x;
        car.locY = rRects[0].center.y;
        car.dir  = 0;
		CarwithHistory carWh(car);
		carWh.relativeX = rRects_tank[0].center.x - car.locX;
		carWh.relativeY = rRects_tank[0].center.y - car.locY;
		carWh.dirHis.push_back(0);
		p->_Cars.push_back(car);
        p->cars.push_back(carWh);
		//cout << "carWh.loc:" << carWh.locX<<endl<<endl<<endl;
		cout << "carWh.relativeY" << carWh.relativeY<<endl << endl << endl;
        //p->tracker.enterCar();
        return 0;
    }

    int findCar(std::vector<cv::Mat>& frames, std::vector<CarAllInfo>** out, void* _h) {
        _tracker* p = static_cast<_tracker*>(_h);

        Mat &frame = frames[0];
        std::vector<cv::RotatedRect> res = p->tracker.process(frame,"STC");

        fprintf(stderr, "cars(%d), res(%d)\n", p->cars.size(), res.size());

        for(int i = 0; i < res.size(); i++){
            cv::RotatedRect box = res[i];
            p->cars[i].locX = box.center.x;
            p->cars[i].locY = box.center.y;
            if(box.size.width < box.size.height){
                p->cars[i].dir = -90-box.angle;
            }else{
                p->cars[i].dir = -box.angle;
            }
        }

        *out = &p->_Cars;
		//*out = &p->cars;

        return 0;
    }

    void trackerDestroy(void* _h){
        _tracker* p = static_cast<_tracker*>(_h);
        delete p;
    }
}
