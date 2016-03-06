#include "CarInfo.hpp"
//#include "Tracker3.hpp"
#include "MultiTracker2.h"
//#include "GMMTracker.h"

using namespace cv;

namespace CarTracker {
	Point2f mark_center;
	bool compare(Point2f p1, Point2f p2){
		//cout << mark_center;
		return (p1.x - mark_center.x)*(p1.x - mark_center.x)+
			(p1.y - mark_center.y)*(p1.y - mark_center.y) <
			(p2.x - mark_center.x)*(p2.x - mark_center.x) +
			(p2.y - mark_center.y)*(p2.y - mark_center.y);
	}
	bool comp2Rotat(RotatedRect r1, RotatedRect r2){
		//cout << mark_center;
		Point2f p1 = r1.center;
		Point2f p2 = r2.center;
		return (p1.x - mark_center.x)*(p1.x - mark_center.x) +
			(p1.y - mark_center.y)*(p1.y - mark_center.y) <
			(p2.x - mark_center.x)*(p2.x - mark_center.x) +
			(p2.y - mark_center.y)*(p2.y - mark_center.y);
	}

    //algHandle trackerInit();
    //int registerCar(const CarBaseInfo &_car, const cv::Mat &frame, const cv::Rect &rect, algHandle _h);
    //int findCar(std::vector<cv::Mat>& inputImages, std::vector<CarAllInfo>** out, algHandle _h);
    //void trackerDestroy(algHandle _h);

    void* trackerInit() {
        return new _tracker();
    }

    int registerCar(const CarBaseInfo &_car, const cv::Mat &frame, const cv::Rect &rect, void* _h) {
        _tracker* p = static_cast<_tracker*>(_h);
		Mat img = frame.clone();
		vector<RotatedRect>rRects = p->tracker.gTracker.id_Mark(img, rect);
		if (rRects.size()!=1)
		{
			cout << "regist failed! Mark:" ;
			if (rRects.size() == 0)
				cout << "none" << endl;
			return -1;
		}
		mark_center = rRects[0].center;
		vector<Point2f>tank_center;
		vector<RotatedRect>rRects_tank;
		for (size_t i = 0; i < p->tracker.gTracker.conNectBox.size(); i++)
		{
			rRects_tank.push_back(minAreaRect(p->tracker.gTracker.conNectBox[i]));
			if (rRects_tank[i].center.x>rect.tl().x&&rRects_tank[i].center.x<rect.br().x
				&&rRects_tank[i].center.y>rect.tl().y&&rRects_tank[i].center.y < rect.br().y)
				tank_center.push_back(rRects_tank[i].center);
		}
		if (tank_center.size() < 1)
		{
			cout << "regist failed! no tank fund" << endl;
			return -2;
		}
		sort(tank_center.begin(),tank_center.end(), compare);

        //printf("registerCar (%d,%d,%d,%d)\n", rect.x, rect.y, rect.width, rect.height);

        CarAllInfo car(_car);
        car.locX = rRects[0].center.x;
        car.locY = rRects[0].center.y;
        car.dir  = 0;
		CarwithHistory carWh(car);
		carWh.relativeX = tank_center[0].x - car.locX;
		carWh.relativeY = tank_center[0].y - car.locY;
		carWh.dirHis.push_back(0);
		p->_Cars.push_back(car);
        p->cars.push_back(carWh);
		//cout << "carWh.loc:" << carWh.locX<<endl<<endl<<endl;
		cout << "carWh.relativeY" << carWh.relativeY<<endl << endl << endl;
        //p->tracker.enterCar();
        return 0;
    }
	
	Mat drawTrack(void* _h,Mat &src){
        _tracker* p = static_cast<_tracker*>(_h);
		for (size_t i = 0; i < p->cars.size(); i++)
		{
			Point center = Point(cvRound(p->cars[i].locX), cvRound(p->cars[i].locY));
			circle(src, center, 2, Scalar(255 * i, 255, 128*i), 1, 8, 0);
		}
		imshow("track", src);
		return src;
	}

	int distributeMark(vector<cv::RotatedRect> rRects,void* _h){
        _tracker* p = static_cast<_tracker*>(_h);
		vector<RotatedRect>rect_neigh;
		//correct the angle
		for (size_t i = 0; i < rRects.size(); i++)
		{
			if (1){
				//if (boundRect[i].size.width < boundRect[i].size.height){
				if (rRects[i].angle > -45)
					rRects[i].angle = -rRects[i].angle;
				else rRects[i].angle = 90 - rRects[i].angle;
			}
			else
			{
				if (rRects[i].angle < -45)
					rRects[i].angle = -rRects[i].angle;
				else rRects[i].angle = 90 - rRects[i].angle;
			}
		}

		for (size_t i = 0; i < p->cars.size(); i++)
		{
			Rect neighborRect = Rect(p->cars[i].locX - 200, p->cars[i].locY - 100, 400, 200);
			p->tracker.gTracker.roi_adjust(p->frame, neighborRect);

			rect_neigh.clear();
			//in the neighbor
			for (size_t j = 0; j < rRects.size(); j++)
			{
				if (rRects[j].center.x > neighborRect.tl().x&&rRects[j].center.x<neighborRect.br().x
					&&rRects[j].center.y>neighborRect.tl().y&&rRects[j].center.y < neighborRect.br().y)
					rect_neigh.push_back(rRects[j]);
			}
			//choose the nearest
			if (rect_neigh.size()>0)
			{ 
				mark_center = Point2f(p->cars[i].locX, p->cars[i].locY);
				sort(rect_neigh.begin(), rect_neigh.end(), comp2Rotat);
				p->cars[i].locX = rect_neigh[0].center.x;
				p->cars[i].locY = rect_neigh[0].center.y;
				//need to change the angle here***
				p->cars[i].mark_flag = 1;
			}
			else p->cars[i].mark_flag = 0;
		}
		return 0;
	}

    int findCar(cv::Mat& frame, std::vector<CarAllInfo>** out, void* _h) {
        _tracker* p = static_cast<_tracker*>(_h);
		Mat frmCp = frame.clone();
		Mat track = Mat::zeros(frame.size(), frame.type());
		p->frame = frame;
		p->tracker.gTracker.findConnect(frmCp);
        std::vector<cv::RotatedRect> res = p->tracker.gTracker.id_Mark(frmCp,Rect(Point(0,0),Point(frame.cols,frame.rows)));
		if (res.size()>0)
			distributeMark(res,_h);
		drawTrack(_h,track);
        //Mat &frame = frames[0];
        //std::vector<cv::RotatedRect> res = p->tracker.process(frame,"STC");

        //fprintf(stderr, "cars(%d), res(%d)\n", p->cars.size(), res.size());

        //for(int i = 0; i < res.size(); i++){
        //    cv::RotatedRect box = res[i];
        //    p->cars[i].locX = box.center.x;
        //    p->cars[i].locY = box.center.y;
        //    if(box.size.width < box.size.height){
        //        p->cars[i].dir = -90-box.angle;
        //    }else{
        //        p->cars[i].dir = -box.angle;
        //    }
        //}

        //*out = &p->_Cars;
		//*out = &p->cars;

        return 0;
    }

    void trackerDestroy(void* _h){
        _tracker* p = static_cast<_tracker*>(_h);
        delete p;
    }
}
