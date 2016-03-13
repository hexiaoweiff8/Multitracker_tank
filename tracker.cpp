//#include "tracker.h"
//#include "Tracker3.hpp"
#include "MultiTracker2.h"
//#include "GMMTracker.h"

using namespace cv;

namespace CarTracker {
	static int frameNo = 0;
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

	int anglediff(int b1, int b2){
		int a1 = b1;
		int a2 = b2;
		if (a1>a2)
			swap(a1, a2);
		if (a2 - a1 < abs(a2 - a1 - 360))
			return a2 - a1;
		else
			return abs(a2 - a1 - 360);
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
		//if the connected box.center in the rect
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
		//find the most nearest to mark_center
		sort(tank_center.begin(),tank_center.end(), compare);

        //printf("registerCar (%d,%d,%d,%d)\n", rect.x, rect.y, rect.width, rect.height);

        CarAllInfo car(_car);
        car.locX = rRects[0].center.x;
        car.locY = rRects[0].center.y;
        car.dir  = 0;
		p->_Cars.push_back(car);

		CarwithHistory carWh(car);
		carWh.relativeX = tank_center[0].x - car.locX;
		carWh.relativeY = tank_center[0].y - car.locY;
		carWh.dirHis.push_back(0);
        p->cars.push_back(carWh);
		//cout << "carWh.loc:" << carWh.locX<<endl<<endl<<endl;
		//cout << "carWh.relativeY" << carWh.relativeY<<endl << endl << endl;
        //p->tracker.enterCar();
        return 0;
    }

	int locatChange(int carNo,double x, double y, void*_h){
        _tracker* p = static_cast<_tracker*>(_h);
		if (x < 0) x = 0;
		else if (x>p->frame.cols) x = p->frame.cols;
		if (y < 0) y = 0;
		else if (y>p->frame.rows) y = p->frame.rows;
		double lx = p->cars[carNo].locX;
		double ly = p->cars[carNo].locY;
		double dis = sqrtf(float((x - lx)*(x - lx)+(y - ly)*(y - ly)));
		if (dis>30)
		{
			p->cars[carNo].locX = lx + (x - lx) * 30 / dis;
			p->cars[carNo].locY = ly + (y - ly) * 30 / dis;
		}
		else
		{
			p->cars[carNo].locX = x;
			p->cars[carNo].locY = y;
		}
		return 0;
	}
	
	Mat drawTrack(void* _h,Mat &src,Mat &frmCp){
        _tracker* p = static_cast<_tracker*>(_h);
		for (size_t i = 0; i < p->cars.size(); i++)
		{
			Point center = Point(cvRound(p->cars[i].locX), cvRound(p->cars[i].locY));
			circle(src, center, 1, Scalar((64 * i)%255, (127*i)%255 , 128), 1, 8, 0);
			if (!p->cars[i].mark_flag)
				rectangle(frmCp, p->cars[i].rectRes, Scalar(255, 0, 0), 1, 8, 0);
			cout << p->cars[i].dir << " ";
		}
        if(p->cars.size())
		    cout << endl;
		return src;
	}

	int distributeConnect(const vector<vector<Point> > &trackBox,void*_h){
		_tracker*p = static_cast<_tracker*>(_h);
		vector<RotatedRect>rRects;
		for (size_t i = 0; i < trackBox.size(); i++)
			rRects.push_back(minAreaRect(trackBox[i]));
		if (rRects.size() < p->cars.size()) return -1;
		for (size_t i = 0; i < rRects.size(); i++)
		{
			rRects[i].angle = int(rRects[i].angle);
			if (rRects[i].angle<0)
			{
				if (rRects[i].size.width<rRects[i].size.height)
					rRects[i].angle = 90 - rRects[i].angle;
				else rRects[i].angle = -rRects[i].angle;
			}
		}

		for (size_t i = 0; i < p->cars.size(); i++)
		{
			//choose the nearest
			mark_center = Point2f(float(p->cars[i].locX), float(p->cars[i].locY));
			sort(rRects.begin(), rRects.end(), comp2Rotat);
			//need to change the angle here***
			if (anglediff(int(rRects[0].angle), int(p->cars[i].dir))<
				anglediff(int(rRects[0].angle) + 180, int(p->cars[i].dir)))
				p->cars[i].dir = rRects[0].angle;
			else p->cars[i].dir = int(rRects[0].angle + 180) % 360;
		}
		return 0;
	}

	int distributeMark(vector<cv::RotatedRect> & rRects,void* _h){
        _tracker* p = static_cast<_tracker*>(_h);
		vector<RotatedRect>rect_neigh;
		Mat frame_gray;
		cvtColor(p->frame, frame_gray, CV_BGR2GRAY);
		//correct the angle
		for (size_t i = 0; i < rRects.size(); i++)
		{
			//cout << rRects[i].angle << " origin" << endl;
			rRects[i].angle = int(rRects[i].angle);
			if (rRects[i].angle<0)
			{
				if (rRects[i].size.width < rRects[i].size.height)
					rRects[i].angle = -rRects[i].angle;
				else rRects[i].angle = 90 - rRects[i].angle;
			}
		}

		for (size_t i = 0; i < p->cars.size(); i++)
		{
			Rect neighborRect = Rect(int(p->cars[i].locX) - 200,int(p->cars[i].locY) - 100, 400, 200);
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
				mark_center = Point2f(float(p->cars[i].locX), float(p->cars[i].locY));
				sort(rect_neigh.begin(), rect_neigh.end(), comp2Rotat);
				locatChange(i, rect_neigh[0].center.x, rect_neigh[0].center.y, _h);
				//p->cars[i].locX = rect_neigh[0].center.x;
				//p->cars[i].locY = rect_neigh[0].center.y;

				//need to change the angle here***
				if (!p->tracker.gTracker.flag)
				{
					if (anglediff(int(rect_neigh[0].angle),int(p->cars[i].dir))<
						anglediff(int(rect_neigh[0].angle)+180,int(p->cars[i].dir)))
						p->cars[i].dir = rect_neigh[0].angle;
					else p->cars[i].dir = int(rect_neigh[0].angle + 180) % 360;
				}
				if (!p->cars[i].mark_flag)//frist gain the mark
					p->cars[i].frameNo = 0;
				p->cars[i].mark_flag = 1;
			}
			else{
				if (p->cars[i].mark_flag)//first lost mark
				{
					p->cars[i].rectRes = Rect(int(p->cars[i].locX+p->cars[i].relativeX-30),
						int(p->cars[i].locY+p->cars[i].relativeY-30),60,60);
					//cout << rect << endl;
					//init the rect to track
					p->tracker.gTracker.roi_adjust(p->frame, p->cars[i].rectRes);
					p->cars[i].stctracker.init(frame_gray, p->cars[i].rectRes);
					p->cars[i].frameNo = 1;
					p->cars[i].stctracker.tracking(frame_gray, p->cars[i].rectRes, p->cars[i].frameNo);
					//locatChange(i, p->cars[i].rectRes.x+p->cars[i].rectRes.width / 2 - p->cars[i].relativeX,
						//p->cars[i].rectRes.y + p->cars[i].rectRes.height / 2 - p->cars[i].relativeY,_h);
					p->cars[i].mark_flag = 0;
				}
				else//lost marks for many frame
				{
					p->cars[i].frameNo++;
					cout << p->cars[i].frameNo << endl;
					p->cars[i].stctracker.tracking(frame_gray, p->cars[i].rectRes, p->cars[i].frameNo);
					//cout << p->cars[i].rectRes << endl;
					locatChange(i, p->cars[i].rectRes.x+p->cars[i].rectRes.width / 2 - p->cars[i].relativeX,
						p->cars[i].rectRes.y + p->cars[i].rectRes.height / 2 - p->cars[i].relativeY,_h);
				}
			}
		}
		return 0;
	}

    // Mat findCar(cv::Mat& frame, std::vector<CarAllInfo>** out, void* _h) {
    int findCar(std::vector<cv::Mat>& inputImages, std::vector<CarAllInfo>** out, algHandle _h){
    // int findCar(cv::Mat& frame, std::vector<CarAllInfo>** out, void* _h) {
        cv::Mat frame = inputImages[0];
		frameNo++;
		_tracker* p = static_cast<_tracker*>(_h);
		//_tracker p = static_cast<_tracker>(_h);
		Mat frmCp = frame.clone();
		static Mat track = Mat::zeros(frame.size(), frame.type());
		static Mat dst = Mat(Size(frame.size().width,frame.size().height*2), frame.type());
		// Mat roit = dst(Rect(Point(0, 0), Point(dst.cols, dst.rows / 2)));
		// Mat roib = dst(Rect(Point(0, dst.rows / 2), Point(dst.cols, dst.rows)));
		p->frame = frame;
		p->tracker.gTracker.findConnect(frmCp,p->cars.size());
        std::vector<cv::RotatedRect> res = p->tracker.gTracker.id_Mark(frmCp,Rect(Point(0,0),Point(frame.cols,frame.rows)));
		if (p->tracker.gTracker.flag)
		{
			distributeConnect(p->tracker.gTracker.trackBox,_h);
			p->tracker.gTracker.drawTrackBox(frmCp);
		}
		distributeMark(res,_h);
		drawTrack(_h,track,frmCp);
		//imshow("frm", frmCp);
		// frmCp.copyTo(roit);
		// track.copyTo(roib);
		// imshow("dst", dst);
		imshow("frmCp", frmCp);
		imshow("track", track);
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
		for (size_t i = 0; i < p->_Cars.size(); i++)
		{
			p->_Cars[i].locX = p->cars[i].locX;
			p->_Cars[i].locY = p->cars[i].locY;
			p->_Cars[i].dir = p->cars[i].dir;
		}

		*out = &p->_Cars;

        // return dst;
        return 0;
    }

    void trackerDestroy(void* _h){
        _tracker* p = static_cast<_tracker*>(_h);
        delete p;
    }
}
