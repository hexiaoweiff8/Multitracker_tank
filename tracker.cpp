//#include "tracker.h"
//#include "Tracker3.hpp"
#include "MultiTracker2.h"
//#include "GMMTracker.h"
#define mark_car_dis 100
#define max_wait_reg 300
#define max_two_dis 30
using namespace cv;

namespace CarTracker {
	class carLoc
	{
	public:
		carLoc(Point2f c,int idx);
		~carLoc();
		Point2f center;
		int index;
		bool getDistrubed;
		int lastDis;
	private:

	};

	carLoc::carLoc(Point2f c,int idx)
        : center(c), index(idx), getDistrubed(false), lastDis(-1)
	{}

	carLoc::~carLoc()
	{
	}

	struct Reginfo{
		CarBaseInfo regCar;
		cv::Rect regRect;
		int tryRegtime;

		Reginfo() :tryRegtime(0){}
	};
	bool regSignal = false;
	Reginfo _regCar;

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
	bool comp2carLoc(carLoc c1, carLoc c2){
		Point2f p1 = c1.center;
		Point2f p2 = c2.center;
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

	int delTarget(void*_h){
        _tracker* p = static_cast<_tracker*>(_h);
		for (size_t i = 0; i < p->cars.size(); i++)
		{
			if (p->cars[i].locX<p->frame.cols/8)
			{
				int locatSize = p->cars[i].locatHis.size();
				int in_flag = 0;
				if (locatSize>20)
				{
					for (size_t j = 0; j < 10-1; j++)
					{
						if (p->cars[i].locatHis[locatSize - 1 - j].x
							>p->cars[i].locatHis[locatSize - 2 - j].x)
							in_flag = 1;
					}
					if (!in_flag)
					{
						vector<CarTracker::CarwithHistory>::iterator itc = p->cars.begin();
						itc += i;
						p->cars.erase(itc);
					}
					//if (p->cars[i].locatHis[locatSize-2].x>
					//	p->cars[i].locatHis[locatSize-1].x&&
					//	p->cars[i].locatHis[locatSize-3].x>
					//	p->cars[i].locatHis[locatSize-2].x)
					//{
					//}
				}
			}
		}
		return 0;
	}

	

    int _registerCar(const CarBaseInfo &_car, const cv::Mat &frame, const cv::Rect &rect, void* _h) {
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
		RotatedRect rRect_tank;
		//if the connected box.center in the rect
		for (size_t i = 0; i < p->tracker.gTracker.conNectBox.size(); i++)
		{
			rRect_tank = minAreaRect(p->tracker.gTracker.conNectBox[i]);
			if (rRect_tank.center.x>rect.tl().x&&rRect_tank.center.x<rect.br().x
				&&rRect_tank.center.y>rect.tl().y&&rRect_tank.center.y < rect.br().y)
				rRects_tank.push_back(rRect_tank);
				//tank_center.push_back(rRects_tank[i].center);
		}
		if (rRects_tank.size() < 1)
		{
			cout << "regist failed! no tank fund" << endl;
			return -2;
		}
		//find the most nearest to mark_center
		sort(rRects_tank.begin(), rRects_tank.end(), comp2Rotat);
		//sort(tank_center.begin(),tank_center.end(), compare);
        //printf("registerCar (%d,%d,%d,%d)\n", rect.x, rect.y, rect.width, rect.height);

		double anGle = 0;
		if (rRects_tank[0].angle<0 || rRects_tank[0].angle == -0)
		{
			if (rRects_tank[0].size.width<rRects_tank[0].size.height)
				anGle = 90 - rRects_tank[0].angle;
			else anGle = -rRects_tank[0].angle;
			if (anGle > 90) anGle += 180;
		}
        CarAllInfo car(_car);
        car.locX = rRects[0].center.x;
        car.locY = rRects[0].center.y;
        car.dir  = 0;
		p->_Cars.push_back(car);

		CarwithHistory carWh(car);
		carWh.lastdir = anGle;
		carWh.relativeX = rRects_tank[0].center.x - car.locX;
		carWh.relativeY = rRects_tank[0].center.y - car.locY;
		carWh.locatHis.push_back(rRects[0].center);
		carWh.dirHis.push_back(0);
        p->cars.push_back(carWh);
		//cout << "carWh.loc:" << carWh.locX<<endl<<endl<<endl;
		//cout << "carWh.relativeY" << carWh.relativeY<<endl << endl << endl;
        //p->tracker.enterCar();
        return 0;
    }

	int registerCar(const CarBaseInfo &_car, const cv::Mat &frame, const cv::Rect &rect, void* _h) {
        _tracker* p = static_cast<_tracker*>(_h);
		regSignal = true;
		_regCar.regCar = _car;
		_regCar.regRect = rect;
		_regCar.tryRegtime++;
		if (!_registerCar(_car, frame, rect, _h)){
			regSignal = false;
			_regCar.tryRegtime = 0;
		}
		return 0;
	}

	int locatChange(int carNo,double x, double y, void*_h){
        _tracker* p = static_cast<_tracker*>(_h);
		if (x < 0) x = 0;
		else if (x>p->frame.cols) x = p->frame.cols-1;
		if (y < 0) y = 0;
		else if (y>p->frame.rows) y = p->frame.rows-1;
		double lx = p->cars[carNo].locX;
		double ly = p->cars[carNo].locY;
		double dis = sqrtf(float((x - lx)*(x - lx)+(y - ly)*(y - ly)));
		//***delete this part?
		if (dis > max_two_dis)
		{
			p->cars[carNo].locX = lx + (x - lx) * max_two_dis / dis;
			p->cars[carNo].locY = ly + (y - ly) * max_two_dis / dis;
		}
		else
		{
			p->cars[carNo].locX = x;
			p->cars[carNo].locY = y;
		}
		if (p->cars[carNo].locatHis.size() > 59) p->cars[carNo].locatHis.pop_front();
		p->cars[carNo].locatHis.push_back(Point2f(p->cars[carNo].locX, p->cars[carNo].locY));
		return 0;
	}
	
	Mat drawTrack(void* _h,Mat &src,Mat &frmCp){
        _tracker* p = static_cast<_tracker*>(_h);
		for (size_t i = 0; i < p->cars.size(); i++)
		{
			for (size_t j = 0; j < p->cars[i].locatHis.size(); j++)
			{
				Point center = static_cast<Point>(p->cars[i].locatHis[j]);
					//Point(cvRound(p->cars[i].locatHis[j].x), cvRound(p->cars[i].locY));
				circle(src, center, 1, p->cars[i].color, 1, 8, 0);
			}
			if (p->cars[i].locatHis.size()>0)
			{
				Point center = static_cast<Point>(p->cars[i].locatHis.back());
				circle(src, center, 2, Scalar::all(255), 1, 8, 0);
			}
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
			//rRects[i].angle = int(rRects[i].angle);
			if (rRects[i].angle<0||rRects[i].angle==-0)
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
		//vector<RotatedRect>rect_neigh;
		Mat frame_gray;
		cvtColor(p->frame, frame_gray, CV_BGR2GRAY);
		//correct the angle
		for (size_t i = 0; i < rRects.size(); i++)
		{
			//cout << rRects[i].angle << " origin" << endl;
			rRects[i].angle = int(rRects[i].angle);
			if (rRects[i].angle<0||rRects[i].angle==-0)
			{
				if (rRects[i].size.width < rRects[i].size.height)
					rRects[i].angle = -rRects[i].angle;
				else rRects[i].angle = 90 - rRects[i].angle;
			}
		}
		vector<carLoc> carSample;
		for (size_t i = 0; i < p->cars.size(); i++)
			carSample.push_back(carLoc(Point2f(p->cars[i].locX, p->cars[i].locY), i));
		if (carSample.size() < 1)
			return -1;
		//distribute each mark if dis<the defined dis
		for (size_t i = 0; i < rRects.size(); i++)
		{
			mark_center = rRects[i].center;
			sort(carSample.begin(), carSample.end(), comp2carLoc);
			int dis = int(sqrt(float((carSample[0].center.x - mark_center.x)*
				(carSample[0].center.x - mark_center.x) +
				(carSample[0].center.y - mark_center.y)*
				(carSample[0].center.y - mark_center.y))));
			if (dis < mark_car_dis)//get mark
			{
				int idx = carSample[0].index;
				if (carSample[0].lastDis<0||dis<carSample[0].lastDis)
				{
					carSample[0].getDistrubed = true;
					locatChange(idx, mark_center.x, mark_center.y, _h);
				}
				//need to change the angle here***when connect is unavailable

				if (!p->tracker.gTracker.flag)
				{
					if (anglediff(int(rRects[i].angle), int(p->cars[i].dir))<
						anglediff(int(rRects[i].angle) + 180, int(p->cars[i].dir)))
						p->cars[i].dir = rRects[i].angle;
					else p->cars[i].dir = int(rRects[i].angle + 180) % 360;
				}
				if (!p->cars[idx].mark_flag)//frist gain the mark
					p->cars[idx].frameNo = 0;
				p->cars[idx].mark_flag = 1;
			}
		}
		//deal the car without the mark
		for (size_t j = 0; j < p->cars.size(); j++)
		{
			if (!carSample[j].getDistrubed)//lost mark
			{
				int i = carSample[j].index;
				if (p->cars[i].mark_flag)//first lost mark
				{
					double rx, ry, thelta;
					thelta = (p->cars[i].dir - p->cars[i].lastdir)/360*2*CV_PI;
					//p->cars[i].lastdir = p->cars[i].dir;
					rx = p->cars[i].relativeX*cos(thelta) - p->cars[i].relativeY*sin(thelta);
					ry = p->cars[i].relativeX*sin(thelta) + p->cars[i].relativeY*cos(thelta);
					p->cars[i].rectRes = Rect(int(p->cars[i].locX + rx - 20),
						int(p->cars[i].locY + ry - 20), 40, 40);
					//cout << rect << endl;
					//init the rect to track
					p->tracker.gTracker.roi_adjust(p->frame, p->cars[i].rectRes);
                    if(p->cars[i].rectRes.width > 20&&p->cars[i].rectRes.height > 20){
					    p->cars[i].stctracker.init(frame_gray, p->cars[i].rectRes);
					    p->cars[i].frameNo = 1;
					    p->cars[i].stctracker.tracking(frame_gray, p->cars[i].rectRes, p->cars[i].frameNo);
                    }
					//locatChange(i, p->cars[i].rectRes.x+p->cars[i].rectRes.width / 2 - p->cars[i].relativeX,
					//p->cars[i].rectRes.y + p->cars[i].rectRes.height / 2 - p->cars[i].relativeY,_h);
					p->cars[i].mark_flag = 0;
				}
				else if(p->cars[i].frameNo > 0)//lost marks for many frames
				{
					p->cars[i].frameNo++;
					cout << p->cars[i].frameNo << endl;
					p->cars[i].stctracker.tracking(frame_gray, p->cars[i].rectRes, p->cars[i].frameNo);
					//cout << p->cars[i].rectRes << endl;
					double rx, ry, thelta;
					thelta = (p->cars[i].dir - p->cars[i].lastdir)/360*2*CV_PI;
					rx = p->cars[i].relativeX*cos(thelta) - p->cars[i].relativeY*sin(thelta);
					ry = p->cars[i].relativeX*sin(thelta) + p->cars[i].relativeY*cos(thelta);
					locatChange(i, p->cars[i].rectRes.x + p->cars[i].rectRes.width / 2 - rx,
						p->cars[i].rectRes.y + p->cars[i].rectRes.height / 2 - ry, _h);
				}

			}
		}

		//for (size_t i = 0; i < p->cars.size(); i++)
		//{
		//	Rect neighborRect = Rect(int(p->cars[i].locX) - 200,int(p->cars[i].locY) - 125, 400, 250);
		//	p->tracker.gTracker.roi_adjust(p->frame, neighborRect);

		//	rect_neigh.clear();
		//	//in the neighbor
		//	for (size_t j = 0; j < rRects.size(); j++)
		//	{
		//		if (rRects[j].center.x > neighborRect.tl().x&&rRects[j].center.x<neighborRect.br().x
		//			&&rRects[j].center.y>neighborRect.tl().y&&rRects[j].center.y < neighborRect.br().y)
		//			rect_neigh.push_back(rRects[j]);
		//	}
		//	//choose the nearest
		//	if (rect_neigh.size()>0)
		//	{ 
		//		mark_center = Point2f(float(p->cars[i].locX), float(p->cars[i].locY));
		//		sort(rect_neigh.begin(), rect_neigh.end(), comp2Rotat);
		//		locatChange(i, rect_neigh[0].center.x, rect_neigh[0].center.y, _h);
		//		//p->cars[i].locX = rect_neigh[0].center.x;
		//		//p->cars[i].locY = rect_neigh[0].center.y;

		//		//need to change the angle here***
		//		if (!p->tracker.gTracker.flag)//connect is unavailable
		//		{
		//			if (anglediff(int(rect_neigh[0].angle),int(p->cars[i].dir))<
		//				anglediff(int(rect_neigh[0].angle)+180,int(p->cars[i].dir)))
		//				p->cars[i].dir = rect_neigh[0].angle;
		//			else p->cars[i].dir = int(rect_neigh[0].angle + 180) % 360;
		//		}
		//		if (!p->cars[i].mark_flag)//frist gain the mark
		//			p->cars[i].frameNo = 0;
		//		p->cars[i].mark_flag = 1;
		//	}
		//	else{
		//		if (p->cars[i].mark_flag)//first lost mark
		//		{
		//			p->cars[i].rectRes = Rect(int(p->cars[i].locX+p->cars[i].relativeX-30),
		//				int(p->cars[i].locY+p->cars[i].relativeY-30),60,60);
		//			//cout << rect << endl;
		//			//init the rect to track
		//			p->tracker.gTracker.roi_adjust(p->frame, p->cars[i].rectRes);
		//			p->cars[i].stctracker.init(frame_gray, p->cars[i].rectRes);
		//			p->cars[i].frameNo = 1;
		//			p->cars[i].stctracker.tracking(frame_gray, p->cars[i].rectRes, p->cars[i].frameNo);
		//			//locatChange(i, p->cars[i].rectRes.x+p->cars[i].rectRes.width / 2 - p->cars[i].relativeX,
		//				//p->cars[i].rectRes.y + p->cars[i].rectRes.height / 2 - p->cars[i].relativeY,_h);
		//			p->cars[i].mark_flag = 0;
		//		}
		//		else//lost marks for many frames
		//		{
		//			p->cars[i].frameNo++;
		//			cout << p->cars[i].frameNo << endl;
		//			p->cars[i].stctracker.tracking(frame_gray, p->cars[i].rectRes, p->cars[i].frameNo);
		//			//cout << p->cars[i].rectRes << endl;
		//			locatChange(i, p->cars[i].rectRes.x+p->cars[i].rectRes.width / 2 - p->cars[i].relativeX,
		//				p->cars[i].rectRes.y + p->cars[i].rectRes.height / 2 - p->cars[i].relativeY,_h);
		//		}
		//	}
		//}
		return 0;
	}

    // Mat findCar(cv::Mat& frame, std::vector<CarAllInfo>** out, void* _h) {
    int findCar(std::vector<cv::Mat>& inputImages, std::vector<CarAllInfo>** out, algHandle _h){
    // Mat findCar(std::vector<cv::Mat>& inputImages, std::vector<CarAllInfo>** out, algHandle _h){
        cv::Mat frame = inputImages[0];
		frameNo++;
		_tracker* p = static_cast<_tracker*>(_h);
		//_tracker p = static_cast<_tracker>(_h);
		Mat frmCp = frame.clone();
		Mat track = Mat::zeros(frame.size(), frame.type());
		static Mat dst = Mat(Size(frame.size().width,frame.size().height*2), frame.type());
		 //Mat roit = dst(Rect(Point(0, 0), Point(dst.cols, dst.rows / 2)));
		 //Mat roib = dst(Rect(Point(0, dst.rows / 2), Point(dst.cols, dst.rows)));
		p->frame = frame;
		if (regSignal)
		{
			_regCar.tryRegtime++;
			if (!_registerCar(_regCar.regCar, frame, _regCar.regRect, _h))
			{
				regSignal = false;
				_regCar.tryRegtime = 0;
			}
			if (_regCar.tryRegtime > max_wait_reg)
			{
				regSignal = false;
				cout << "*********cannot find the car*********" << endl;
			}
		}
		p->tracker.gTracker.findConnect(frmCp,p->cars.size());
        std::vector<cv::RotatedRect> res = p->tracker.gTracker.id_Mark(frmCp,Rect(Point(0,0),Point(frame.cols,frame.rows)));
		if (p->tracker.gTracker.flag)
		{
			distributeConnect(p->tracker.gTracker.trackBox,_h);
			p->tracker.gTracker.drawTrackBox(frmCp);
		}
		distributeMark(res,_h);
		drawTrack(_h,track,frmCp);
		delTarget(_h);
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
