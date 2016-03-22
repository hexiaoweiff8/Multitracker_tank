#include "MultiTracker2.h"
#include "CarInfo.hpp"
//using namespace cardemo;

int main(int argc, char** argv)
{
	//string videoFile = "F:/data/³µÁ¾Êä×ª/2CARS_RLFRLT.avi";
	//string videoFile = "e:\\resources\\tank\\cam4\\demo0.avi";
	string videoFile = "e:\\resources\\tank\\20160312\\s1.avi";
	//string videoFile = "e:\\resources\\tank\\res.avi";
	//string videoFile = "e:\\resources\\tank\\cam4\\99R_LF.avi";
	//string videoFile = "e:\\cam4\\2CARS_RLFRLT.avi";
	//string videoFile = "e:\\carTrack\\cam4\\RES.avi";
	VideoCapture capture;
	capture.open(videoFile);
	if (!capture.isOpened())
	{
		cout << "read video failure" << endl;
	}

	VideoWriter vw;
    //CV_FOURCC('M', 'J', 'P', 'G'),
	Mat frame;
	capture.read(frame);
	//resize(frame, frame, Size(frame.cols / 4, frame.rows / 4));

	int ex = static_cast<int>(capture.get(CV_CAP_PROP_FOURCC));
	vw.open("res.avi", ex , capture.get(CV_CAP_PROP_FPS), 
		Size(frame.size().width,frame.size().height*2));

	//MultiTracker2 mTracker;

	CarTracker::algHandle ip = CarTracker::trackerInit();

	int key;
	int carNum = 0;
	vector<CarTracker::CarAllInfo> *out;
	while (capture.read(frame))
	{
		//std::vector<cv::RotatedRect> res = p->tracker.process(frame,"STC");

		//resize(frame, frame, Size(frame.cols / 4, frame.rows / 4));
		vector<Mat>frames;
		frames.push_back(frame);
		key = waitKey(1);
		if (key == 27)
			break;
		else if (key == 32)
			waitKey(0);
		else if (key == 13)
		{
			CarTracker::CarBaseInfo cb;
			cb.type = to_string(carNum);
			cb.ID = to_string(carNum);
			if (!CarTracker::registerCar(cb,
				frame, Rect(Point(0, 0), Point(frame.cols/4, frame.rows)), ip))
				carNum++;
		}
		//Mat dst = CarTracker::findCar(frames, &out, ip);
		Mat dst = CarTracker::findCar(frames, &out, ip);
		//std::vector<cv::RotatedRect> res = mTracker.process(frame,"STC");
		//std::cout << res.size() << std::endl;

		vw << dst;
	}
	waitKey(0);

	CarTracker::trackerDestroy(ip);

	
	return 0;
}