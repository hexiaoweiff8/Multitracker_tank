#include "MultiTracker2.h"
//using namespace cardemo;

int main(int argc, char** argv)
{
	//string videoFile = "F:/data/³µÁ¾Êä×ª/2CARS_RLFRLT.avi";
	string videoFile = "e:\\resources\\tank\\cam4\\2CARS_RLFRLT.avi";
	VideoCapture capture;
	capture.open(videoFile);
	if (!capture.isOpened())
	{
		cout << "read video failure" << endl;
	}

	VideoWriter vw;
	vw.open("res.avi", CV_FOURCC('M', 'J', 'P', 'G'), capture.get(CV_CAP_PROP_FPS), Size(capture.get(CV_CAP_PROP_FRAME_WIDTH)/2,capture.get(CV_CAP_PROP_FRAME_HEIGHT)/2));

	MultiTracker2 mTracker;

	Mat frame;
	while (capture.read(frame))
	{
		resize(frame, frame, Size(frame.cols/2,frame.rows/2));

		mTracker.process(frame);

		imshow("video", frame);
		if (waitKey(1)==27)
			break;

		vw << frame;
	}
	
	return 0;
}