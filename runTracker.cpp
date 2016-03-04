#include "MultiTracker2.h"
//using namespace cardemo;

int main(int argc, char** argv)
{
	//string videoFile = "F:/data/³µÁ¾Êä×ª/2CARS_RLFRLT.avi";
	string videoFile = "e:\\resources\\tank\\cam4\\demo0.avi";
	//string videoFile = "e:\\resources\\tank\\cam4\\99RLFRLT.avi";
	//string videoFile = "e:\\cam4\\2CARS_RLFRLT.avi";
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

	int ex = static_cast<int>(capture.get(CV_CAP_PROP_FOURCC));
	vw.open("res.avi", ex , capture.get(CV_CAP_PROP_FPS), 
		Size(capture.get(CV_CAP_PROP_FRAME_WIDTH),frame.size().height));

	MultiTracker2 mTracker;

	int key;
	while (capture.read(frame))
	{
		//resize(frame, frame, Size(frame.cols/2,frame.rows/2));

		std::vector<cv::RotatedRect> res = mTracker.process(frame,"STC");
		std::cout << res.size() << std::endl;

		imshow("video", frame);
		key = waitKey(1);
		if (key == 27)
			break;
		else if (key == 32)
			waitKey(0);

		vw << frame;
	}
	waitKey(0);
	
	return 0;
}