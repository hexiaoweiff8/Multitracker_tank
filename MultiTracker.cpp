#include "MultiTracker2.h"
//using namespace cardemo;

MultiTracker2::MultiTracker2()
{
	res[0] = Rect(Point(0,0), Point(0,0));
	res[1] = Rect(Point(0,0), Point(0,0));
	res[2] = Rect(Point(0,0), Point(0,0));
	res[3] = Rect(Point(0,0), Point(0,0));
	res[4] = Rect(Point(0,0), Point(0,0));
	
	res2[0] = Rect(Point(0,0), Point(0,0));
	res2[1] = Rect(Point(0,0), Point(0,0));
	res2[2] = Rect(Point(0,0), Point(0,0));
	res2[3] = Rect(Point(0,0), Point(0,0));
	res2[4] = Rect(Point(0,0), Point(0,0));

	init = true;
	frameNo = 0;
}

MultiTracker2::~MultiTracker2()
{
}

void MultiTracker2::process(Mat &frame)
{
	frameNo++;
	Mat gray;
	cvtColor(frame, gray, CV_RGB2GRAY);

	gTracker.tracking(frame);
	if (gTracker.flag)//rectangle not decrese
	{
		gTracker.drawTrackBox(frame);//draw the background result
		init = true;
	}
	else//rec decrease
	{
		if (init) 
		{
			for (size_t i = 0; i < gTracker.trackBox.size(); i++)
			{
				res[i] = boundingRect(Mat(gTracker.trackBox[i]));
				gTracker.roi_adjust(gray, res[i]);
				res[i].x = cvRound(res[i].x + res[i].width * 0.3);//correct the rectangle
				res[i].y = cvRound(res[i].y + res[i].height * 0.3);
				res[i].width = cvRound(res[i].width * 0.4);
				res[i].height = cvRound(res[i].height * 0.4);
				sTracker[i].init(gray, res[i]);
			}
			frameNo = 1;
			init = false;
		}
		for (size_t i = 0; i < gTracker.trackBox.size(); i++)
		{
			sTracker[i].tracking(gray, res[i], frameNo);
			res2[i].x = cvRound(res[i].x - res[i].width * 0.5);//correct to display
			res2[i].y = cvRound(res[i].y - res[i].height * 0.5);
			res2[i].width = cvRound(res[i].width * 2.0);
			res2[i].height = cvRound(res[i].height * 2.0);
			rectangle(frame, res2[i], Scalar(255, 0, 0), 2);
		}
	}
}