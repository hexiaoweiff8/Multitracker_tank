#include "MultiTracker2.h"

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
	obj = MultiTracker("KCF");
}

MultiTracker2::~MultiTracker2()
{
}

void MultiTracker2::process(Mat &frame)
{
	frameNo++;
	Mat gray,frame_copy;
	cvtColor(frame, gray, CV_RGB2GRAY);
	frame_copy = frame.clone();

	gTracker.tracking(frame);//background differ
	if (gTracker.flag)//rectangle not decrese
	{
		gTracker.drawTrackBox(frame);//draw the background result
		init = true;
	}
	else//rec decrease
	{
		if (init) 
		{
			Rect2d res2d;
			obj.clear();
			//while (obj.objects.size())
			//	obj.del(0);
			for (size_t i = 0; i < gTracker.trackBox.size(); i++)
			{
				res[i] = boundingRect(Mat(gTracker.trackBox[i]));
				gTracker.roi_adjust(gray, res[i]);
				res[i].x = cvRound(res[i].x + res[i].width * 0.3);//correct the rectangle
				res[i].y = cvRound(res[i].y + res[i].height * 0.3);
				res[i].width = cvRound(res[i].width * 0.4);
				res[i].height = cvRound(res[i].height * 0.4);
				res2d = res[i];
				obj.add(frame_copy,res2d);
/*				sTracker[i].init(gray, res[i])*/;
			}
			frameNo = 1;
			init = false;
		}
		//else
		//{
			obj.update(frame_copy);
			for (size_t i = 0; i < obj.objects.size(); i++)
			{
				res2[i].x = cvRound(obj.objects[i].x - obj.objects[i].width * 0.5);//correct to display
				res2[i].y = cvRound(obj.objects[i].y - obj.objects[i].height * 0.5);
				res2[i].width = cvRound(obj.objects[i].width * 2.0);
				res2[i].height = cvRound(obj.objects[i].height * 2.0);
				rectangle(frame, res2[i], Scalar(255, 0, 0), 2);
			}

		//}
		//for (size_t i = 0; i < gTracker.trackBox.size(); i++)
		//{
		//	sTracker[i].tracking(gray, res[i], frameNo);
		//	res2[i].x = cvRound(res[i].x - res[i].width * 0.5);//correct to display
		//	res2[i].y = cvRound(res[i].y - res[i].height * 0.5);
		//	res2[i].width = cvRound(res[i].width * 2.0);
		//	res2[i].height = cvRound(res[i].height * 2.0);
		//	rectangle(frame, res2[i], Scalar(255, 0, 0), 2);
		//}
	}
}