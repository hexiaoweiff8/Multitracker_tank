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
	//obj = MultiTracker("KCF");
}

MultiTracker2::~MultiTracker2()
{
}

std::vector<cv::RotatedRect> MultiTracker2::process(Mat &frame,string alg)
{
	std::vector<cv::RotatedRect> rRects;
	frameNo++;
	Mat gray,frame_copy;
	cvtColor(frame, gray, CV_RGB2GRAY);
	frame_copy = frame.clone();

	int algorithm;
	if (alg=="KCF")
		algorithm = 2;
	else algorithm = 1;
	//Mat lab, lab3c[3];
	//cvtColor(frame, lab, CV_BGR2Lab);
	//split(lab, lab3c);
	//lab3c[0] *= 2;
	//merge(lab3c,3, lab);
	//cvtColor(lab, frame, CV_Lab2BGR);

	gTracker.tracking(frame);//background differ
	gTracker.id_Mark(frame);
	//imshow("id_mark", gTracker.id_Mark(frame));
	if (gTracker.flag)//rectangle not decrese
	{
		gTracker.drawTrackBox(frame);//draw the background result
		for (size_t i = 0; i < gTracker.trackBox.size(); i++)
			rRects.push_back(cv::minAreaRect(cv::Mat(gTracker.trackBox[i])));
		init = true;
	}
	else//rec decrease
	{
		if (init) 
		{
			Rect2d res2d;
			//if (algorithm==2)
				//obj.clear();
			for (size_t i = 0; i < gTracker.trackBox.size(); i++)
			{
				res[i] = boundingRect(Mat(gTracker.trackBox[i]));
				gTracker.roi_adjust(gray, res[i]);
				res[i].x = cvRound(res[i].x + res[i].width * 0.3);//correct the rectangle
				res[i].y = cvRound(res[i].y + res[i].height * 0.3);
				res[i].width = cvRound(res[i].width * 0.4);
				res[i].height = cvRound(res[i].height * 0.4);
				if (algorithm==2)
				{
					res2d = res[i];
					//obj.add(frame_copy,res2d);
				}
				else if (algorithm == 1)
					sTracker[i].init(gray, res[i]);
			}
			frameNo = 1;
			init = false;
		}

		if (algorithm==2)
		{
			//obj.update(frame_copy);
			//for (size_t i = 0; i < obj.objects.size(); i++)
			//{
			//	res2[i].x = cvRound(obj.objects[i].x - obj.objects[i].width * 0.5);//correct to display
			//	res2[i].y = cvRound(obj.objects[i].y - obj.objects[i].height * 0.5);
			//	res2[i].width = cvRound(obj.objects[i].width * 2.0);
			//	res2[i].height = cvRound(obj.objects[i].height * 2.0);
			//	rectangle(frame, res2[i], Scalar(0, 255, 0), 2);
			//}
		}
		else if (algorithm==1)
		{
			for (size_t i = 0; i < gTracker.trackBox.size(); i++)
			{
			sTracker[i].tracking(gray, res[i], frameNo);
			res2[i].x = cvRound(res[i].x - res[i].width * 0.5);//correct to display
			res2[i].y = cvRound(res[i].y - res[i].height * 0.5);
			res2[i].width = cvRound(res[i].width * 2.0);
			res2[i].height = cvRound(res[i].height * 2.0);
			rectangle(frame, res2[i], Scalar(255, 0, 0), 2);
			cv::RotatedRect tmp;
			tmp.angle = 0;
			tmp.center = (res2[i].tl() + res2[i].br()) / 2;
			tmp.size.width = res2[i].width;
			tmp.size.height = res2[i].height;
			rRects.push_back(tmp);
			}
		}
	}
	return rRects;
}