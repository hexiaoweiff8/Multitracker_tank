#pragma once

#ifndef _GMMTRACKER_H
#define _GMMTRACKER_H

#include <opencv2\opencv.hpp>

using namespace cv;
using namespace std;

class GMMTracker
{
public:
	GMMTracker();
	~GMMTracker();
	void refineSegments(const Mat& src, const Mat& mask, Mat& dst);
	void roi_adjust(const Mat &img, Rect &rec);
	vector< vector<Point> > tracking(const Mat &img);
	Mat id_Mark(Mat &img);
	void drawTrackBox(Mat &img);
	void help();

private:
	Ptr<BackgroundSubtractorMOG2> mog2;
	//BackgroundSubtractorMOG2 mog2;

public:
	vector< vector<Point> > trackBox;
	bool flag;
};

#endif