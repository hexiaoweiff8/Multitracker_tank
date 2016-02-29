#pragma once

#ifndef _MULTITRACKER2_H
#define _MULTITRACKER2_H

#include "GMMTracker.h"
#include "STCTracker.h"

class MultiTracker2
{
public:
	MultiTracker2();
	~MultiTracker2();
	void process(Mat &frame);

public:
	GMMTracker gTracker;
	STCTracker sTracker[5];//circle to track
	Rect res[5];//stc detect consequnce
	Rect res2[5];//correct the res[5] to display
	bool init;//the signal of stc
	long frameNo;//current frame number
};

#endif