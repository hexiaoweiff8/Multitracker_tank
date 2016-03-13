#pragma once

#ifndef _STCTRACKER_H
#define _STCTRACKER_H

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class STCTracker
{
public:
	STCTracker();
	~STCTracker();
	void init(const Mat frame, const Rect box);	
	void tracking(const Mat frame, Rect &trackBox, int _frameNum);

private:
	void createHammingWin();
	void complexOperation(const Mat src1, const Mat src2, Mat &dst, int flag = 0);
	void getCxtPriorPosteriorModel(Mat image, Rect rect);
	void learnSTCModel();

private:
	double padding;         //目标周围上下文比例
	double sigma;			// scale parameter (variance)
	double alpha;			// scale parameter
	double beta;			// shape parameter
	double rho;				// learning parameter
	Point center;			// the object position
	Rect cxtRegion;			// context region
	double lambda;
	int num;
	double scale;
	vector<double> maxValue;

	Mat cxtPriorPro;		// prior probability
	Mat cxtPosteriorPro;	// posterior probability
	Mat STModel;			// conditional probability
	Mat STCModel;			// spatio-temporal context model
	Mat hammingWin;			// Hamming window
};

#endif
