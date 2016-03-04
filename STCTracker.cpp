#include "STCTracker.h"

STCTracker::STCTracker()
{
	padding = 1.0;
	alpha = 2.25;
	beta = 1;
	rho = 0.075;
	scale = 1.0;    //initial scale ratio
	lambda = 0.25;  //Eq.(15)
	num = 5;        //num consecutive frames
}

STCTracker::~STCTracker()
{

}

/************ Create a Hamming window ********************/
void STCTracker::createHammingWin()
{
	for (int i = 0; i < hammingWin.rows; i++)
	{
		for (int j = 0; j < hammingWin.cols; j++)
		{
			hammingWin.at<double>(i, j) = (0.54 - 0.46 * cos( 2 * CV_PI * i / hammingWin.rows )) 
				* (0.54 - 0.46 * cos( 2 * CV_PI * j / hammingWin.cols ));
		}
	}
}

/************ Define two complex-value operation *****************/
void STCTracker::complexOperation(const Mat src1, const Mat src2, Mat &dst, int flag)
{
	CV_Assert(src1.size == src2.size);
	CV_Assert(src1.channels() == 2);

	Mat A_Real, A_Imag, B_Real, B_Imag, R_Real, R_Imag;
	vector<Mat> planes;
	split(src1, planes);
	planes[0].copyTo(A_Real);
	planes[1].copyTo(A_Imag);

	split(src2, planes);
	planes[0].copyTo(B_Real);
	planes[1].copyTo(B_Imag);

	dst.create(src1.rows, src1.cols, CV_64FC2);
	split(dst, planes);
	R_Real = planes[0];
	R_Imag = planes[1];

	for (int i = 0; i < A_Real.rows; i++)
	{
		for (int j = 0; j < A_Real.cols; j++)
		{
			double a = A_Real.at<double>(i, j);
			double b = A_Imag.at<double>(i, j);
			double c = B_Real.at<double>(i, j);
			double d = B_Imag.at<double>(i, j);
			if (flag)
			{
				// division: (a+bj) / (c+dj)
				R_Real.at<double>(i, j) = (a * c + b * d) / (c * c + d * d + 0.000001);
				R_Imag.at<double>(i, j) = (b * c - a * d) / (c * c + d * d + 0.000001);
			}
			else
			{
				// multiplication: (a+bj) * (c+dj)
				R_Real.at<double>(i, j) = a * c - b * d;
				R_Imag.at<double>(i, j) = b * c + a * d;
			}
		}
	}
	merge(planes, dst);
}

/************ Get context prior and posterior probability ***********/
void STCTracker::getCxtPriorPosteriorModel(Mat image,Rect rect)
{
	Mat context = Mat::zeros(rect.height, rect.width, CV_64FC1);
	vector<int> vecx,vecy;//xy×ø±êµÄÈÝÆ÷
	for (int i = rect.x; i < rect.x + rect.width; i++)
	{
		if (i < 0)
		    vecx.push_back(0);
		else if (i > image.cols)
			vecx.push_back(image.cols);		
		else
		    vecx.push_back(i);			
	}
	/*for (int i =0; i < rect.height; i++)
	{
		cout << vecx[i] << endl;
	}*/
	for (int i = rect.y; i < rect.y + rect.height; i++)
	{
		if (i < 0)
			vecy.push_back(0);
		else if (i > image.rows)
			vecy.push_back(image.rows);		
		else
			vecy.push_back(i);			
	}
	double sum_prior(0), sum_post(0);
	for (int i = 0; i < rect.height; i++)
	{
		for (int j = 0; j < rect.width; j++)
		{
			context.at<double>(i, j) =((uchar*)image.data)[vecy[i] * image.cols + vecx[j]];
			double x = j + cxtRegion.x;
			double y = i + cxtRegion.y;
			double dist = sqrt((center.x - x) * (center.x - x) + (center.y - y) * (center.y - y));
			// equation (5) in the paper
			cxtPriorPro.at<double>(i, j) = exp(- dist * dist / (2 * sigma * sigma));
			sum_prior += cxtPriorPro.at<double>(i, j);
			// equation (6) in the paper
			cxtPosteriorPro.at<double>(i, j) = exp(- dist /alpha);
			sum_post += cxtPosteriorPro.at<double>(i, j);
		}
	}
	
	// normalized by subtracting the average intensity of that region
	Scalar average = mean(context);	
	context.convertTo(context, CV_64FC1, 1.0, - average[0]);
	context = context.mul(hammingWin);
	cxtPriorPro.convertTo(cxtPriorPro, -1, 1.0/sum_prior);
	cxtPriorPro = cxtPriorPro.mul(context);
	cxtPosteriorPro.convertTo(cxtPosteriorPro, -1, 1.0/sum_post);
	vecx.clear();
	vecy.clear();
}

/************ Learn Spatio-Temporal Context Model ***********/
void STCTracker::learnSTCModel()
{
	// step 2-1: Execute 2D DFT for prior probability
	Mat priorFourier;
	Mat planes1[] = {cxtPriorPro, Mat::zeros(cxtPriorPro.size(), CV_64F)};
	merge(planes1, 2, priorFourier);
	dft(priorFourier, priorFourier);

	// step 2-2: Execute 2D DFT for posterior probability
	Mat postFourier;
	Mat planes2[] = {cxtPosteriorPro, Mat::zeros(cxtPosteriorPro.size(), CV_64F)};
	merge(planes2, 2, postFourier);
	dft(postFourier, postFourier);

	// step 3: Calculate the division
	Mat conditionalFourier;
	complexOperation(postFourier, priorFourier, conditionalFourier, 1);
	// step 4: Execute 2D inverse DFT for conditional probability and we obtain STModel
	dft(conditionalFourier, STModel, DFT_INVERSE | DFT_REAL_OUTPUT | DFT_SCALE);
	// step 5: Use the learned spatial context model to update spatio-temporal context model
	addWeighted(STCModel, 1.0 - rho, STModel, rho, 0.0, STCModel);
}

/************ Initialize the hyper parameters and models ***********/
void STCTracker::init(const Mat gray, const Rect box)
{
	// initial some parameters
	sigma = 0.5 * (box.width + box.height)*scale;
	// the object position
	center.x = box.x + 0.5 * box.width;
	center.y = box.y + 0.5 * box.height;
	// the context region
	cxtRegion.width =(1 + padding) * box.width;
	cxtRegion.height =(1 + padding) * box.height;
	cxtRegion.x = center.x - cxtRegion.width * 0.5;
	cxtRegion.y = center.y - cxtRegion.height * 0.5;

	//cxtRegion &= Rect(0, 0, gray.cols, gray.rows);
	//printf("image= h:%d w:%d\n",gray.cols,gray.rows);
	//printf("cxtRegion= x:%d y:%d h:%d w:%d\n",cxtRegion.x,cxtRegion.y,cxtRegion.height,cxtRegion.width);	
	// the prior, posterior and conditional probability and spatio-temporal context model
	cxtPriorPro = Mat::zeros(cxtRegion.height, cxtRegion.width, CV_64FC1);//c(x);
	cxtPosteriorPro = Mat::zeros(cxtRegion.height, cxtRegion.width, CV_64FC1);//w();//P(c(z)|O);
	STModel = Mat::zeros(cxtRegion.height, cxtRegion.width, CV_64FC1);
	STCModel = Mat::zeros(cxtRegion.height, cxtRegion.width, CV_64FC1);
	//create a Hamming window
	hammingWin = Mat::zeros(cxtRegion.height, cxtRegion.width, CV_64FC1);
	createHammingWin();
	//Mat gray;
	//cvtColor(frame, gray, CV_RGB2GRAY);
	getCxtPriorPosteriorModel(gray, cxtRegion);
	//learn Spatio-Temporal context model from first frame
	learnSTCModel();
}

/******** STCTracker: calculate the confidence map and find the max position *******/
void STCTracker::tracking(const Mat gray, Rect &trackBox,int _frameNum)
{
	// step 1: Get context prior probability
	getCxtPriorPosteriorModel(gray, cxtRegion);
	// step 2-1: Execute 2D DFT for prior probability
	Mat priorFourier;
	Mat planes1[] = {cxtPriorPro, Mat::zeros(cxtPriorPro.size(), CV_64F)};
	merge(planes1, 2, priorFourier);
	dft(priorFourier, priorFourier);
	// step 2-2: Execute 2D DFT for conditional probability
	Mat STCModelFourier;
	Mat planes2[] = {STCModel, Mat::zeros(STCModel.size(), CV_64F)};
	merge(planes2, 2, STCModelFourier);
	dft(STCModelFourier, STCModelFourier);
	// step 3: Calculate the multiplication
	Mat postFourier;
	complexOperation(STCModelFourier, priorFourier, postFourier, 0);

	// step 4: Execute 2D inverse DFT for posterior probability namely confidence map
	Mat confidenceMap;
	dft(postFourier, confidenceMap, DFT_INVERSE | DFT_REAL_OUTPUT| DFT_SCALE);

	// step 5: Find the max position
	Point point;
	double maxVal;
	minMaxLoc(confidenceMap, 0, &maxVal, 0, &point);
	maxValue.push_back(maxVal);
	//cout << "maxValue: " << maxValue[_frameNum-2] << endl;
	/***********update scale by Eq.(15)**********/
	if (_frameNum%(num+2) == 0)
	{   
		double scale_curr = 0.0;
		for (int k = 0;k < num; k++)
		{
			scale_curr += sqrt(maxValue[_frameNum - k - 1] / maxValue[_frameNum - k - 2]);
		}
		scale = (1 - lambda) * scale + lambda * (scale_curr / num);
		cout << "scale " << scale<< endl;
		sigma = sigma * scale;
	}
	// step 6-1: update center, trackBox and context region
	center.x = cxtRegion.x + point.x;
	center.y = cxtRegion.y + point.y;
	//trackBox.width *= scale;
	//trackBox.height *= scale;
	trackBox.width = cvRound(trackBox.width*scale);
	trackBox.height = cvRound(trackBox.height*scale);
	scale = 1;
	trackBox.x = center.x - 0.5 * trackBox.width;
	trackBox.y = center.y - 0.5 * trackBox.height;
	trackBox &= Rect(0, 0, gray.cols, gray.rows);

	cxtRegion.x = center.x - cxtRegion.width * 0.5;
	cxtRegion.y = center.y - cxtRegion.height * 0.5;
	//cxtRegion &= Rect(0, 0, gray.cols, gray.rows);

	//printf("trackBox= x:%d y:%d h:%d w:%d\n", trackBox.x, trackBox.y, trackBox.height, trackBox.width);
	getCxtPriorPosteriorModel(gray, cxtRegion);
	learnSTCModel();
}