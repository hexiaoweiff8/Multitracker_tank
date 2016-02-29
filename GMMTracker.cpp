#include "GMMTracker.h"

GMMTracker::GMMTracker()
{
	int history = 10;
	float varThreshold = 100;
	bool bShadowDetection = false;
	int nmixtures = 6;
	//mog2 = BackgroundSubtractorMOG2(history, varThreshold, bShadowDetection);
	mog2 = createBackgroundSubtractorMOG2(history, varThreshold, bShadowDetection);
	//mog2.set("nmixtures", nmixtures);
	mog2->setNMixtures(nmixtures);
	flag = true;
}

GMMTracker::~GMMTracker()
{

}

void GMMTracker::refineSegments(const Mat& img, const Mat& mask, Mat& dst)
{
	int niters = 3;
	Mat temp;
	dilate(mask, temp, Mat(), Point(-1,-1), niters);
	erode(temp, temp, Mat(), Point(-1,-1), niters*2);
	dilate(temp, temp, Mat(), Point(-1,-1), niters);
	erode(temp, temp, Mat(), Point(-1,-1), 1);

	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours( temp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	dst = Mat::zeros(img.size(), CV_8UC1);
	if( contours.size() == 0 )
		return;

	Scalar color(255);
	for(int idx = 0; idx >= 0; idx = hierarchy[idx][0] )
	{
		drawContours( dst, contours, idx, color, CV_FILLED, 8, hierarchy );
	}
}

void GMMTracker::roi_adjust(const Mat &img, Rect &rec)
{
	if (rec.x < 0)
		rec.x = 0;
	//rec.x = (rec.x < 0) ? 0 : rec.x;

	if (rec.y < 0)
		rec.y = 0;
	//rec.y = (rec.y < 0) ? 0 : rec.y;

	if (rec.x >= img.cols)
		rec.x = img.cols-1;
	//rec.x = (rec.x >= img.cols) ? img.cols-1 : rec.x;

	if (rec.y >= img.rows)
		rec.y = img.rows-1;
	//rec.y = (rec.y >= img.rows) ? img.rows-1 : rec.y;

	if (rec.y + rec.height >= img.rows)
		rec.height = img.rows-1-rec.y;
	//rec.height = (rec.y + rec.height < img.rows) ? rec.height : img.rows-1-rec.y;

	if (rec.x + rec.width >= img.cols)
		rec.width = img.cols-1-rec.x;
	//rec.width = (rec.x + rec.width < img.cols) ? rec.width : img.cols-1-rec.x;
}

vector< vector<Point> > GMMTracker::tracking(const Mat &src)
{
	//blur(src, src, cv::Size(5,5));
	cv::Mat fg;
	double learningRate = 0.0;
	//mog2(src, fg, learningRate);
	mog2->apply(src, fg, learningRate);

	cv::Mat bg;
	//mog2.getBackgroundImage(bg);

	if (1)//(trackBox.size() < 2)
	{		
		cv::Mat refined_fg;
		refineSegments(src, fg, refined_fg);
		//cv::imshow("refined_fg", refined_fg);

		std::vector< std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(refined_fg, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		
		std::vector< std::vector<cv::Point> > res;
		if(contours.size() > 0)
		{
			for (int idx = 0; idx >= 0; idx = hierarchy[idx][0])
			{
				//std::cout << cv::contourArea(contours[idx]) << std::endl;
				if (cv::contourArea(contours[idx]) < 15000 || cv::contourArea(contours[idx]) >200000)
					continue;
				res.push_back(contours[idx]);
			}
		}
		if (res.size() >= trackBox.size())//next>=last one rec increase
		{
			flag = true;
			trackBox = res;
		}
		else //rec decrease
		{
			flag = false;
		}
	}
	else
	{
		cv::Mat src2, src3;
		std::vector< std::vector<cv::Point> > res2;

		for (size_t i = 0; i < trackBox.size(); i++)
		{
			src.copyTo(src2);
			//src.copyTo(src3);
			for (size_t j = 0; j < trackBox.size(); j++)
			{
				if (j != i)
				{
					cv::Rect roi = cv::minAreaRect(cv::Mat(trackBox[j])).boundingRect();
					roi_adjust(src2, roi);
					//bg(roi).copyTo(src2(roi));
					for (int ii = roi.x; ii < roi.x+roi.width; ii++)
					{
						for (int jj = roi.y; jj < roi.y+roi.height; jj++)
						{
							//if (cv::pointPolygonTest(trackBox[j], cv::Point2f(ii, jj), false) > 0)
							{
								src2.at<cv::Vec3b>(jj,ii)[0] = bg.at<cv::Vec3b>(jj,ii)[0];
								src2.at<cv::Vec3b>(jj,ii)[1] = bg.at<cv::Vec3b>(jj,ii)[1];
								src2.at<cv::Vec3b>(jj,ii)[2] = bg.at<cv::Vec3b>(jj,ii)[2];
							}
						}
					}
				}
			}
			//cv::imshow("src2", src2);
			cv::Mat fg2;
			//mog2(src2, fg2, learningRate);
			mog2->apply(src2, fg2, learningRate);
			//cv::imshow("fg2", fg2);
			//cv::waitKey();

			cv::Mat refined_fg2;
			refineSegments(src2, fg2, refined_fg2);

			std::vector< std::vector<cv::Point> > contours2;
			std::vector<cv::Vec4i> hierarchy2;
			cv::findContours(refined_fg2, contours2, hierarchy2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
				
			if(contours2.size() > 0)
			{
				for (int idx = 0; idx >= 0; idx = hierarchy2[idx][0])
				{
						
					if (cv::contourArea(contours2[idx]) < 15000 || cv::contourArea(contours2[idx]) >200000)
					{
						continue;
					}
					bool flag = true;
					for (size_t j = 0; j < res2.size(); j++)
					{
						if (contours2[idx] == res2[j])
						{
							flag = false;
							break;
						}
					}
					if (flag)
					{
						res2.push_back(contours2[idx]);

						//std::cout << cv::contourArea(contours2[idx]) << std::endl;
						//cv::rectangle(src3, box2.boundingRect(), cv::Scalar(0,0,255));
						//imshow("src3", src3);
						//if (frameNo > 435)
						//cv::waitKey();
					}
				}
			}
		}
		if (res2.size() >= trackBox.size())
		{
			flag = true;
			trackBox = res2;
		}
		else
		{
			flag = false;
		}
	}

	return trackBox;
}

void GMMTracker::drawTrackBox(Mat &src)
{
	if(trackBox.size() > 0)
	{
		for (size_t idx = 0; idx < trackBox.size(); idx++)
		{
			Point2f tmp_points[4] = {Point2f(0,0), Point2f(0,0), Point2f(0,0), Point2f(0,0)};
			RotatedRect box = minAreaRect(Mat(trackBox[idx]));
			box.points(tmp_points);
			Point box_points[4] = {Point(0,0), Point(0,0), Point(0,0), Point(0,0)};
			for (int i = 0; i < 4; i++)
				box_points[i] = tmp_points[i];

			for (int i = 0; i < 4; i++)
				line(src, box_points[i], box_points[(i+1)%4], Scalar(0,0,255), 2);
		}
	}
}

void help()
{
	cout << "\n"
				"This program demonstrated a simple method of connected components clean up of background subtraction\n"
				"When the program starts, it begins learning the background.\n"
				"You can toggle background learning on and off by settting the learning rate.\n\n";
}