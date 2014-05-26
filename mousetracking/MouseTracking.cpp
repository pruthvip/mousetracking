#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <Windows.h>
#include<stdio.h>
#include<iostream>
#include<fstream>



using namespace cv;
using namespace std;

int main()
{
	ofstream myfile;
	myfile.open("example.txt");

	KalmanFilter KF(4, 2, 0);
	POINT mousePos;
	GetCursorPos(&mousePos);

	// intialization of KF...
	KF.transitionMatrix = *(Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
	//KF.transitionMatrix = *(Mat_<float>(2, 2) << 1, 0, 1, 0);// , 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
	//setIdentity(KF.transitionMatrix);
	Mat_<float> measurement(2, 1); measurement.setTo(Scalar(0));

	KF.statePre.at<float>(0) = mousePos.x;
	KF.statePre.at<float>(1) = mousePos.y;
	KF.statePre.at<float>(2) = 0;
	KF.statePre.at<float>(3) = 0;
	setIdentity(KF.measurementMatrix);   //H
	setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
	setIdentity(KF.measurementNoiseCov, Scalar::all(10));
	setIdentity(KF.errorCovPost, Scalar::all(.1));
	 

	cout <<"R  =  "<< KF.measurementNoiseCov << "\n\n\n";
	cout <<"Q   =" << KF.processNoiseCov << "\n\n\n";

	myfile <<" P =  \n "<< KF.errorCovPost << "\n\n\n";

	myfile << "R  =  " << KF.measurementNoiseCov << "\n\n\n";
	myfile << "Q   =" << KF.processNoiseCov << "\n\n\n";

	myfile << " P =  \n " << KF.errorCovPost << "\n\n\n";

	//getchar();
	// Image to show mouse tracking
	Mat img(600, 800, CV_8UC3);
	vector<Point> mousev, kalmanv;
	mousev.clear();
	kalmanv.clear();
#define drawCross( center, color, d )                                 \
	line(img, Point(center.x - d, center.y - d), Point(center.x + d, center.y + d), color, 2, CV_AA, 0); \
	line(img, Point(center.x + d, center.y - d), Point(center.x - d, center.y + d), color, 2, CV_AA, 0)
	char key = 0;
	while (1)
	{
		// First predict, to update the internal statePre variable
		
		// x is predicted and P is predicted according to eq 3 and 4 in ramsey
		Mat prediction = KF.predict();
		Point predictPt(prediction.at<float>(0), prediction.at<float>(1));

		myfile << " predictiond matrix " << prediction << "\n";
		myfile << " P =  \n " << KF.errorCovPost << "\n\n";
	//	getchar();
		// Get mouse point
		GetCursorPos(&mousePos);
		measurement(0) = mousePos.x;
		measurement(1) = mousePos.y;


		Point measPt(measurement(0), measurement(1));
		myfile << " Measurement " << measPt << "\n\n";

		// The update phase 
		Mat estimated = KF.correct(measurement);

		myfile << " fusion of estimate n measurement matrix " << estimated << "\n";
		Point statePt(estimated.at<float>(0), estimated.at<float>(1));

		 
//		Point measPt(measurement(0), measurement(1));
		// plot points
		imshow("mouse kalman", img);
		img = Scalar::all(0);

		mousev.push_back(measPt);
		kalmanv.push_back(statePt);
		drawCross(statePt, Scalar(255, 255, 255), 5);
		drawCross(measPt, Scalar(0, 0, 255), 5);

		for (int i = 0; i < mousev.size() - 1; i++)
			line(img, mousev[i], mousev[i + 1], Scalar(255, 255, 0), 1);

		for (int i = 0; i < kalmanv.size() - 1; i++)
			line(img, kalmanv[i], kalmanv[i + 1], Scalar(0, 155, 255), 1);


		key=waitKey(3);
		if (key == 27)
			break;
	}

	char i = 0;
	i=getchar();

	return 0;
}