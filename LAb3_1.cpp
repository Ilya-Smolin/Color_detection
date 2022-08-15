#include <opencv2\opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include "cvDirectory.h"
#include <string>


using namespace cv;
using namespace std;


int th = 0;
Mat img, thimg, cntrs, gk, gk_cntrs, gks;
Mat red_img;
Mat blue_img;
Mat green_img;
Mat lamp_img;
Mat img_clone;
Mat adjust_image;

vector<vector<Point>> red_contours;
vector<vector<Point>> blue_contours;
vector<vector<Point>> green_contours;
vector<vector<Point>> red_contours_upd;
vector<vector<Point>> green_contours_upd;
vector<vector<Point>> blue_contours_upd;
vector<vector<Point>> lamp_contours;
vector<vector<Point>> contours;
vector<vector<Point>> gk_contour;
vector<vector<Point>> gk_contours;

vector<Point> find_largest_contour(vector<vector<Point>> vect) {
	vector<Point> answer;
	double biggest_contour = 0;
	for (int i = 0; i < vect.size(); i++)
	{
		double new_contour = contourArea(vect[i]);
		if (new_contour > biggest_contour) {
			biggest_contour = new_contour;
			answer = vect[i];
		}
	}
	//cout << answer << endl;
	return answer;
}



void proc_img(int, void* user_data) {
	int* th_type = (int*)user_data;
	threshold(img, thimg, th, 255, *th_type);
	threshold(img, cntrs, 255, 255, 0);
	cvtColor(cntrs, cntrs, CV_GRAY2BGR);
	imshow("thimg", thimg);
	findContours(thimg.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	drawContours(cntrs, find_largest_contour(contours), -1 , Scalar(255, 255, 255), 1);
	Moments contour_moment = moments(find_largest_contour(contours));
	Point center_point = Point2f(
		contour_moment.m10 / contour_moment.m00,
		contour_moment.m01 / contour_moment.m00
	);
	circle(cntrs, center_point, 2, Scalar(0, 255, 0), FILLED);
	imshow("contours", cntrs);
}


void proc_img_1(int, void* user_data) {
	int* th_type = (int*)user_data;
	threshold(img, thimg, th, 255, *th_type);
	threshold(img, cntrs, 255, 255, 0);
	cvtColor(cntrs, cntrs, CV_GRAY2BGR);
	imshow("thimg", thimg);
	findContours(thimg.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	drawContours(cntrs, find_largest_contour(contours), -1, Scalar(255, 255, 255), 1);
	Moments contour_moment = moments(find_largest_contour(contours));
	Point center_point = Point2f(
		contour_moment.m10 / contour_moment.m00,
		contour_moment.m01 / contour_moment.m00
	);
	circle(cntrs, center_point, 2, Scalar(0, 255, 0), FILLED);
	imshow("contours", cntrs);
}


void task_1(string image_name) {
	img = imread(image_name, IMREAD_GRAYSCALE);
	//GaussianBlur(img, img, Size(25, 25), 0);
	imshow("img.jpg", img);
	//int th_type = THRESH_OTSU;
	int th_type = THRESH_BINARY;
	proc_img(0, &th_type);
	createTrackbar("th", "contours", &th, 255, proc_img, &th_type);
	waitKey();
}


void task_2(string image_name) {
	img = imread(image_name, IMREAD_COLOR);
	imshow("img_color.jpg", img);
	vector<Mat> bgr_channels;
	split(img, bgr_channels);
	Mat red_channel = bgr_channels[2];
	int th_type = THRESH_OTSU;
	imshow("red", red_channel);
	Mat green_channel = bgr_channels[1];
	imshow("green", green_channel);
	Mat blue_channel = bgr_channels[0];
	imshow("blue", blue_channel);
	img = red_channel - blue_channel - green_channel;
	imshow("diff", img);
	proc_img_1(0, &th_type);
	//createTrackbar("th", "red", &th, 255, proc_img_1, &th_type);
	waitKey();
}

Scalar red_high = Scalar(255, 98, 255);
Scalar red_low = Scalar(159, 35, 213);

Scalar red_high_1 = Scalar(179, 255, 255);
Scalar red_low_1 = Scalar(118, 100, 0);

Scalar blue_low = Scalar(75, 47, 35);
Scalar blue_high = Scalar(140, 255, 255);

Scalar green_low = Scalar(38, 63, 117);
Scalar green_high = Scalar(80, 255, 255);

Scalar lamp_low = Scalar(68, 38, 218);
Scalar lamp_high = Scalar(255, 108, 255);

int Hue_Lower_Value = 0;//initial hue value(lower)//
int Hue_Lower_Upper_Value = 22;//initial hue value(upper)//
int Saturation_Lower_Value = 0;//initial saturation(lower)//
int Saturation_Upper_Value = 255;//initial saturation(upper)//
int Value_Lower = 0;//initial value (lower)//
int Value_Upper = 255;//initial saturation(upper)//

double euclideanDist(cv::Point2f a, cv::Point2f b){
	cv::Point2f diff = a - b;
	return cv::sqrt(diff.x * diff.x + diff.y * diff.y);
}

Point2f return_closest_robot(vector<vector<Point>> vect) {
	Point closest_point;
	double smallest_distance = DBL_MAX;
	Moments contour_moment = moments(find_largest_contour(lamp_contours));
	Point center_point = Point2f(
		contour_moment.m10 / contour_moment.m00,
		contour_moment.m01 / contour_moment.m00
	);
	
	for (size_t i = 0; i < vect.size(); i++){;
		Moments current_contour = moments(vect[i]);
		Point current_center = Point2f(
			current_contour.m10 / current_contour.m00,
			current_contour.m01 / current_contour.m00
		);
		double distance = euclideanDist(current_center, center_point);
		//cout << distance << endl;
		if (distance < smallest_distance) {
			smallest_distance = distance;
			closest_point = current_center;
		}
	} 
	//cout << closest_point.x << closest_point.y << endl;
	return closest_point;
}

void proc_img_4(int, void* user_data) {
	inRange(img_clone, Scalar(Hue_Lower_Value, Saturation_Lower_Value, Value_Lower), Scalar(Hue_Lower_Upper_Value, Saturation_Upper_Value, Value_Upper), adjust_image);
	imshow("adjust", adjust_image);
}

void adjust() {
	namedWindow("Adjust");

	createTrackbar("Hue_Lower", "Adjust", &Hue_Lower_Value, 255, proc_img_4);//track-bar for lower hue//
	createTrackbar("Hue_Upper", "Adjust", &Hue_Lower_Upper_Value, 255, proc_img_4);//track-bar for lower-upper hue//
	createTrackbar("Sat_Lower", "Adjust", &Saturation_Lower_Value, 255, proc_img_4);//track-bar for lower saturation//
	createTrackbar("Sat_Upper", "Adjust", &Saturation_Upper_Value, 255, proc_img_4);//track-bar for higher saturation//
	createTrackbar("Val_Lower", "Adjust", &Value_Lower, 255, proc_img_4);//track-bar for lower value//
	createTrackbar("Val_Upper", "Adjust", &Value_Upper, 255, proc_img_4);//track-bar for upper value//
}

vector<vector<Point>> erase_small_contours (vector<vector<Point>> vect, int th){
	vector<vector<Point>> answer;
	for (int i = 0; i < vect.size(); i++) {
		double new_contour = contourArea(vect[i]);
		if (new_contour > th)
			answer.push_back(vect[i]);
	}
	return answer;
}



void mark_matched_gks (vector<vector<Point>> vect, vector<Point> gk){
	double match = 0;
	for (size_t i = 0; i < vect.size(); i++){
		match = matchShapes(vect[i], gk, CV_CONTOURS_MATCH_I2, 0);
		cout << match << endl;
		if (match < 0.5) {
			Moments contour_moment = moments(vect[i]);
			Point center_point = Point2f(
				contour_moment.m10 / contour_moment.m00,
				contour_moment.m01 / contour_moment.m00
			);
			circle(gk_cntrs, center_point, 10, Scalar(255, 0, 0), FILLED);
		}
		
	}
}

void task_4(string image_name, string image_name_1) {
	img = imread(image_name, IMREAD_GRAYSCALE);
	gks = imread(image_name_1, IMREAD_GRAYSCALE);
	threshold(gks, gks, 235, 255, THRESH_BINARY);
	bitwise_not(gks, gks);
	imshow("gk", gks);
	findContours(gks.clone(), gk_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	threshold(gks, gk_cntrs, 255, 255, 0);
	drawContours(gk_cntrs, erase_small_contours(gk_contours, 150), -1, Scalar(255, 255, 255), 1);
	imshow("img_color.jpg", img);
	threshold(img, cntrs, 255, 255, 0);
	findContours(img.clone(), gk_contour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	drawContours(cntrs, find_largest_contour(gk_contour), -1, Scalar(255, 255, 255), 1);
	imshow("contour", cntrs);
	mark_matched_gks(erase_small_contours(gk_contours, 150), find_largest_contour(gk_contour));
	imshow("gk_cntrs", gk_cntrs);
	waitKey();
}

void out_contour_area(vector<vector<Point>> vect) {
	cout << "vect.size() = " << vect.size() << endl;
	for (size_t i = 0; i < vect.size(); i++){
		cout << contourArea(vect[i]) << endl;
	}
}

/*в общем - я посмотрел видео и понял, что это не робот, а красная лампа у робота, поэтому из красных контуров, я удалю два контура лампы*/
vector<vector<Point>> erase_lamp_contours(vector<vector<Point>> vect) {
	vector<vector<Point>> answer;
	for (int i = 0; i < vect.size(); i++) {
		Moments contour_moment = moments(vect[i]);
		Point center_point = Point2f(
			contour_moment.m10 / contour_moment.m00,
			contour_moment.m01 / contour_moment.m00
		);
		if (!((center_point.x <= 450) && (center_point.y <= 160) && (center_point.x >= 405) && (center_point.y >= 135)))
			answer.push_back(vect[i]);
	}
	return answer;
}


vector<vector<Point>> return_contours_near_balka(vector<vector<Point>> vect) {
	vector<vector<Point>> answer;	
	for (int i = 0; i < vect.size(); i++) {
		Moments contour_moment = moments(vect[i]);
		Point center_point = Point2f(
			contour_moment.m10 / contour_moment.m00,
			contour_moment.m01 / contour_moment.m00
		);
		if ((center_point.x >= 380) && (center_point.x <= 450))
			answer.push_back(vect[i]);
	}
	return answer;
}

Point2f return_mid_point(cv::Point2f a, cv::Point2f b) {
	return Point2f((a.x + b.x) / 2, (a.y + b.y) / 2);
}

int draw_centers_of_split_robots(vector<vector<Point>> vect, Scalar color, Mat image) { //Возвращает количество найденных пар
	if (vect.size() < 2) return 0;
	vector<int> number_of_checked_vectors;
	vector<Point2f> answer;
	Point closest_point;
	Point center_point_first;
	double smallest_distance = DBL_MAX;
	bool check = false;
	for (size_t i = 0; i < vect.size(); i++){
		check = false;
		if (number_of_checked_vectors.size() != 0) {
			for (size_t k = 0; k < number_of_checked_vectors.size(); k++) {
				if (i == number_of_checked_vectors[k]) check = true; //значит, что этот контур уже рассматривался
			}
		}
		int inj = 0;
		smallest_distance = DBL_MAX;
		if (check == false && (vect.size() - number_of_checked_vectors.size()) > 1) {
			for (size_t j = 0; j < vect.size(); j++) {
				check = false;
				for (size_t l = 0; l < number_of_checked_vectors.size(); l++) {
					if (j == number_of_checked_vectors[l]) check = true; //значит, что этот контур уже рассматривался
				}
				
				if ((i != j) && (check == false)) {
					Moments contour_moment = moments(vect[i]);
					center_point_first = Point2f(
						contour_moment.m10 / contour_moment.m00,
						contour_moment.m01 / contour_moment.m00
					);
					Moments current_contour = moments(vect[j]);
					Point current_center = Point2f(
						current_contour.m10 / current_contour.m00,
						current_contour.m01 / current_contour.m00
					);
					double distance = euclideanDist(current_center, center_point_first);
					//cout << distance << endl;
					if (distance < smallest_distance) {
						smallest_distance = distance;
						closest_point = current_center;
						inj = j;
					}				
				}
				
			}
			if (smallest_distance <= 20){
				answer.push_back(return_mid_point(center_point_first, closest_point));
				number_of_checked_vectors.push_back(inj);
				number_of_checked_vectors.push_back(i);
			}
			
		}

	}
	for (size_t i = 0; i < answer.size(); i++)
	{
		//cout << "x: " << answer[i].x << " y: " << answer[i].y << endl;
		circle(image, answer[i], 15, color, FILLED);
	}
	return answer.size();
}


 
Mat task_3_upd(Mat img) {
	cvtColor(img, img, COLOR_BGR2HSV);
	Mat red_img_1;
	inRange(img, red_low, red_high, red_img);
	inRange(img, red_low_1, red_high_1, red_img_1);
	inRange(img, blue_low, blue_high, blue_img);
	inRange(img, green_low, green_high, green_img);
	inRange(img, lamp_low, lamp_high, lamp_img);
	//imshow("lamp", lamp_img);

	cvtColor(img, img, COLOR_HSV2BGR);
	red_img = red_img + red_img_1;

	findContours(red_img.clone(), red_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	red_contours_upd = erase_lamp_contours(erase_small_contours(red_contours, 18));
	drawContours(img, red_contours_upd, -1, Scalar(0, 0, 255), FILLED);
	
	findContours(blue_img.clone(), blue_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	blue_contours_upd = erase_small_contours(blue_contours, 18);
	drawContours(img, blue_contours_upd, -1, Scalar(255, 0, 0), FILLED);

	findContours(green_img.clone(), green_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	green_contours_upd = erase_small_contours(green_contours, 18);
	drawContours(img, green_contours_upd , -1, Scalar(0, 255, 0), FILLED);

	findContours(lamp_img.clone(), lamp_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	out_contour_area((red_contours_upd));


	string blue = "Number of blue robots:" + to_string(blue_contours_upd.size() - draw_centers_of_split_robots(return_contours_near_balka(blue_contours_upd), Scalar(255, 0, 0), img));
	cv::putText(img, blue, cv::Point(10, 10), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 0), 1, false);

	string green = "Number of green robots:" + to_string(green_contours_upd.size() - draw_centers_of_split_robots(return_contours_near_balka(green_contours_upd), Scalar(0, 255, 0), img));
	cv::putText(img, green, cv::Point(10, 35), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 0), 1, false);

	string red = "Number of red robots:" + to_string(red_contours_upd.size() - draw_centers_of_split_robots(return_contours_near_balka(red_contours_upd), Scalar(0, 0, 255), img));
	cv::putText(img, red, cv::Point(10, 60), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 0), 1, false);

	circle(img, return_closest_robot(red_contours_upd), 5, Scalar(0, 0, 0), FILLED);
	circle(img, return_closest_robot(blue_contours_upd), 5, Scalar(0, 0, 0), FILLED);
	circle(img, return_closest_robot(green_contours_upd), 5, Scalar(0, 0, 0), FILLED);

	return img;
}

void task3_video (string video){
	VideoCapture vid;
	vid.open(video);
	
	while (true) {
		Mat frame;
		vid >> frame;
		if (frame.empty()) {
			break;
		}
		cvtColor(frame.clone(), img_clone, COLOR_BGR2HSV);
		adjust();
		imshow("vid", task_3_upd(frame));
		waitKey();
	}
	
}

int main(){

	//task_1("ig_1.jpg");
	//task_2("teplo3.jpg");
	task3_video("robot_video.mp4");
	//task_4("gk_tmplt.jpg", "gk.jpg");
	waitKey();
}
