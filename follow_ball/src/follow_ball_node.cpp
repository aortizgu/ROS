#include "ros/ros.h"
#include "csuro_tools/PIDController.h"
#include "kobuki_msgs/BumperEvent.h"
#include "geometry_msgs/Twist.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>


class FollowBall{
private:

	static const int LOST		= 0;
	static const int FOLLOWING	= 1;

	float turn_;
	float forward_;

	float x_;
	float y_;

	bool state_;
	bool turnLeft_;
	bool turnOff_;
	bool bestLeft_;
	bool lost_;

	PIDController h_;
	PIDController v_;
	ros::NodeHandle n_;
	cv_bridge::CvImagePtr cv_ptr_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber sub_bumber_;
	ros::Publisher pub_vel_;

public:

	FollowBall():it_(n_),
				h_("h", 0.1,0.1,0.05,0.950),
				v_("v", 0.1,0.1,0.05,0.950),
				state_(LOST),
				lost_(true),
				turnLeft_(true),
				turnOff_(false),
				bestLeft_(false),
				x_(0),
				y_(0),
				forward_(0.0),
				turn_(0.0)

	{
		image_sub_ = it_.subscribe("/hsv/image_filtered", 1, &FollowBall::imageCallback, this);
		pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
	}


	void step()
	{
		geometry_msgs::Twist cmd;

		ROS_INFO("############################################");

		switch(state_)
		{
		case LOST:

			cmd.linear.x = 0.01;
			if(x_ > 0){
				cmd.angular.z = 0.60;
				ROS_INFO("########  I'M LOST TURN -> LEFT   ##########");
			}else{
				cmd.angular.z = -0.60;
				ROS_INFO("#######   I'M LOST TURN -> RIGTH  ##########");
			}
			if (lost_ == false)
			{
				state_ = FOLLOWING;
				ROS_INFO("LOST->FOLLOWING");
			}
			break;

		case FOLLOWING:

			ROS_INFO("#############FOLLOWING######################");
			ROS_INFO("## x -> %f  ####### y -> %f  ##", x_, y_ );
			cmd.linear.x = 0.3*y_;
			cmd.angular.z = 1.2*x_;
			if (lost_ == true)
			{
				state_ = LOST;
				ROS_INFO("FOLLOWING->LOST");
			}
			break;
		}
		pub_vel_.publish(cmd);
		ROS_INFO("############################################");
	}


	void imageCallback(const sensor_msgs::Image::ConstPtr& img)
	{
		int xwork = 0;
		int ywork = 0;

		try
		{
			cv_ptr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		cv::Mat *work_image_ = &(cv_ptr_->image);
				//ROS_INFO("alto: %i, ancho: %i", work_image_.rows, work_image_.cols);//640*480(filas*columnas)

		searchXY(*work_image_, xwork, ywork);

		if(xwork < work_image_->cols && xwork > 0 && ywork < work_image_->rows && ywork > 0){
			lost_ = false;
			ROS_INFO("xwork %i    ywork %i", xwork, ywork);
			x_ = -(float)
					(
							(
									((float)xwork) - ( ((float)work_image_->cols) / 2.00 )
							)
							/
							(
									((float)work_image_->cols) / 2.00
							)
					);
			y_ = -(float)
					(
							(
									((float)ywork) - ( 390.00 )
							)
							/
							(
									390.00
							)
					);

			ROS_INFO("antes x_ %f    y_ %f", x_, y_);

			h_.setReference(x_);
			v_.setReference(y_);
			x_ = h_.getOutput();
			y_ = v_.getOutput();
			//ROS_INFO("output h_ %f    v_ %f", h_.getOutput(), v_.getOutput());
			//ROS_INFO("despues x_ %f    y_ %f", x_, y_);
		}else{
			lost_ = true;
		}
	}

	void process(int cols, int rows){

	}

	void searchXY(const cv::Mat& work_image_, int& x, int& y){

		int posdata = 0;
		int channels = 3;
		int xmin = 9999999;
		int xmax = -1;
		int ymin = 9999999;
		int ymax = -1;
		int counter = 0;

		for (int j = 0; j < work_image_.rows; ++j)
		{
			for (int i = 0; i < work_image_.cols; ++i)
			{
				int posdata = j*work_image_.step+i*channels;
				if(work_image_.data[posdata] != 0){
					counter++;
					if(i < xmin && counter > 5)
						xmin = i;
					if(i > xmax && counter > 5)
						xmax = i;
					if(j < ymin && counter > 5)
						ymin = j;
					if(j > ymax && counter > 5)
						ymax = j;
				}else{
					counter = 0;
				}
			}
		}

		if(xmin > 0 && xmax < work_image_.cols){
			x =(xmin+xmax)/2;
		}else{
			x = -1;
		}
		if(ymin > 0 && ymax < work_image_.rows)
		{
			y = (ymin+ymax)/2;
		}else{
			y = -1;
		}
	}

};

int main(int argc, char **argv) {

	ros::init(argc, argv, "follow_ball");

	FollowBall followballer;

	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		followballer.step();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
