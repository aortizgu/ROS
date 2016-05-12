#include "ros/ros.h"
#include "kobuki_msgs/BumperEvent.h"
#include "geometry_msgs/Twist.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>

class Follower{

public:




	Follower(): success_(0), left_(0), rigth_(0), iteration(0), maxMedia(100), fail_(0), media(), state_(LOST), it_(n_), lost_(true), turnOff_(true), turnLeft_(false), bestLeft_(false)//constructor!!!!!!!!!!!!!!!!!!!!
	{
		image_sub_ = it_.subscribe("/hsv/image_filtered", 1, &Follower::imageCallback, this);
		pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
	}

	void step()
	{
		geometry_msgs::Twist cmd;
		ROS_INFO("======================================");
		if(lost_ ==true){
			ROS_INFO("LOST");
		}else{
			ROS_INFO("FOLLOWING");
		}

		switch(state_)
		{
		case LOST:
			cmd.linear.x = 0;
			if(bestLeft_ ==true){
				cmd.angular.z = 0.40;
				ROS_INFO("I'M LOST TURN -> LEFT");
			}else{
				cmd.angular.z = -0.40;
				ROS_INFO("I'M LOST TURN -> RIGTH");
			}
			if (lost_ == false)
			{
				state_ = FOLLOWING;
				ROS_INFO("LOST->FOLLOWING");
			}
			break;

		case FOLLOWING:

			if (turnOff_ == true) {
				ROS_INFO("I'M FOLLOWING -> GO AHEAD");
				cmd.linear.x = 0.05;
				cmd.angular.z = 0.00;
			}else{
				if(turnLeft_ == true)
				{
					ROS_INFO("I'M FOLLOWING -> TURN LEFT");
					cmd.linear.x = 0.03;
					cmd.angular.z = 0.35;
				}else
				{
					ROS_INFO("I'M FOLLOWING -> TURN RIGTH");
					cmd.linear.x = 0.03;
					cmd.angular.z = -0.35;
				}
			}


			if (lost_ == true)
			{
				state_ = LOST;
				ROS_INFO("FOLLOWING->LOST");
			}
			break;
		}
		pub_vel_.publish(cmd);
		ROS_INFO("======================================");
	}

	void imageCallback(const sensor_msgs::Image::ConstPtr& img)
	{
		int rows_;
		int cols_;
		int steps_;

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

		int edges[40];
		int centers[20];
		int used = 0;
		int var = 0;
		int x = 0;

		buildEdges(edges, *work_image_, used, IMPORTANT_ROW);

		/*if (used == 0)
			buildEdges(edges, *work_image_, used, IMPORTANT_ROW-100);
		*/
		if (used == 0)
			buildEdges(edges, *work_image_, used, IMPORTANT_ROW-110);

		for (var = 0; var < used; ++var) {
			ROS_INFO("EDGE[%d] = %d", var, edges[var]);
		}

		if (used%2 == 0 && used > 0)//information finded
		{
			for (var = 0; var < used; var++) {
				if(var%2 != 0)
				{
					if (((edges[var]+edges[var-1])/2) < 600 &&
						((edges[var]+edges[var-1])/2) > 40 	)
					{
						centers[x] = (edges[var]+edges[var-1])/2;
						x++;
					}
				}
			}
			switch (x) {
				case 1:
					success_++;
					processTurn(centers[0]);
					fail_=0;
					break;
				case 2:
					success_++;
					fail_=0;

					if (abs(centers[0]-320) < abs(centers[1]-320))
					{
						processTurn(centers[0]);
					}
					else
					{
						processTurn(centers[1]);
					}
					break;
				default:
					success_ = 0;
					fail_++;
					break;
			}
			for (var = 0; var < used/2; var++) {
				ROS_INFO("center %i = %i", var, centers[var]);
			}
		}
		else//information not finded
		{
			fail_++;
		}


		if(lost_ == false){
			check();
			refresh(centers, x);
		}


		if(lost_ == true)//estoy perdido
		{
			fail_ = 0;
			ROS_INFO("SUCCESS -> %d", success_);
			if(success_ > 10)
			{
				lost_ = false;
			}
		}
		else//estoy siguiendo
		{
			success_ = 0;
			ROS_INFO("FAIL -> %d", fail_);
			if(fail_ > 10)
			{
				//makeMedia();
				decido();
				lost_ = true;
			}
		}
		iteration++;
	}

	void processTurn(int point){
		if (point > 340) {
			turnOff_=false;
			turnLeft_=false;
		}else if(point < 300)
		{
			turnOff_=false;
			turnLeft_=true;
		}else{
			turnOff_=true;
		}
	}

	void refresh(int centers[], int x){

		if(iteration >= maxMedia){
			reset();
			iteration = 0;
		}
		for (int var = 0; var < x; ++var) {
			if(centers[var]<300 || centers[var]>340){
				if(centers[var] < 320){
				left_++;
				left_++;
				rigth_--;
			}else{
				left_--;
				rigth_++;
				rigth_++;
			}
			}

		}

	}


	void buildEdges(int *edges, const cv::Mat& work_image_, int& used, int rowToLook){

		int i;
		int j;
		int posdata;
		int channels = 3;
		int status_ = 0;
		int accountant;


		for (int i = 0; i < work_image_.cols; ++i)
		{
			int posdata = rowToLook*work_image_.step+i*channels;

			switch (status_)
			{
				case GRASS:
					if (work_image_.data[posdata] == 0)
					{
						status_ = MAYBELINE;
					}
					break;
				case LINE:
					if(i==work_image_.cols-1)
					{
						edges[used] = i;
						used++;
					}
					if (work_image_.data[posdata] != 0)
					{
						status_ = MAYBEGRASS;
					}
					break;
				case MAYBELINE:
					if(work_image_.data[posdata] == 0 && accountant > 20 && used < 39)
					{
						status_ = LINE;
						edges[used] = i-20;
						used++;
						accountant = 0;
					}else if (work_image_.data[posdata] == 0 && accountant <= 20) {
						accountant++;
					}else if (work_image_.data[posdata] != 0)
					{
						status_ = GRASS;
						accountant = 0;
					}
					break;
				case MAYBEGRASS:
					if(work_image_.data[posdata] != 0 && accountant > 20 && used < 39)
					{
						status_ = GRASS;
						edges[used] = i-20;
						used++;
						accountant = 0;
					}else if (work_image_.data[posdata] != 0 && accountant <= 20) {
						accountant++;
					}else if (work_image_.data[posdata] == 0)
					{
						status_ = LINE;
						accountant = 0;
					}
					break;
			}
		}
	}



	void makeMedia(){
		int sum=0;
		for (int var = 0; var < maxMedia; ++var) {
			sum = sum + media[var];
		}
		if (sum > (maxMedia/2)) {
			bestLeft_ = true;
			ROS_INFO("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<DECISION:LEFT>>>>>>>>>>>>>>>>>>");
		} else {
			bestLeft_ = false;
			ROS_INFO("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<DECISION:RIGTH>>>>>>>>>>>>>>>>>>");
		}
	}

	void reset(){
		if(left_>rigth_){
			left_ =10;
			rigth_ = 0;
		}else{
			left_ = 0;
			rigth_ = 10;
		}
	}

	void check(){
		ROS_INFO("left->%d   rigth->%d", left_, rigth_);
			if(left_>rigth_){
				ROS_INFO("bestLeft");
			}else{
				ROS_INFO("bestRigth");
			}
		}
	void decido(){
		if(left_>rigth_){
			bestLeft_=true;
		}else{
			bestLeft_=false;
		}
	}

private:

	ros::NodeHandle n_;

	static const int LOST		= 0;
	static const int FOLLOWING	= 1;
	static const int IMPORTANT_ROW= 380;
	static const int GRASS = 0;
	static const int LINE = 1;
	static const int MAYBELINE = 2;
	static const int MAYBEGRASS = 3;


	int state_;
	int fail_;
	int success_;
	int iteration;
	int maxMedia;
	int media[20];
	int left_;
	int rigth_;

	bool turnLeft_;
	bool turnOff_;
	bool bestLeft_;
	bool lost_;

	cv_bridge::CvImagePtr cv_ptr_;

	ros::Time break_ts_;


	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;

	ros::Subscriber sub_bumber_;
	ros::Publisher pub_vel_;

};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "follow_lines");

	Follower follower;

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		follower.step();

		ros::spinOnce();
		loop_rate.sleep();

	}

	return 0;
}
