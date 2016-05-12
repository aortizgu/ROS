#include "ros/ros.h"
#include "csuro_tools/PIDController.h"
#include "kobuki_msgs/BumperEvent.h"
#include "geometry_msgs/Twist.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>
#include <math.h>
#include "search_3d/ObsMsg.h"

enum{
	LOST_LEFT_,
	LOST_RIGTH_,
	FOLLOWING
};

float REP = 0.2;
float ATRACT = 0.8;
float DEPHT_TO_LOOK = 1.00;

class Follower_3d
{
private:

	ros::NodeHandle nVel_;
	ros::Subscriber sub_obs_;
	PIDController ro_;
	PIDController teta_;
	ros::Publisher pub_vel_;
	ros::NodeHandle nObs_;

public:

	tf::Transform bf2obj;
	tf::StampedTransform bf2odom;
	tf::StampedTransform odom2obj;
	tf::TransformListener listener_;
	tf::Vector3 objectData_;
	tf::Vector3 repulData_;
	tf::Vector3 balanceData_;
	float x_;
	float y_;
	int state_;
	bool lost_;
	bool start_;
	bool repForce_;
	Follower_3d():
					ro_("h", 0.08,1.0,0.01,0.15),
					teta_("v", 0.05,0.8,0.05,0.8),
					x_(0),
					y_(0),
					state_(LOST_LEFT_),
					lost_(true),
					start_(false),
					repForce_(false)
	{
		pub_vel_ = nVel_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
		sub_obs_ = nObs_.subscribe("obstacles", 1, &Follower_3d::getForces, this);
		ro_.setPIDKs(0.5, 0.2, 0.3);
		teta_.setPIDKs(0.5, 0.2, 0.3);
	}

	void takeControl()
	{
		geometry_msgs::Twist cmd;

		switch(state_)
		{
		case LOST_LEFT_:

			cmd.linear.x = 0.01;
			cmd.angular.z = 0.20;
			ROS_INFO("########  I'M LOST TURN -> LEFT   ##########");
			if (lost_ == false)
			{
				state_ = FOLLOWING;
				ROS_INFO("LOST->FOLLOWING");
			}
			break;

		case LOST_RIGTH_:

			cmd.linear.x = 0.01;
			cmd.angular.z = -0.20;
			ROS_INFO("########  I'M LOST TURN -> RIGTH   ##########");
			if (lost_ == false)
			{
				state_ = FOLLOWING;
				ROS_INFO("LOST->FOLLOWING");
			}
			break;

		case FOLLOWING:

			ROS_INFO("#############FOLLOWING###########################");
			ROS_INFO("## ro -> %lf  ####### teta -> %lf  ##", ro_.getOutput(), teta_.getOutput());
			cmd.linear.x = ro_.getOutput();
			cmd.angular.z = teta_.getOutput();

			if (lost_ == true)
			{
				if(teta_.getOutput() > 0)
				{
					state_ = LOST_LEFT_;
				}
				else
				{
					state_ = LOST_RIGTH_;
				}
				ROS_INFO("FOLLOWING->LOST");
			}
			break;

		}
		pub_vel_.publish(cmd);
	}

	void localize()
	{
		try
		{
			if(listener_.canTransform("/base_footprint", "/odom", ros::Time(0)))
				listener_.lookupTransform("/base_footprint", "/odom", ros::Time(0), bf2odom);
		}
		catch (tf::TransformException& ex)
		{
			ROS_ERROR("%s",ex.what());
		}
		try
		{
			if(listener_.canTransform("/odom", "/object/0", ros::Time(0))){
				lost_=false;
				listener_.lookupTransform("/odom", "/object/0", ros::Time(0), odom2obj);
			}
		}
		catch (tf::TransformException& ex)
		{
			ROS_ERROR("%s",ex.what());
		}
		/*if((ros::Time::now()-odom2obj.stamp_).toSec() > 10)
		{
			lost_ = true;
			//ROS_INFO("----TRUE");
		}*/
		bf2obj =  bf2odom * odom2obj;
		ROS_INFO("bf2obj  [%lf, %lf, %lf] %lf ago",
				bf2obj.getOrigin().getX(),
				bf2obj.getOrigin().getY(),
				bf2obj.getOrigin().getZ(),
				(ros::Time::now()-odom2obj.stamp_).toSec());
		ROS_INFO("bf2odom  [%lf, %lf, %lf] %lf ago",
				bf2odom.getOrigin().getX(),
				bf2odom.getOrigin().getY(),
				bf2odom.getOrigin().getZ(),
				(ros::Time::now()-bf2odom.stamp_).toSec());
		ROS_INFO("odom2obj  [%lf, %lf, %lf] %lf ago",
				odom2obj.getOrigin().getX(),
				odom2obj.getOrigin().getY(),
				odom2obj.getOrigin().getZ(),
				(ros::Time::now()-odom2obj.stamp_).toSec());
	}

	float getRo(tf::Vector3 input)
	{
		return sqrt(
				input.getX()*input.getX()+
				input.getY()*input.getY()
				);
	}

	float getTetha(tf::Vector3 input)
	{
		return atan2(input.getY(), input.getX());
	}

	void getForces(const search_3d::ObsMsg::ConstPtr& obstacles)
	{
		float roAux;
		float tethaAux;
		int numObj;
		tf::Vector3 auxVector;
		//ROS_INFO("numObjects %li", obstacles->numObjects);
		if(obstacles->numObjects == 0)
			repForce_ = false;
		else{
			repForce_ = true;
			for (int var = 0; var < obstacles->numObjects; ++var) {
				//ROS_INFO("peak %lf xpeak %lf yPeak %lf", obstacles->tethaPeak.data()[var], obstacles->xPeak.data()[var], obstacles->yPeak.data()[var]);
				auxVector.setX(auxVector.getX() + obstacles->xPeak.data()[var]);
				auxVector.setY(auxVector.getY() + obstacles->yPeak.data()[var]);
			}
			//cambio signo y hago media
			auxVector.setX(-(auxVector.getX()/obstacles->numObjects));
			auxVector.setY(-(auxVector.getY()/obstacles->numObjects));
			//saco angulo y modulo
			roAux = getRo(auxVector);
			tethaAux = getTetha(auxVector);
			//cambio el modulo para no generar fuerzas raras
			roAux = DEPHT_TO_LOOK-roAux;
			//saco el vector de fuerzas repulsivas
			auxVector.setX(roAux*cos(tethaAux));
			auxVector.setY(roAux*sin(tethaAux));
			//lo asigno a la v.global
			repulData_ = auxVector;
		}
	}



	void normalizeAtract()
	{

		float tethaAux = getTetha(objectData_);
		float roAux = (getRo(objectData_)-0.6)/0.6;
	/*	if(roAux > 1.00)
		{
			roAux = 1.00;
		}*/
		objectData_.setX(roAux*cos(tethaAux));
		objectData_.setY(roAux*sin(tethaAux));
	}

	void normalizeRep(tf::Vector3 input, float* ro, float* tetha)
	{
		*ro = 1.5/getRo(input);
		*tetha = getTetha(input);
	}

	void setBalance()
	{
		float katract= 1.2*(1-getRo(repulData_));
		float krep = 1-katract;
		ROS_INFO("|krep:%lf|katrac:%lf|", krep, katract);
		balanceData_.setX(katract * objectData_.getX() + krep * repulData_.getX());
		balanceData_.setY(katract * objectData_.getY() + krep * repulData_.getY());
	}
	void step()
	{
		//busco bola
		localize();
		//cojo vector
		objectData_ = bf2obj.getOrigin();
		if(lost_ == false)
		{
			ROS_INFO("-------------------------------------------------");
			normalizeAtract();
			ROS_INFO("|Atract|x:%lf|y:%lf|ro:%lf|tetha:%lf|", objectData_.getX(), objectData_.getY(), getRo(objectData_), getTetha(objectData_));
			if(repForce_ == true)
			{
				setBalance();
				ROS_INFO("|Rep|x:%lf|y:%lf|ro:%lf|tetha:%lf|", repulData_.getX(), repulData_.getY(), getRo(repulData_), getTetha(repulData_));
				ROS_INFO("|Balance|x:%lf|y:%lf|ro:%lf|tetha:%lf|", balanceData_.getX(), balanceData_.getY(), getRo(balanceData_), getTetha(balanceData_));
				ro_.setReference(getRo(balanceData_));
				teta_.setReference(getTetha(balanceData_));
			}
			else
			{
				ro_.setReference(getRo(objectData_));
				teta_.setReference(getTetha(objectData_));
			}
			ROS_INFO("-------------------------------------------------");
		}
		takeControl();
	}
};


int main(int argc, char **argv) {

	ros::init(argc, argv, "follow_ball_3d");

	Follower_3d ball_3d;

	ros::Rate loop_rate(20);

	while(ros::ok())
	{
		ball_3d.step();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
