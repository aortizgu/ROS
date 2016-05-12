#include "ros/ros.h"
#include <iostream>
#include <list>
#include "csuro_tools/Scan.h"
#include <cstdlib>
#include "search_3d/ObsMsg.h"

class Obstaculo
{

public:

	float xPeak_;
	float yPeak_;
	float angMax_;
	float angMin_;
	float minDis_;
	float peak_;
	int count_;
	int id_;
	Obstaculo(float dis, float ang, int id, float x, float y){
		count_ = 0;
		id_ = id;
		angMax_ = ang;
		angMin_ = ang;
		xPeak_ = x;
		yPeak_ = y;
		minDis_ = dis;
		peak_ = 0.0;
	}

	int isTheSame(float ro, float tetha, float x, float y){
		if(tetha < angMax_+0.10 && tetha > angMin_-0.10){
			count_++;
			if(tetha < angMin_)
				angMin_ = tetha;

			if(tetha > angMax_)
				angMax_ = tetha;
			if(ro < minDis_){
				minDis_ = ro;
				peak_ = tetha;
				xPeak_ = x;
				yPeak_ = y;
			}

			return 1;
		}else{
			return 0;
		}
	}

	~Obstaculo(){
		;
	}
};

int main(int argc, char **argv)
{
   ros::init(argc, argv, "laser");
   ros::NodeHandle n;
   ros::Publisher pub_obs_;
   ros::NodeHandle n_;
   ros::Rate loop_rate(20);
   Scan scan;
   std::list<Obstaculo> mylist;
   std::list<Obstaculo> mylistaux;
   std::list<Obstaculo>::iterator itObs;

   float auxTetha;
   float auxRo;
   int count=0;
   int isInList=0;
   bool emptyList = true;


   pub_obs_ = n_.advertise<search_3d::ObsMsg>("obstacles",1);
   while (ros::ok())
   {
		 search_3d::ObsMsg data;
		 std::vector<tf::Stamped<tf::Point> > last_scan = scan.getLastScan();
		 emptyList = true;

		 if(last_scan.size()>0 && (last_scan.begin()->stamp_ - ros::Time::now()).toSec() < 1.0)
		 {
			 count=0;
			 std::vector<tf::Stamped<tf::Point> >::iterator it;
			 for(it=last_scan.begin(); it!=last_scan.end(); ++it)
			 {
				 isInList = 0;
				 auxRo = sqrt(
						 it->getX()*it->getX() +
						 it->getY()*it->getY()
						 );
				 auxTetha = atan2(it->getY(), it->getX());

				 if(auxRo > 0.30 && auxRo < 1.0)
				 {
					 if(emptyList == true)
					 {
						 mylistaux.push_back(Obstaculo(auxRo, auxTetha, count, it->getX(), it->getY()));
						 emptyList = false;

					 }
					 else
					 {
						 for(itObs=mylistaux.begin();itObs!=mylistaux.end();itObs++){//Busco
							if((isInList = itObs->isTheSame(auxRo, auxTetha, it->getX(), it->getY())) == 1)
								break;
						 }
						 if(isInList != 1){//añado
							mylistaux.push_back(Obstaculo(auxRo, auxTetha, count, it->getX(), it->getY()));
						 }
					 }
				 }
			 }
		 }
		 for(itObs=mylistaux.begin();itObs!=mylistaux.end();itObs++){
			 if(itObs->count_>3){
				 mylist.push_back(*itObs);
				 count++;
			 }
		 }

		data.numObjects = count;
		data.header.stamp = ros::Time::now();
		ROS_INFO("numObjects %d", count);
		for(itObs=mylist.begin();itObs!=mylist.end();itObs++){
			ROS_INFO("obj %d  angMax->%lf  angMin->%lf dismin->%lf  count->%d xPeak->%lf yPeak->%lf "
																					, itObs->id_
																					, itObs->angMax_
																					, itObs->angMin_
																					, itObs->minDis_
																					, itObs->count_
																					, itObs->xPeak_
																					, itObs->yPeak_);

			data.tethaPeak.push_back(itObs->peak_);
			data.ro.push_back(itObs->minDis_);
			data.tethaMin.push_back(itObs->angMin_);
			data.tethaMax.push_back(itObs->angMax_);
			data.count.push_back(itObs->count_);
			data.xPeak.push_back(itObs->xPeak_);
			data.yPeak.push_back(itObs->yPeak_);
		}
		pub_obs_.publish(data);
		mylist.clear();
		mylistaux.clear();
		ros::spinOnce();
		loop_rate.sleep();
		count = 0;
   }
   return 0;
 }
