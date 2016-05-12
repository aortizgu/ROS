#include <pluginlib/class_list_macros.h>
#include <csuro_navigation/csuro_global_planner.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(csuro_global_planner::CSUROGlobalPlanner, nav_core::BaseGlobalPlanner)
int DEF = 255;
using namespace std;

//Default Constructor
namespace csuro_global_planner {
CSUROGlobalPlanner::CSUROGlobalPlanner ()
: nh_(),
  gradient_(),
  original_costmap_(),
  gradient_pub_(&nh_, &gradient_, "/map", "/csuro_navigation_gradient", true)
{

}

CSUROGlobalPlanner::CSUROGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
: nh_(),
  gradient_(),
  original_costmap_(),
  gradient_pub_(&nh_, &gradient_, "/map", "/csuro_navigation_gradient", true)
{
	initialize(name, costmap_ros);
}


void CSUROGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
	ROS_INFO("CSURO Global Planner initializing...");

	original_costmap_ = *costmap_ros->getCostmap();
	gradient_ = original_costmap_;


	gradient_pub_.publishCostmap();
	ROS_INFO("CSURO Global Planner initilizazed");

}

bool CSUROGlobalPlanner::isCoordInMap(const costmap_2d::Costmap2D &costmap, const float& x, const float& y)
{
	int cellI, cellJ;

	original_costmap_.worldToMapEnforceBounds(x, y, cellI, cellJ);

	if(cellI < costmap.getSizeInCellsX() && cellI >= 0 && cellJ < costmap.getSizeInCellsY() && cellJ >= 0)
		return true;
	return false;
}

void upTo(costmap_2d::Costmap2D &gradient_){

	for (int varX = 0; varX < gradient_.getSizeInCellsX(); varX++) {
		for (int varY = 0; varY < gradient_.getSizeInCellsY(); varY++) {
			//if(gradient_.getCost(varX, varY) == 0)
				gradient_.setCost(varX, varY, DEF);
		}
	}
}

bool inMap(costmap_2d::Costmap2D &gradient_, int i, int j)
{
	if(i < gradient_.getSizeInCellsX() && i >= 0 && j < gradient_.getSizeInCellsY() && j >= 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool go(costmap_2d::Costmap2D &gradient_, costmap_2d::Costmap2D &original_, int i, int j, int cost)
{
	if(inMap(gradient_, i, j) == false){
		//ROS_INFO("salgo por no estar en mapa");
		return false;
	}

	if(original_.getCost( i, j) != 0 ){
		//ROS_INFO("salgo por ser obstaculo");
		return false;
	}

	if(gradient_.getCost( i, j) <= cost){
		//ROS_INFO("salgo por ser igual o menor");
		return false;
	}

	return true;
}

void spread(costmap_2d::Costmap2D &gradient_, costmap_2d::Costmap2D &original_, int value, int i, int j, int startCellI, int startCellJ)
{

	gradient_.setCost(i, j, value);

	//ROS_INFO("(%d, %d) <- %d", i, j, value);

	value = value + 5;
	if(value>254)
		value=254;

	if(go(gradient_, original_, i-1,j, value))
		spread(gradient_, original_, value, i-1, j, startCellI, startCellJ);
	if(go(gradient_, original_, i,j-1, value))
		spread(gradient_, original_, value, i, j-1, startCellI, startCellJ);
	if(go(gradient_, original_, i+1,j, value))
		spread(gradient_, original_, value, i+1, j, startCellI, startCellJ);
	if(go(gradient_, original_, i,j+1, value))
		spread(gradient_, original_, value, i, j+1, startCellI, startCellJ);


}

bool isEnd(costmap_2d::Costmap2D gradient_, int i, int j, int iGoal, int jGoal)
{
	if(
			((i == iGoal) && (j == jGoal)) ||
			((i-1 == iGoal) && (j == jGoal)) ||
			((i+1 == iGoal) && (j == jGoal)) ||
			((i == iGoal) && (j-1 == jGoal)) ||
			((i-1 == iGoal) && (j-1 == jGoal)) ||
			((i+1 == iGoal) && (j-1 == jGoal)) ||
			((i == iGoal) && (j+1 == jGoal)) ||
			((i-1 == iGoal) && (j+1 == jGoal)) ||
			((i+1 == iGoal) && (j+1 == jGoal))
		)return true;
	return false;
}

void getMin(costmap_2d::Costmap2D gradient_, int &i, int &j)
{
	int iOut = i;
	int jOut = j;

	if(gradient_.getCost( i-1, j-1) < gradient_.getCost( iOut, jOut))
	{
		iOut = i - 1;
		jOut = j - 1;
	}
	if(gradient_.getCost( i-1, j) < gradient_.getCost( iOut, jOut))
	{
		iOut = i - 1;
		jOut = j;
	}
	if(gradient_.getCost( i-1, j+1) < gradient_.getCost( iOut, jOut))
	{
		iOut = i - 1;
		jOut = j + 1;
	}
	if(gradient_.getCost( i, j-1) < gradient_.getCost( iOut, jOut))
	{
		iOut = i;
		jOut = j - 1;
	}
	if(gradient_.getCost( i, j+1) < gradient_.getCost( iOut, jOut))
	{
		iOut = i;
		jOut = j + 1;
	}
	if(gradient_.getCost( i+1, j-1) < gradient_.getCost( iOut, jOut))
	{
		iOut = i + 1;
		jOut = j - 1;
	}
	if(gradient_.getCost( i+1, j) < gradient_.getCost( iOut, jOut))
	{
		iOut = i + 1;
		jOut = j;
	}
	if(gradient_.getCost( i+1, j+1) < gradient_.getCost( iOut, jOut))
	{
		iOut = i + 1;
		jOut = j + 1;
	}
	i = iOut;
	j = jOut;
}

void fillPlan(costmap_2d::Costmap2D gradient_, std::vector<geometry_msgs::PoseStamped>& plan, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, int startI, int startJ, int goalI, int goalJ)
{


	int iPoint = startI;
	int jPoint = startJ;

	plan.push_back(start);

	double x,y;
	int counter = 0;
	geometry_msgs::PoseStamped before = start;
	geometry_msgs::PoseStamped point;
	tf::Quaternion point_quat;

	while(isEnd(gradient_, iPoint, jPoint, goalI, goalJ) == false)
	{
		counter++;
		getMin(gradient_, iPoint, jPoint);
		gradient_.mapToWorld (iPoint, jPoint, x, y);
		point.pose.position.x = x;
		point.pose.position.y = y;
		point_quat = tf::createQuaternionFromYaw(atan2 (y, x));
		point.pose.orientation.x = point_quat.x();
		point.pose.orientation.y = point_quat.y();
		point.pose.orientation.z = point_quat.z();
		point.pose.orientation.w = point_quat.w();
		if(counter%2 == 0)
			plan.push_back(point);
		before = point;
	}

	plan.push_back(goal);



}

bool CSUROGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan )
{

	ROS_INFO("CSURO Global Planner making plan");

	float startX = start.pose.position.x;
	float startY = start.pose.position.y;
	float goalX = goal.pose.position.x;
	float goalY = goal.pose.position.y;

	ROS_INFO("New Goal coordinate (%lf, %lf) in %s", goalX, goalY, goal.header.frame_id.c_str());
	ROS_INFO("Coords vals in ([%lf, %lf], [%lf, %lf])",
			original_costmap_.getOriginX(), original_costmap_.getOriginY(),
			original_costmap_.getOriginX() + original_costmap_.getSizeInMetersX(),
			original_costmap_.getOriginY() + original_costmap_.getSizeInMetersY());

	if(!isCoordInMap(original_costmap_, goalX, goalY))
	{
		ROS_ERROR("Coordinate (%lf, %lf) is outside of the map", goalX, goalY);
		return false;
	}

	int startCellI, startCellJ;
	int goalCellI, goalCellJ;


	original_costmap_.worldToMapEnforceBounds(goalX, goalY, goalCellI, goalCellJ);
	original_costmap_.worldToMapEnforceBounds(startX, startY, startCellI, startCellJ);

	//Compute gradient
	ROS_INFO("SET (%d, %d) [%d]", goalCellI, goalCellJ, static_cast<int>(gradient_.getCost(goalCellI, goalCellJ)));
	//ROS_INFO("SIZEMAP = [%d x %d]", gradient_.getSizeInCellsX(), gradient_.getSizeInCellsY());
	gradient_ = original_costmap_;
	upTo(gradient_);
	spread(gradient_, original_costmap_, 0, goalCellI, goalCellJ, startCellI, startCellJ);
	gradient_pub_.publishCostmap();
	fillPlan(gradient_, plan, start, goal, startCellI, startCellJ, goalCellI, goalCellJ);

	ROS_INFO("Plan ========================");
	std::vector<geometry_msgs::PoseStamped>::iterator it;
	for(it=plan.begin(); it<plan.end(); ++it)
	{
		ROS_INFO("%lf %lf", it->pose.position.x, it->pose.position.y);
	}
	ROS_INFO("=============================");

	ROS_INFO("CSURO Global Plann done");

	return true;
}

};
