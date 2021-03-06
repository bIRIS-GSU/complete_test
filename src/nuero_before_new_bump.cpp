//Ros Header
#include "ros/ros.h"

//Msg headers
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <tf/tf.h>

#include <geometry_msgs/Twist.h> //Movement Msgs

#include <nav_msgs/Odometry.h> //Odom Msgs


#include <sensor_msgs/Image.h> //Depth Image Msgs
#include <sensor_msgs/CameraInfo.h> //CameraInfo Msgs
#include <sensor_msgs/LaserScan.h> //LaserScan Msgs

#include <kobuki_msgs/DockInfraRed.h> //Kobuki IR Sensor Status Msgs
#include <kobuki_msgs/SensorState.h> //Kobuki interal Sensor Msgs
#include "complete_test/irobotdock.h"


#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <sstream>
using namespace std;

// Global Variables

// -------------
// - control/flow variables
const double PI = 3.1415926; //PI constant for rotation
int enable_sub = 0; //rotation enable
int controlint = 0; //rotation control
float originyaw; //radian control
int stage = 0; //rotation stage control
float fromleft = 0; //needed in DestinRad Subfunction
float fromright = 0; //needed in DestinRad Subfunction

float camera; //global camera variable
int charger = 0; //charger status
int bumper = 0; //bumper status
std::vector<int>IR_Sensor (3,0); //IR Sensor vector
vector <double> locate(5,0); //Camera Vectors
int locate_enable = 0; //Vectoring Enable

int chc = 0; //neuron choice


// --------------
// - recording data
vector <vector <double> > loginx; //total data
vector<double> logvec(28,0); //individual run vectors

// ----------------
// - state neurons
int STATE_WALL_FOLLOW = 1;
int STATE_OPEN_FIELD = 2;
int STATE_EXPLORE_OBJECT = 3;
int STATE_FIND_HOME = 4;
int STATE_AT_HOME = 5;
int STATE_LEAVE_HOME = 6;

// ----------------
// - events
int e_ping_value = 1;
int e_battery = 2;
int e_bump = 3;
int e_beam = 4;
int e = 4;
//event = zeros (1,e);
std::vector<double>event (e,0);

// ----------------
// - parameters for return home
int FIND_HOME_TIMEOUT = 1;
int dock_time = 0;
int found_home = 0;

// ----------------
// - parameters for wall following
int WALL_LEFT = -1;
int WALL_RIGHT = 1;
int wall = 0;
int wallfollow_time = 0;

// ----------------
// - neurons
int N = 4;
//global n;
//n = zeros (1,N);
std::vector<double> n (N,0);

// ----------------
// - neuromodulators
int NM_DA =  1;
int NM_5HT = 2;
int NM = 2;
//global nm;
//nm = zeros (1,NM);
std::vector<double> nm (NM,0);

// ----------------
// - ACH/NE neuromodulation
//global achne;
//achne = zeros (1,e);
//%achne =ones(1,e);
std::vector<double> achne (e,0);

// ----------------
// - parameters for ping sensor event
//int comparing_distance = 20; //% 20 centimeter
float comparing_distance = 0.52;
//%comparing_max_distance = 70;

// ----------------
// - parameters for battery event
float battery;
float battery_level;
float battery_initial_level;
int battery_lock = 0;

// ----------------
// - parameters for bump event
float too_close = 0.54; //% meausred in m

// ----------------
// - parameters for beam event
int force_field = 242;
int buoy_and_force_field = 250;

// ----------------
// - set weights to their initial values
vector <vector <double> > w_n_n_exc;
vector <vector <double> > w_n_n_inh;
vector <vector <double> > w_nm_n;
vector <vector <double> > w_e_n;
vector <vector <double> > w_e_nm;
vector <double> w_e_achne;

ros::Publisher chatter_pub; //movement publisher
ros::Publisher chatter_pub2; //odom reset publisher

void roomba_net_init() 
{
	// state neuron intrinsic connectivity
	for (int r = 1; r <= N; r++)
	{
		vector <double> row(N, 0.5);
		w_n_n_exc.push_back(row);
	}
	
	for (int r = 1; r <= N; r++)
	{
		vector <double> row(N, -1.0);
		w_n_n_inh.push_back(row);
	}
		
	for (int r = 1; r <N; r++)
	{
		ROS_INFO("Pass 3.1");
		w_n_n_exc[r-1][r-1] = 0.0;
    	w_n_n_inh[r-1][r-1] = 0.0;
    	ROS_INFO("Pass 3.2");
    }
   
    

	// neuromodulator to state neuron connectivity
	//w_nm_n = zeros(NM,N);
	for (int r = 1; r <= NM; r++)
	{
		vector <double> row(N, 0);
		w_nm_n.push_back(row);
	}
	w_nm_n[NM_5HT-1][STATE_FIND_HOME-1] = 5;//% 5*rand;5;
	w_nm_n[NM_5HT-1][STATE_WALL_FOLLOW-1] = 5 ;//%5*rand;%5;
	w_nm_n[NM_DA-1][STATE_EXPLORE_OBJECT-1] =5;//%5*rand;%5;
	w_nm_n[NM_DA-1][STATE_OPEN_FIELD-1] =5;   //% 5*rand;%5;

	// event neuron to state neuron connectivity
	//w_e_n = ones(e,N);
	//% w_e_n(e_bump,STATE_FIND_HOME) = 0;
	//%w_e_n = rand(e,N);
	for (int r = 1; r <= e; r++)
	{
		vector <double> row(N, 1);
		w_e_n.push_back(row);
	}
	
	// event neuron to neuromodulator  connectivity
	//w_e_nm = zeros(e,NM);
	for (int r = 1; r <= e; r++)
	{
		vector <double> row(NM, 0);
		w_e_nm.push_back(row);
	}
	w_e_nm[e_ping_value-1][NM_DA-1] =1; //% 0+(1-0).*rand;
	w_e_nm[e_battery-1][NM_5HT-1] =1;  //%0+(1-0).*rand;%1;
	w_e_nm[e_beam-1][NM_5HT-1] =1;  //%0+(1-0).*rand;% 1;
	//w_e_nm[e_bump-1][NM_5HT-1] = 1;  //%0+(1-0).*rand; %1;    % risk averse behavior

	w_e_nm[e_bump-1][NM_DA-1] = 1; //0+(1-0).*rand;   % risk taking behavior
	
	//event neuron to neuromodulator  connectivity
	//w_e_achne = ones(1,e); //% 0+(1-0).*rand(1,e);
 	for (int r = 1; r <= e; r++)
	{
		w_e_achne.push_back(1);
	}

}




double activity(double I, double g)
{
/*% activity - sigmoid activation function
%
% Description
%   Sigmoid activation function for rate neuron
%
% Inputs
%   I - synaptic input
%   g - gain or slope of sigmoid curve
%
% Outputs
%   s - activity of neuron between 0 and 1
*/
	double sigmoid;
	sigmoid = 1 / (1 + exp(-g*I));
	return sigmoid;
}

double stp(double xin, double p, double tau, double maxi, bool spk)
{
/*% stp - Short-Term Plasticity
%
% Description
%   Simple version of short-term plasticity rule
%
% Inputs
%   xin - current weight value
%   p - amount to increase or decrease weight if there is a spike
%   tau - recovery time constant.
%   max - maximum weight value
%   spk - 1 if spike occurred.
%
% Outputs
%   x - new weight value
%
*/
double x;

	if (spk == true)
		x = p*xin;
	else
		x = xin + (1-xin)/tau;
		
	x = min(maxi,x);
	return x;
}



double rand_num()
{
	int v1 = (rand() % 10001);
    double number = (rand() % 10001) / 10000.0;
	return number;
}

double sum_vector(vector <double> vector)
{
	double sum;
	for(int i = 0; i < vector.size();i++)
	{
	  sum = sum + vector[i];	
	}
 return sum;
}

double min_vector(vector <double> vector)
{
	double min_value;
	min_value = vector[0];
	
	for (int i = 0;i < (vector.size()-1);i++)
	{
		if (min_value > vector[i+1]){
			min_value = vector[i+1];
		} 

	}
	
	return min_value;
}

void max_vector_element(vector <double> vector, double& c, double& I)
{
	c = vector[0]; //in case if all elements equal
	I = 0;	
	for (int i = 0;i < (vector.size()-1);i++) 
	{
		if (c < vector[i+1]){
			c = vector[i+1];
			I = i+1;}
	}
}

double DestinRad(double ori, double dest) //computes destination radian
{
	double destra;
	if ((ori - dest) < -PI) //readjust distance from lower limit
	{	
		destra = (ori - dest) + (2*PI);
		fromright = 1; //apply 2pi correction to right direction below limit
		fromleft = 0;
	}
	else if ((ori - dest) > PI) //readjust distance from upper limit
	{
		destra = (ori - dest) - (2*PI);
		fromright = 0;
		fromleft = 1; //apply 2pi correction to left direction above limit
	
	}
	else //within limit
	{
		destra = (ori - dest);
		fromright = 0;
		fromleft = 0;
	}
	return destra;
}

void reveal_vector(vector <double> vector)
{
	ROS_INFO("--DIRECTION VECTOR--");
	ROS_INFO("--DIRECTION VECTOR SIZE-- [%lu]", vector.size());
	for(int i = 0; i < vector.size();i++)
	{
		ROS_INFO("Element [%i]: [%f]", i, vector[i]);
	}
}


void roomba_net_cycle(std::vector<double> event, int& choice, std::vector<double>& achne_out, std::vector<double>& n_out, std::vector<double>& nm_out)
{
	double ACTION_SELECTION_THRESHOLD = 0.67;
	
	//parameters for state neuron activation function
	double N_ACT_GAIN = 2;
	double N_ACT_PERSIST = 0.25;
	double N_ACT_BASECURRENT = -1.0;
	std::vector<double> nprev = n;
	
	//parameters for neuromodulatory neuron activation function and synaptic
	//plasticity
	//global NM;
	//global nm;
	std::vector<double> nmprev = nm;
	double NM_ACT_GAIN =2; //%2+(5-2)*rand; %2;    % gain for sigmoid function
	double NM_ACT_BASECURRENT = -1.0;
	double NM_ACT_PERSIST = 0.25;  //% persistence of synaptic current
	double NM_STP_GAIN = 1.1;  //% facillitating synapse
	double NM_STP_DECAY = 50;  //% recovery time constant
	double NM_STP_MAX = 2;     //% weight value ceiling
	
	//% parameters for ACh/NE neuron activation function and synaptic plasticity
	//global achne;
	std::vector<double> achneprev = achne;
	double ACHNE_ACT_GAIN =  5; //%2+(5-2)*rand ; gain for sigmoid function
	double ACHNE_ACT_BASECURRENT = -0.5;
	double ACHNE_ACT_PERSIST = 0.25;   //% persistence of synaptic current
	double ACHNE_STP_GAIN = 0.1;   //% depressing synapse
	double ACHNE_STP_DECAY = 50;   //% recovery time constant
	double ACHNE_STP_MAX = 1;      //% weight value ceiling

	//calculate cholinergic/noradrenergic neural activity
	for (int i = 1; i <= e; i++)
	{
	//achne(i) =  activity (ACHNE_ACT_BASECURRENT + ACHNE_ACT_PERSIST * achneprev(i) + event(i)*w_e_achne(i), ACHNE_ACT_GAIN);
 	achne[i-1] =  activity(ACHNE_ACT_BASECURRENT + ACHNE_ACT_PERSIST*achneprev[i-1] + event[i-1]*w_e_achne[i-1], ACHNE_ACT_GAIN);
 	ROS_INFO("achne[i-1] %f, achneprev[i-1] %f,",achne[i-1],achneprev[i-1]);
 	ROS_INFO("event[i-1] %f",event[i-1]);
 	}
 	
 	
 	//%achne = ones(1,4); % FOR DISTRACTED BEHAVIOR
	for (int i = 1; i <= e; i++) //FOR DISTRACTED BEHAVIOR
	{
	achne[i-1] = 1;
	}
	
	
	// calculate neuromodulatory activity
	for (int i = 1; i <= NM; i++)
	{
    	//I = NM_ACT_BASECURRENT + NM_ACT_PERSIST * nmprev(i);
    	double I = NM_ACT_BASECURRENT + NM_ACT_PERSIST * nmprev[i-1];
    	
    	for (int j = 1; j <=e; j++)
    	{
    	 //I = I + event(j)*w_e_nm(j,i);
       	 I = I + event[j-1]*w_e_nm[j-1][i-1];
       	}
       	
       	//nm(i) =  activity (I, NM_ACT_GAIN);
    	nm[i-1] =  activity (I, NM_ACT_GAIN);
    }
    
    
    // calculate state neural activity
	for (int i = 1; i <= N; i++)
	{
		//I = N_ACT_BASECURRENT+0.5*rand+N_ACT_PERSIST * nprev(i);
		double I = N_ACT_BASECURRENT+0.5*rand_num()+N_ACT_PERSIST * nprev[i-1];
		
		//intrinsic synaptic input
		for (int j = 1; j <= N; j++)
		{
			//I = I + nprev(j) * w_n_n_exc(j,i) + (sum(nm)) * nprev(j) * w_n_n_inh(j,i);
			I = I + nprev[j-1] * w_n_n_exc[j-1][i-1] + (sum_vector(nm)) * nprev[j-1] * w_n_n_inh[j-1][i-1];
		}
		
		//event synaptic input
		for (int j = 1; j <= e; j++)
		{
			for (int k = 1; k <= NM; k++)
			{
				//I = I + nm(k) * w_nm_n(k,i) * achne(j)* event(j) * w_e_n(j,i);
				I = I + nm[k-1] * w_nm_n[k-1][i-1] * achne[j-1]* event[j-1] * w_e_n[j-1][i-1];
			}
		}
		
		//n(i) = activity (I, N_ACT_GAIN);
		n[i-1] = activity (I, N_ACT_GAIN);
		ROS_INFO("n[i-1]: %f",n[i-1]);
	}
	
	//update plastic weights with short-term plasticity rule. a spike occurs
	//when an event occurs.
	for (int i = 1; i <= e; i++)
	{
		//w_e_achne(i) = stp (w_e_achne(i), ACHNE_STP_GAIN, ACHNE_STP_DECAY, ACHNE_STP_MAX, event(i) > 0.5);
		w_e_achne[i-1] = stp (w_e_achne[i-1], ACHNE_STP_GAIN, ACHNE_STP_DECAY, ACHNE_STP_MAX, event[i-1] > 0.5);
	}
	
	for (int i = 1; i <= e; i++)
	{
		for (int j = 1; j <= NM; j++)
		{
			//if w_e_nm (i,j) > 0
            //	w_e_nm (i,j) = stp (w_e_nm (i,j), NM_STP_GAIN, NM_STP_DECAY, NM_STP_MAX, event(i) > 0.5);
        	//end
        	
        	if ((w_e_nm[i-1][j-1]) > 0)
        		w_e_nm[i-1][j-1] = stp (w_e_nm[i-1][j-1], NM_STP_GAIN, NM_STP_DECAY, NM_STP_MAX, event[i-1] > 0.5);
		}
	}
	
	//find most active state neuron. perform action selection if activity is
	//above threshold
	//[y,i] = max(n)
	double y;
	double i_active;
	max_vector_element(n,y,i_active);
	if (y > ACTION_SELECTION_THRESHOLD){
		choice = i_active;}
	else 
		{choice = -1;} //if no neuron active, no selection made (selection depends on chc >= 0)
		
	achne_out = achne;
	n_out = n;
	nm_out = nm;

} 

void CoreInfo(const kobuki_msgs::SensorState::ConstPtr& msg2)
//void IRInfo(const complete_test::irobotdock::ConstPtr& msg)
{
	//ROS_INFO("Battery voltage: [%i]",msg2->battery);
	
	if (battery_lock == 0){
	battery_initial_level = msg2->battery; 
	battery_lock++; //locks up initial value of battery voltage
	}
	else{
	battery_level = msg2->battery; 
	}
	
	
	
	//ROS_INFO("Bumper Values: [%i]", msg2->bumper);
	bumper = msg2->bumper;
	//ROS_INFO("Charge Values: [%i]", msg2->charger);
	charger = msg2->charger;
	//ROS_INFO("Wheel Drops: [%i]", msg2->wheel_drop);
	
	
}

void IRInfo(const kobuki_msgs::DockInfraRed::ConstPtr& msg)
//void IRInfo(const complete_test::irobotdock::ConstPtr& msg)
{
	//ROS_INFO("Data 0: [%i]",msg->data[0]);
	//ROS_INFO("Data 1: [%i]",msg->data[1]);
	//ROS_INFO("Data 2: [%i]",msg->data[2]);
	
	IR_Sensor[0] = msg->data[0]; //data to be passed onto main subfunction
	IR_Sensor[1] = msg->data[1];
	IR_Sensor[2] = msg->data[2];
	
}

void LsrInfo(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//int times_run;
	
	//ros::param::set("/camera/depthimage_to_laserscan_loader/range_max", 10.0);
	//ROS_INFO("Begin loop - LaserScan Info");
	//ROS_INFO("Range Min: [%f]", msg->range_min);
  	//ROS_INFO("Range Max: [%f]", msg->range_max);
  	//ROS_INFO("Size of Ranges: [%lu]",msg->ranges.size());
  	float minVal = (msg->range_max); //to begin loop
  	float finalVal = 0;
  	for (int i = 0;i<(msg->ranges.size()); i++){
  		if ((msg->range_min) <= (msg->ranges[i]) && (msg->ranges[i]) <= (msg->range_max)){  //if in range, valid dist.
  			if ((msg->ranges[i]) < minVal){													//if lower than prev. min. dist.
  				minVal = (msg->ranges[i]);
  				finalVal = minVal;
  			}
  		}
  	}
  	
  	
  	
  	
  	
  	
  	ROS_INFO("Min. Dist: [%f]", finalVal);
  	camera = finalVal;
  	/*if (locate_enable > 0)
  	{
  	  ROS_INFO("--DIRECTION RECORDED!!!!!!--");
  	  ROS_INFO("Location Enable: [%i]", locate_enable);
  	  locate[2] = finalVal;
  	  //ros::Duration(2).sleep();
  	}
  	*/
  	
  	switch(locate_enable){
  	case 1:
  	locate[0] = finalVal;
  	//ROS_INFO("LOC 1: [%f]", finalVal);
  	break;
  	case 2:
  	locate[1] = finalVal;
  	//ROS_INFO("LOC 2: [%f]", finalVal);
  	break;
  	case 3:
  	locate[2] = finalVal;
  	//ROS_INFO("LOC 3: [%f]", finalVal);
  	break;
  	case 4:
  	locate[3] = finalVal;
  	//ROS_INFO("LOC 4: [%f]", finalVal);
  	break;
  	case 5:
  	locate[4] = finalVal;
  	//ROS_INFO("LOC 5: [%f]", finalVal);
  	break;
  	}
  	
  
  	//ROS_INFO("BEFORE FUNCTION");
  	//ROS_INFO("DIRECTION #: [%i]", dir_vector);
  	//SetFront(dir_vector, finalVal);
  	//ROS_INFO("AFTER FUNCTION");
  	//dir_vector++;
  	//ROS_INFO("DIRECTION #: [%i]", dir_vector);
  	//WallFollow(finalVal);
  /*	
  	SetFront(finalVal);
  	SetRightFront(finalVal);
  	SetRight(finalVal);
  	SetLeft(finalVal);
  	SetLeftFront(finalVal);
  */	
	//ROS_INFO("End loop - LaserScan Info");
}

void OdomInfo2(const nav_msgs::Odometry::ConstPtr& msg)
{


 if (locate_enable == 0){ //resets odometry
  
  	std_msgs::Empty resetodom;
  	
 	double secs = ros::Time::now().toSec();
	while ((ros::Time::now().toSec() - secs) <= 1.5){
 	chatter_pub2.publish(resetodom);
 	}
 }



 if (enable_sub == 1){ //to use vectorized distances
    //ROS_INFO("ODOM ACTIVATED!");
	double yaw;
	
	yaw = tf::getYaw(msg->pose.pose.orientation);
	//ROS_INFO("Yaw Angle: [%f]", yaw);
	
	geometry_msgs::Twist vel;
 	float deg90 = (PI/2);
 	float deg45 = (PI/4);
 	float deg180 = (PI);
 	float destrad;
 	
 	
 	if (controlint == 0)
 	{
 		originyaw = yaw;
 		ROS_INFO("Origin locked in: [%f]", originyaw);
 		controlint++;
 	} 

	switch (stage){
	case 0: //turn right -45 degrees
	{
	locate_enable = 1;
	ROS_INFO("--Case 0---");
	destrad = DestinRad(originyaw, deg45);
	if ((tf::getYaw(msg->pose.pose.orientation) + (fromright*2*PI)) > destrad){
	
	//ROS_INFO("--START Case 0--");
	//ROS_INFO("Current Yaw: [%f]", (tf::getYaw(msg->pose.pose.orientation) + (fromright*2*PI)));
	//ROS_INFO("Destrad: [%f]", destrad);
	//ROS_INFO("Stage: [%i]", stage);
	vel.linear.x = 0.0;
	vel.angular.z = -1.0;
	ROS_INFO("Origin locked in: [%f]", originyaw);
	ROS_INFO("Yaw Angle Target (-45 degrees): [%f]", destrad);
	ROS_INFO("Yaw Angle Progress (-45 degrees): [%f]",tf::getYaw(msg->pose.pose.orientation));
	}
	else{
	ROS_INFO("--DONE Case 0--");
	vel.linear.x = 0.0;
	vel.angular.z = 0.0;
	ROS_INFO("Yaw Angle Target (-45 degrees): [%f]", destrad);
	ROS_INFO("Yaw Angle Progress (-45 degrees): [%f]",tf::getYaw(msg->pose.pose.orientation));
	//ROS_INFO("90 degrees done");
	stage++;
	controlint= 0;
	//ros::Duration(3).sleep();
	}
	chatter_pub.publish(vel);
	//ROS_INFO("Stage: [%i]", stage);
	ROS_INFO("--PUBLISHING--");
	
	}
	break;
	case 1: //turn right 45 degrees
	{
	locate_enable = 2;
	ROS_INFO("--Case 1---");
	//ROS_INFO("Stage: [%i]", stage);
	destrad = DestinRad(originyaw, deg45);
	if ((tf::getYaw(msg->pose.pose.orientation) + (fromright*2*PI)) > destrad)
	{
	vel.linear.x = 0.0;
	vel.angular.z = -1.0;
	ROS_INFO("Origin locked in: [%f]", originyaw);
	ROS_INFO("Yaw Angle Target (-45 degrees): [%f]", destrad);
	ROS_INFO("Yaw Angle Progress (-45 degrees): [%f]",tf::getYaw(msg->pose.pose.orientation));
	}
	else
	{
	//locate_enable = 2;
	vel.linear.x = 0.0;
	vel.angular.z = 0.0;
	ROS_INFO("Yaw Angle Target (-45 degrees): [%f]", destrad);
	ROS_INFO("Yaw Angle Progress (-45 degrees): [%f]",tf::getYaw(msg->pose.pose.orientation));
	//ROS_INFO("90 degrees done");
	stage++;
	controlint = 0;
	//ros::Duration(3).sleep();
	}
	chatter_pub.publish(vel);
	//ROS_INFO("--PUBLISHING--");
	}
	break;
	case 2: //turn left 180 degrees
	{
	locate_enable = 3;
	ROS_INFO("--Case 2---");
	destrad = DestinRad(originyaw, deg180);
	if ((tf::getYaw(msg->pose.pose.orientation) - (fromleft*2*PI)) <= destrad)
	{
	vel.linear.x = 0.0;
	vel.angular.z = 1.0;
	ROS_INFO("Origin locked in: [%f]", originyaw);
	ROS_INFO("Yaw Angle Target (+180 degrees): [%f]", destrad);
	ROS_INFO("Yaw Angle Progress (+180 degrees): [%f]",tf::getYaw(msg->pose.pose.orientation));
	}
	else
	{
	//locate_enable = 3;
	vel.linear.x = 0.0;
	vel.angular.z = 0.0;
	ROS_INFO("Yaw Angle Target (+180 degrees): [%f]", destrad);
	ROS_INFO("Yaw Angle Progress (+180 degrees): [%f]",tf::getYaw(msg->pose.pose.orientation));
	//ROS_INFO("90 degrees done");
	stage++;
	controlint = 0;
	//ros::Duration(3).sleep();
	}
	chatter_pub.publish(vel);
	//ROS_INFO("--PUBLISHING--");
	}
	break;
	case 3: //turn right 45 degrees
	{
	locate_enable = 4;
	ROS_INFO("--Case 3---");
	destrad = DestinRad(originyaw, deg45);
	if ((tf::getYaw(msg->pose.pose.orientation) + (fromright*2*PI)) > destrad)
	{
	vel.linear.x = 0.0;
	vel.angular.z = -1.0;
	ROS_INFO("Origin locked in: [%f]", originyaw);
	ROS_INFO("Yaw Angle Target (-45 degrees): [%f]", destrad);
	ROS_INFO("Yaw Angle Progress (-45 degrees): [%f]",tf::getYaw(msg->pose.pose.orientation));
	}
	else
	{
	//locate_enable = 4;
	vel.linear.x = 0.0;
	vel.angular.z = 0.0;
	ROS_INFO("Yaw Angle Target (-45 degrees): [%f]", destrad);
	ROS_INFO("Yaw Angle Progress (-45 degrees): [%f]",tf::getYaw(msg->pose.pose.orientation));
	//ROS_INFO("90 degrees done");
	stage++;
	controlint= 0;
	//ros::Duration(3).sleep();
	}
	chatter_pub.publish(vel);
	//ROS_INFO("--PUBLISHING--");
	}
	break;
	case 4: //turn right 45 degrees
	{
	locate_enable = 5;
	ROS_INFO("--Case 4---");
	destrad = DestinRad(originyaw, deg45);
	if ((tf::getYaw(msg->pose.pose.orientation) + (fromright*2*PI)) > destrad)
	{
	vel.linear.x = 0.0;
	vel.angular.z = -1.0;
	ROS_INFO("Origin locked in: [%f]", originyaw);
	ROS_INFO("Yaw Angle Target (-45 degrees): [%f]", destrad);
	ROS_INFO("Yaw Angle Progress (-45 degrees): [%f]",tf::getYaw(msg->pose.pose.orientation));
	}
	else
	{
	//locate_enable = 5;
	vel.linear.x = 0.0;
	vel.angular.z = 0.0;
	ROS_INFO("Yaw Angle Target (-45 degrees): [%f]", destrad);
	ROS_INFO("Yaw Angle Progress (-45 degrees): [%f]",tf::getYaw(msg->pose.pose.orientation));
	//ROS_INFO("90 degrees done");
	stage++;
	controlint= 0;
	//ros::Duration(3).sleep();
	}
	chatter_pub.publish(vel);
	//ROS_INFO("--PUBLISHING--");
	}
	break;
	case 5: //origin point
	{ROS_INFO("---DONE!----");
	
	//reveal_vector(locate);
	//ros::Duration(3).sleep();
	stage = 0;
	controlint = 0;
	enable_sub = 0; //turn off vectorized distances
	locate_enable = 0;
	}
	break;
	default:
	{
	ROS_INFO("--DEFAULT--");
	controlint = 0;
	stage = 0;
	}
	break;
	}
	
	}
	
 else{
 	ROS_INFO("Turning subfunction not activated");
 	}


}



void getinfo(double duration)
{	ROS_INFO("Inside GetInfo Loop!");
	double secs =ros::Time::now().toSec();
	ROS_INFO("Get Info Loop SECS! [%f]", secs);
	while ((ros::Time::now().toSec() - secs) <= duration)
	{
	ROS_INFO("Inside GetInfo While Loop!");
	//ROS_INFO("Sleeping");
	//ros::spinOnce();
	//ros::spin();
	
	//ros::AsyncSpinner spinner(4); // Use 4 threads
   	//spinner.start();
   	
   	
	ROS_INFO("End of GetInfo While Loop!");
	}
	ROS_INFO("Done with GetInfo Loop!");
	
	//ros::AsyncSpinner spinner(4); // Use 4 threads
   	//spinner.start();
	
}

void WallFollow(float bump, float distVal)
{

ROS_INFO("--WALL FOLLOW--");
ROS_INFO("Bump [%f]", bump);
ROS_INFO("DistVal [%f]", distVal);

geometry_msgs::Twist vel;
//int wall;
//float WALL_CLOSE = 1.5;
//float WALL_FAR = WALL_CLOSE + 2;
float WALL_CLOSE = 0.53;
//float WALL_FAR = WALL_CLOSE + 0.2;
float WALL_FAR = 0.77;
int condition;
//vel.angular.z = 0.0;
//vel.linear.x = 0.0;
float duration;


ROS_INFO("Wall Follow");
	if (bump == 1) 
	{
	vel.linear.x = 0.0;
	vel.angular.z = 0.8;
	ROS_INFO("Bump - Wall Follow");
	condition = 1;
	duration = 1.5;
	}
	else if (distVal > WALL_FAR)
	{
	vel.linear.x = 0.3;
	vel.angular.z = 0.4*(-wall);
	ROS_INFO("distVal > WALL_FAR"); 
	condition = 2;
	duration = 1.0;
	}
	else if (distVal < WALL_CLOSE)
	{
	vel.linear.x = 0.3;
	vel.angular.z = 0.4*wall;
	ROS_INFO("distVal < WALL_CLOSE");
	condition = 3;
	duration = 1.0;
	}
	else 
	{
	vel.linear.x = 0.3;
	vel.angular.z = 0.0;
	ROS_INFO("Straight Ahead");
	condition = 4;
	duration = 1.0;
	}

	double secs =ros::Time::now().toSec();
	while ((ros::Time::now().toSec() - secs) <= duration){
	chatter_pub.publish(vel);
	ROS_INFO("--Wall Follow Stats [%f], [%f]---", bump, distVal);
	ROS_INFO("--Wall Follow Parameters - Condition: [%i], Wall: [%i], Left: [%f], Right: [%f]", condition, wall, locate[2], locate[1]); 
	}
	//return;
}

void OpenField(float bump, float left, float front, float right){

ROS_INFO("--OPEN FIELD--");
ROS_INFO("--Left: [%f], Front: [%f], Right: [%f]--", left, front, right);

float MAXSPEED = 0.40;
float MINSPEED = 0.10;
float MAXDIST = 300;
float TURN = 0.25;
geometry_msgs::Twist vel;
//int bump = 0;
float duration;

//find the most open area. speed is proportional to the amount of open space
//go straight
if (front > left && front > right){ 
    if (front > MAXDIST){
        front = MAXDIST;}
    
    if (bump == 1){
        //SetFwdVelAngVelCreate(serport, 0.0, 2*TURN);
    vel.linear.x = 0.0;
	vel.angular.z = 0.8;
	duration = 1.5;
	ROS_INFO("---OPEN FIELD - Go Straight - Bump---");
}
    else{
        //SetFwdVelAngVelCreate(serport, max(MINSPEED,(front/MAXDIST)^2*MAXSPEED), 0.0);
    vel.linear.x = 0.3;
	vel.angular.z = 0.0;
	duration = 1.0;
	ROS_INFO("---OPEN FIELD - Go Straight - NO Bump---");
}
}    

//go left
else if (left > right){ 
    if (left > MAXDIST){
        left = MAXDIST;}
    
    if (bump == 1){
        //SetFwdVelAngVelCreate(serport, 0.0, 2*TURN);
    vel.linear.x = 0.0;
	vel.angular.z = 0.8;
	duration = 1.5;
	ROS_INFO("---OPEN FIELD - Go Left - Bump---");
}
    else{
        //SetFwdVelAngVelCreate(serport, max(MINSPEED,(left/MAXDIST)^2*MAXSPEED), TURN);
    vel.linear.x = 0.3;
	vel.angular.z = 0.4;
	duration = 1.0;
	ROS_INFO("---OPEN FIELD - Go Left - NO Bump---");
}
}

else{ //go to the right
    if (right > MAXDIST){
        right = MAXDIST;}
    if (bump == 1){
        //SetFwdVelAngVelCreate(serport, 0.0, 2*TURN);
    vel.linear.x = 0.0;
	vel.angular.z = 0.8;
	duration = 1.5;
	ROS_INFO("---OPEN FIELD - Go Right - Bump---");
}
    else{
        //SetFwdVelAngVelCreate(serport, max(MINSPEED,(right/MAXDIST)^2*MAXSPEED), -1*TURN);
    vel.linear.x = 0.3;
	vel.angular.z = -0.4;
	duration = 1.0;
	ROS_INFO("---OPEN FIELD - Go Right - NO Bump---");
}
}		

	double secs =ros::Time::now().toSec();
	while ((ros::Time::now().toSec() - secs) <= duration){
		chatter_pub.publish(vel);}
}

void ExploreObject(float bump, float left, float front, float right){

ROS_INFO("--EXPLORE OBJECT--");
ROS_INFO("--Left: [%f], Front: [%f], Right: [%f]--", left, front, right);

float MAXSPEED = 0.25;
float MINSPEED = 0.10;
float MAXDIST = 300;
float TURN = 0.5;
//int bump = 0;
geometry_msgs::Twist vel;

float duration;

//speed is proportional to the amount of change
if (front > left && front > right){ //% go straight
    if (front > MAXDIST){
        front = MAXDIST;}
    
    if (bump == 1){
        //SetFwdVelAngVelCreate(serport, 0.0, 2*TURN);
    vel.linear.x = 0.0;
	vel.angular.z = 0.8;
	duration = 1.5;
	ROS_INFO("---EXPLORE OBJ - Go Straight - Bump---");
}
    else{
        //SetFwdVelAngVelCreate(serport, max(MINSPEED,(front/MAXDIST)^2*MAXSPEED), 0.0);
    vel.linear.x = 0.3;
	vel.angular.z = 0.0;
	duration = 1.0;
	ROS_INFO("---EXPLORE OBJ - Go Straight - NO Bump---");
}
}

    
else if (left > right){     //% go to the left
    if (left > MAXDIST){
        left = MAXDIST;}
        
    if (bump == 1){
        //SetFwdVelAngVelCreate(serport, 0.0, 2*TURN);
    vel.linear.x = 0.0;
	vel.angular.z = 0.8;
	duration = 1.5;
	ROS_INFO("---EXPLORE OBJ - Go Left - Bump---");
}
    else{
        //SetFwdVelAngVelCreate(serport, max(MINSPEED,(left/MAXDIST)^2*MAXSPEED), TURN);
    vel.linear.x = 0.3;
	vel.angular.z = 0.4;
	duration = 1.0;
	ROS_INFO("---EXPLORE OBJ - Go Left - NO Bump---");
}
}        
    
else{ //% go to the right 
    if (right > MAXDIST){
        right = MAXDIST;}
    
    if (bump == 1){
        //SetFwdVelAngVelCreate(serport, 0.0, 2*TURN);
    vel.linear.x = 0.0;
	vel.angular.z = 0.8;
	duration = 1.5;
	ROS_INFO("---EXPLORE OBJ - Go Right - Bump---");
}
    else{
        //SetFwdVelAngVelCreate(serport, max(MINSPEED,(right/MAXDIST)^2*MAXSPEED), -1*TURN);
	vel.linear.x = 0.3;
	vel.angular.z = -0.4;
	duration = 1.0;
	ROS_INFO("---EXPLORE OBJ - Go Right - NO Bump---");
}
}		
	double secs =ros::Time::now().toSec();
	while ((ros::Time::now().toSec() - secs) <= duration){
		chatter_pub.publish(vel);}
}

void FindHome(){
ROS_INFO("--FIND HOME--");
//system("roslaunch kobuki_auto_docking activate.launch --screen");
}

void AtHome(){
ROS_INFO("--AT HOME--");
}

void LeaveHome(){
ROS_INFO("--LEAVE HOME--");
geometry_msgs::Twist vel;

	double secs =ros::Time::now().toSec();
	while ((ros::Time::now().toSec() - secs) <= 1.0){
	vel.linear.x = -0.3;
	vel.angular.z = 0.0;
	chatter_pub.publish(vel);
	}

	double secs2 =ros::Time::now().toSec();
	while ((ros::Time::now().toSec() - secs2) <= 1.0){
	vel.linear.x = 	0.0;
	vel.angular.z = 0.3;
	chatter_pub.publish(vel);
	}
}



void main_neuromodulated_program_direct_sensor()
{
	//initialize neural network
	roomba_net_init();
	
	ros::AsyncSpinner spinner(4); // Use 4 threads
   	spinner.start();
	
    ROS_INFO("Initialization Done!");
	double tic = ros::Time::now().toSec(); //start timer
	ROS_INFO("Tic Saved!");
	//ros::Duration(5).sleep();
	
	
	double behave_state_time = (ros::Time::now().toSec()) - tic;
	int behave_state = STATE_WALL_FOLLOW;
	int new_behave_state = behave_state;
	ROS_INFO("Behave State Time Saved!");
	//getinfo(2.5); //% initial battery level
	//ROS_INFO("GetInfo Activated!");
	double current_time = (ros::Time::now().toSec()) - tic;
	//loginx = 0;
	//running the network for approximately five minutes or whatever minutes...
	ROS_INFO("Current Time Logged!: [%f]", current_time);
	//ros::Duration(5).sleep();
	
	
	
	
	
	while (current_time < 300){
	
	current_time = (ros::Time::now().toSec()) - tic;
	ROS_INFO("Inner Loop - Current Time Logged!: [%f]", current_time);
	//ros::Duration(5).sleep();
	
	
	//vectorize_sensor_reading -- done in Odom2 subroutine
	//vectorize_sensor_reading = [front right left right_close_front left_close_front]
	
	enable_sub = 1; //activate sensor vectoring
	
	
	while(enable_sub == 1){ //stays in loop until vectoring is done
	ROS_INFO("Sensor Vectoring in Progress - Main Subroutine Paused");
	}

	ROS_INFO("Sensor Vectoring Finished");
	
	ROS_INFO("WHILE LOOP DONE!!!!");
	//spinner.stop();
	//ros::Duration(15).sleep();
	
	
	//get bump sensor information -- done in CoreInfo subroutine
	//spinner.start();
	//get dock beam status - get information from IR and Core(charging) subroutine
	int beam; //true if home base detected
	
	if ((IR_Sensor[0] > 0) || (IR_Sensor[1] > 0) || (IR_Sensor[2] > 0)){
	beam = 1;
	}
	else{
	beam = 0;
	}
	
	
	int homed; //homed if charging or close to the docking station
	
	if ((beam == 1) || (charger > 0)){
	homed = 1;
	}
	else{
	homed = 0;
	}
	
	battery = battery_level/battery_initial_level;
	
	//% get events (binary 1 == event occurred)
    //event(e_battery) = rand < (1-battery);  % event more likely as battery level drops
    
    if (rand_num() < (1 - battery)){
    event[e_battery-1] = 1;
    }
    else{ 
    event[e_battery-1] = 0;
    }
    
    //event(e_beam) = beam ~=0;
    
    if (beam != 0){
    event[e_beam-1] = 1;
    }
    else{ 
    event[e_beam-1] = 0;
    }
	
	//event(e_bump) = BumpLeft || BumpRight || BumpFront || min(vectorize_sensor_reading) < too_close; % for 5 sensors
	
	if ((bumper > 0) || (min_vector(locate) < too_close)){
	event[e_bump-1] = 1;
	}
	else{
	event[e_bump-1] = 0;
	}
	
	if (min_vector(locate) < comparing_distance){
	event[e_ping_value-1] = 1;
	}
	else{
	event[e_ping_value-1] = 0;
	}
	//% for 5 sensors comparing_max_distance = 70
	
	//special processing for returning home
	if (behave_state == STATE_FIND_HOME){
	
		//if near docking station transition to at home sub-state
		if (homed == 1){
            new_behave_state = STATE_AT_HOME;}
        //if timed out searching for docking station, abort search if no
        //beam detected. if beam is detected continue searching
        else if ((current_time-behave_state_time) > FIND_HOME_TIMEOUT){
        	if (beam == 0)
        		new_behave_state = STATE_LEAVE_HOME;
        	else
        		behave_state_time = (ros::Time::now().toSec()) - tic;  //there was a beam, try a little longer
        	}
        }
     
	else if (behave_state == STATE_AT_HOME){
	new_behave_state = STATE_LEAVE_HOME;}
	else if (behave_state == STATE_LEAVE_HOME){
	new_behave_state = STATE_WALL_FOLLOW;}
	
	//action selection based on neural network activity. chc is non-zero
    //if action is selected by the network
    else{ ROS_INFO("ROOMBA NET CYCLE ACTIVATED!");
    	roomba_net_cycle(event, chc, achne, n, nm);
    	if (chc >= 0){
    		new_behave_state = (chc+1);}
    }
    
    /*
    spinner.stop();
    ROS_INFO("--State Neurons: n[0]: [%f]", n[0]);
    ROS_INFO("--State Neurons: n[1]: [%f]", n[1]);
    ROS_INFO("--State Neurons: n[2]: [%f]", n[2]);
    ROS_INFO("--State Neurons: n[3]: [%f]", n[3]);
    ROS_INFO("--Choice chc [%i]", chc);
    ROS_INFO("--New Behave State [%i]", new_behave_state);
	ros::Duration(15).sleep();
    
    spinner.start();	
    */
    
    
	//transitioned to a new state, print state information
    if (behave_state != new_behave_state){
        behave_state_time = (ros::Time::now().toSec()) - tic;
	
	switch(new_behave_state){
	case 1: //case STATE_WALL_FOLLOW
		//if min(vectorize_sensor_reading(3),vectorize_sensor_reading(5))<=50 % while using 5 sensors
		//       left                         left_close_front 
		if (min(locate[2], locate[3]) <= 0.77){ //wall value
			wall = WALL_LEFT;}
     		else{
     			wall = WALL_RIGHT;}
     	ROS_INFO("State: WallFollow");
	ROS_INFO("--WALL--", wall);
     	break;
     	
     	case 2: //case STATE_OPEN_FIELD
     	ROS_INFO("State: OpenField");
     	break;
     	
     	case 3: //case STATE_EXPLORE_OBJECT
     	ROS_INFO("State: ExploreObject");
     	break;
     	
     	case 4: //case STATE_FIND_HOME
     	ROS_INFO("State: FindHome");
     	break;
     	
     	case 5: //case STATE_AT_HOME
     	ROS_INFO("State: AtHome");
     	//dock_time = toc;
     	dock_time = (ros::Time::now().toSec()) - tic;
     	break;
     	
     	case 6: //case STATE_LEAVE_HOME
     	ROS_INFO("State: LeaveHome");
     	break;
	}
	
	behave_state = new_behave_state;
	}
	
	//handle states
    switch(behave_state){
    	case 1: //case STATE_WALL_FOLLOW

		if (wall == 0){ //if wall hasn't been defined by previous functions
			if (locate[2] < locate[1]){
				wall = WALL_LEFT;
			}
			else if (locate[1] < locate[2]){
				wall = WALL_RIGHT;
			}
		}		


     		//use the smallest ping sensor's value to the appropriate wall for following
            if (wall == WALL_LEFT){
        	//WallFollow (serRoombaport, event(e_bump), left);
 		WallFollow (event[e_bump-1], locate[2]);}
            else{
            	//WallFollow(serRoombaport, event(e_bump), right);
            	WallFollow(event[e_bump-1], locate[1]);}
            	
            break;
            	
            	
        case 2: //case STATE_OPEN_FIELD
        	//OpenField (serRoombaport, event(e_bump), left, front, right);
        	OpenField(event[e_bump-1], locate[2], locate[4], locate[1]);
        	break;
        	
        case 3: //case STATE_EXPLORE_OBJECT
        	//ExploreObject (serRoombaport, event(e_bump), left, front, right);
    		ExploreObject(event[e_bump-1], locate[2], locate[4], locate[1]);
    		break;
    
    	case 4: //case STATE_FIND_HOME
    		//FindHome (serRoombaport);
    		FindHome();
    		break;
    		
    	case 5: //case STATE_AT_HOME
    		//AtHome (serRoombaport);
    		AtHome();
    		break;
    	
    	case 6: //case STATE_LEAVE_HOME	
    		//LeaveHome (serRoombaport);
    		LeaveHome();
    		break;	
    }
	
	//log state, event, and neural network information for post-processing
	//loginx = loginx + 1;
    //log(loginx, 1) = current_time;
    logvec[0] = current_time;
    //log(loginx, 2) = behave_state;
    logvec[1] = behave_state;
    //log(loginx, 3:3+e-1) = event;
    logvec[2] = event[0];
    logvec[3] = event[1];
    logvec[4] = event[2];
    logvec[5] = event[3];
    //log(loginx, 3+e:3+e+N+NM+e-1) = [n nm achne];
    logvec[6] = n[0];
    logvec[7] = n[1];
    logvec[8] = n[2];
    logvec[9] = n[3];
    logvec[10] = nm[0];
    logvec[11] = nm[1];
    logvec[12] = achne[0];
    logvec[13] = achne[1];
    logvec[14] = achne[2];
    logvec[15] = achne[3];
    //log(loginx,17:20)= w_e_achne; % These weights are getting updated
    logvec[16] = w_e_achne[0]; 
    logvec[17] = w_e_achne[1];
    logvec[18] = w_e_achne[2];
    logvec[19] = w_e_achne[3];
    //log(loginx,21:22) = w_e_nm(1,1:2); % These weights are getting updated
    logvec[20] = w_e_nm[0][0];
    logvec[21] = w_e_nm[0][1];
    //log(loginx,23:24) = w_e_nm(2,1:2);% These weights are getting updated
    logvec[22] = w_e_nm[1][0]; 
    logvec[23] = w_e_nm[1][1];
    //log(loginx,25:26) = w_e_nm(3,1:2);% These weights are getting updated
    logvec[24] = w_e_nm[2][0];
    logvec[25] = w_e_nm[2][1];
    //log(loginx,27:28) = w_e_nm(4,1:2);% These weights are getting updated
    logvec[26] = w_e_nm[3][0];
    logvec[27] = w_e_nm[3][1];
    
    loginx.push_back(logvec);
    //spinner.start();
	} //end of timing while loop
	                                                                     
 spinner.stop();
 //writing of data to xls
 ofstream f("./src/complete_test/src/results.xls");
	//f << m << " " << n << "n";
	
	//Data Headers
	f<<"current_time";
	f<<" ";
	f<<"behave_state";
	f<<" ";
	f<<"event[0]-ping";
	f<<" ";
	f<<"event[1]-battery";
	f<<" ";
	f<<"event[2]-bump";
	f<<" ";
	f<<"event[3]-beam";
	f<<" ";
	f<<"n[0]-Wall_Follow";
	f<<" ";
	f<<"n[1]-Open_Field";
	f<<" ";
	f<<"n[2]-Explore_Object";
	f<<" ";
	f<<"n[3]-Find_Home";
	f<<" ";
	f<<"nm[0]-DA";
	f<<" ";
	f<<"nm[1]-5HT";
	f<<" ";
	f<<"achne[0]-ping";
	f<<" ";
	f<<"achne[1]-battery";
	f<<" ";
	f<<"achne[2]-bump";
	f<<" ";
	f<<"achne[3]-beam";
	f<<" ";
	f<<"w_e_achne[0]-ping_ach";
	f<<" ";
	f<<"w_e_achne[1]-bat_ach";
	f<<" ";
	f<<"w_e_achne[2]-bmp_ach";
	f<<" ";
	f<<"w_e_achne[3]-bea_ach";
	f<<" ";
	f<<"w_e_nm[0][0]-ping_DA";
	f<<" ";
	f<<"w_e_nm[0][1]-ping_5HT";
	f<<" ";
	f<<"w_e_nm[1][0]-bat_DA";
	f<<" ";
	f<<"w_e_nm[1][1]-bat_5HT";
	f<<" ";
	f<<"w_e_nm[2][0]-bmp_DA";
	f<<" ";
	f<<"w_e_nm[2][1]-bmp_5HT";
	f<<" ";
	f<<"w_e_nm[3][0]-bea_DA";
	f<<" ";
	f<<"w_e_nm[3][1]-bea_5HT";
	f<<"\n";
	
	
	//writes data
	for (int i = 0; i < loginx.size(); i++)
  	{
  		for (int j = 0; j < loginx[i].size(); j++)
    	{
    		f << loginx[i][j];
    		f << " ";
    	}	
  		f << "\n";
  	}


}







int main(int argc, char **argv)
{
	
	// %Tag(INIT)%
	ros::init(argc, argv, "nuero");
	// %EndTag(INIT)%
	
	// %Tag(NODEHANDLE)%
  	ros::NodeHandle node_handle;
	// %EndTag(NODEHANDLE)%
	
	//initialize neural network
	//roomba_net_init();
	
	ros::Duration(15).sleep();
	
	ros::Subscriber sub1 = node_handle.subscribe("/odom", 0, OdomInfo2); //Internal Odometry sensors
	ros::Subscriber sub2 = node_handle.subscribe("/scan", 0, LsrInfo); //Kinnect Camera
	ros::Subscriber sub3 = node_handle.subscribe("/mobile_base/sensors/dock_ir", 0, IRInfo); //kobuki IR sensors
	ros::Subscriber sub4 = node_handle.subscribe("/mobile_base/sensors/core", 0, CoreInfo); //kobuki core sensors
	chatter_pub = node_handle.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
	chatter_pub2 = node_handle.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry", 1);
	main_neuromodulated_program_direct_sensor();
	//ros::spin();
	
	/*
	int chc;
	for (int i = 1;i <=10;i++)
	{
	ROS_INFO("---- LOOP NUMBER %i ----",i);
	roomba_net_cycle(event, chc, achne, n, nm);
	ROS_INFO("It worked.");
	ROS_INFO("Chc: %i",chc);
	}
	
	event[0] = 1;
	for (int j = 11;j <=20;j++)
	{
	ROS_INFO("---- LOOP NUMBER %i ----",j);
	roomba_net_cycle(event, chc, achne, n, nm);
	ROS_INFO("It worked.");
	ROS_INFO("Chc: %i",chc);
	}
	*/
}






