//Ros Header
#include "ros/ros.h"

//Msg headers
#include <std_msgs/String.h>

#include <geometry_msgs/Twist.h>

#include <sensor_msgs/Image.h> //Depth Image Msgs
#include <sensor_msgs/CameraInfo.h> //CameraInfo Msgs
#include <sensor_msgs/LaserScan.h> //LaserScan Msgs

#include <kobuki_msgs/DockInfraRed.h> //Kobuki IR Sensor Status Msgs
#include <kobuki_msgs/SensorState.h> //Kobuki interal Sensor Msgs

#include <vector>

#include <sstream>
using namespace std;

// Global Variables

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
int wall;
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
int comparing_distance = 20; //% 20 centimeter
//%comparing_max_distance = 70;

// ----------------
// - parameters for battery event
float battery;
float battery_initial_level;

// ----------------
// - parameters for bump event
int too_close = 40; //% centimeter

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
 	
 	/* %achne = ones(1,4); % FOR DISTRACTED BEHAVIOR
	for (int i = 1; r <= e; r++) //FOR DISTRACTED BEHAVIOR
	{
	achne(i-1) = 1;
	}
	*/
	
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
		{choice = 0;}
		
	achne_out = achne;
	n_out = n;
	nm_out = nm;

} 

void CoreInfo(const kobuki_msgs::SensorState::ConstPtr& msg2)
//void IRInfo(const complete_test::irobotdock::ConstPtr& msg)
{
	ROS_INFO("Battery voltage: [%i]",msg2->battery);
	
	battery_initial_level = msg2->battery;
}

void IRInfo(const kobuki_msgs::DockInfraRed::ConstPtr& msg)
//void IRInfo(const complete_test::irobotdock::ConstPtr& msg)
{
	ROS_INFO("Data 0: [%i]",msg->data[0]);
	ROS_INFO("Data 1: [%i]",msg->data[1]);
	ROS_INFO("Data 2: [%i]",msg->data[2]);
}

void getinfo(double duration)
{
	double secs =ros::Time::now().toSec();
	while ((ros::Time::now().toSec() - secs) <= duration)
	{
	//ROS_INFO("Sleeping");
	ros::spinOnce();
	//ros::spin();
	}
}





void main_neuromodulated_program_direct_sensor()
{

	double tic = ros::Time::now().toSec(); //start timer
	
	double behave_state_time = (ros::Time::now().toSec()) - tic;
	int behave_state = STATE_WALL_FOLLOW;
	int new_behave_state = behave_state;
	
	getinfo(0.3); //% initial battery level
	
	double current_time = (ros::Time::now().toSec()) - tic;
	//loginx = 0;
	/*
	while
	{
	
	
	
	
	
	}
	*/
	
	
	

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
	roomba_net_init();
	
	
	ros::Subscriber sub3 = node_handle.subscribe("/mobile_base/sensors/dock_ir", 0, IRInfo); //kobuki IR sensors
	ros::Subscriber sub4 = node_handle.subscribe("/mobile_base/sensors/core", 0, CoreInfo); //kobuki core sensors
	
	
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
	
}






