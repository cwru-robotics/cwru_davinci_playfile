#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/String.h>

#include <cwru_playfile_reader/playfile_record.h>

#include <cwru_davinci_traj_streamer/trajAction.h>

/** \file davinci_playfile_jointspace.cpp
*@brief Top-level playfile reader, JSP version. Takes in a jointspace file and sends commands to a traj streamer.
*
*Reads a joint-space file as described in <a href="../../Joint_Grimoire.pdf">Joint_Grimoire.pdf</a>.
*
*The file is read, checked for size consistency (though not for joint-range viability, nor speed viability)
*The file is packed up as a "trajectory" message and delivered within a "goal" message to the trajectory-streamer action server.
*
*All three readers can either read playfiles directly from a file path
*
*`roslaunch playfile_reader playfile_jointspace ~/ros_ws/absolute/path/to/jointfile.jsp`
*
*	
*<code>cd ~/ros_ws
*roslaunch playfile_reader playfile_jointspace relative/path/to/jointfile.jsp</code>
*
*or from a ROS package
*
*`roslaunch playfile_reader playfile_jointspace generic_package /play/jsp/jointfile.jsp`
*
*Recommended to use with cwru_davinci_traj_streamer_as.
*/

/** @brief main funcion
*/

static bool wfcb_1;
static bool wfcb_2;

static sensor_msgs::JointState p_1;
static sensor_msgs::JointState p_2;

void psm_1_pos_cb(const sensor_msgs::JointState::ConstPtr& incoming){
	p_1 = *incoming;
	wfcb_1 = true;
}
void psm_2_pos_cb(const sensor_msgs::JointState::ConstPtr& incoming){
	p_2 = *incoming;
	wfcb_2 = true;
}


void activeCB(){
	ROS_INFO("Active callback; whatever that is...");
}
void feedbackCB(const control_msgs::FollowJointTrajectoryActionFeedbackConstPtr & fdb){
	ROS_INFO("Feedback callback; whatever that is...");
}
void doneCB_1(const actionlib::SimpleClientGoalState & state, const control_msgs::FollowJointTrajectoryResult::ConstPtr & result){
	ROS_INFO("PSM 1 returned goal state %s", state.getText().c_str());
	wfcb_1 = true;
}
void doneCB_2(const actionlib::SimpleClientGoalState & state, const control_msgs::FollowJointTrajectoryResult::ConstPtr & result){
	ROS_INFO("PSM 2 returned goal state %s", state.getText().c_str());
	wfcb_2 = true;
}

int main(int argc, char **argv) {
	//Set up our node.
	ros::init(argc, argv, "playfile_jointspace");
	ros::NodeHandle nh;

	
	//Locate our file.
	std::string fname;
	//Absolute path mode
	if(argc == 2){
		fname = argv[1];
		ROS_INFO("Absolute file location: %s", fname.c_str());
	}
	//Package file mode
	else if(argc == 3){
		std::string packpart = ros::package::getPath(argv[1]).c_str();
		std::string pathpart = argv[2];
		fname = (packpart + pathpart);
		ROS_INFO("Package file location: %s", fname.c_str());
	}
	else{
		ROS_INFO("argc= %d; missing file command-line argument; halting",argc);
		return 0;
	}
	
	//Read the file in.
	std::ifstream infile(fname.c_str());
	if(!infile){//file couldn't be opened
		ROS_ERROR("Error: file %s could not be opened.", fname.c_str());
		exit(1);
	}
	data_t data;
	infile >> data;
	if (!infile.eof()){
		ROS_ERROR("Error: file %s could not be read properly.", fname.c_str());
		exit(1);
	}
	infile.close();
	
	//Perform a few checks...
	unsigned min_record_size = data[0].size();
	unsigned max_record_size = 0;
	for (unsigned n = 0; n < data.size(); n++) {
		if (max_record_size < data[ n ].size())
			max_record_size = data[ n ].size();
		if (min_record_size > data[ n ].size())
			min_record_size = data[ n ].size();
	}
	if (max_record_size>15) {
		ROS_ERROR("Bad file %s: The largest record has %i fields.", fname.c_str(), max_record_size);
		exit(1);
	}
	if (min_record_size<15) {
		ROS_ERROR("Bad file %s: The smallest record has %i fields.", fname.c_str(), min_record_size);
		exit(1);
	}
	
	//Grab the current position, which will be turned into the first point in the trajectory.
	
	ros::Subscriber state_sub_1;
	ros::Subscriber state_sub_2;
	state_sub_1 = nh.subscribe("/dvrk/PSM1/joint_states", 1, psm_1_pos_cb);
	state_sub_2 = nh.subscribe("/dvrk/PSM2/joint_states", 1, psm_2_pos_cb);
	
	ROS_INFO("Reading robot state...");
	wfcb_1 = false;
	wfcb_2 = false;
	while(!(wfcb_1 && wfcb_2) && ros::ok()){
		ros::spinOnce();
	}
	if(!(wfcb_1 && wfcb_2)){
		ROS_ERROR("Process ended before robot state acquired. Aborting.");
		return 0;
	}
	
	std::vector<trajectory_msgs::JointTrajectoryPoint> joint_trajectories_1 = std::vector<trajectory_msgs::JointTrajectoryPoint>(data.size() + 1);
	std::vector<trajectory_msgs::JointTrajectoryPoint> joint_trajectories_2 = std::vector<trajectory_msgs::JointTrajectoryPoint>(data.size() + 1);
	
	joint_trajectories_1[0].time_from_start = ros::Duration(0.0);
	joint_trajectories_1[0].velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	joint_trajectories_1[0].positions = p_1.position;
	joint_trajectories_2[0].time_from_start = ros::Duration(0.0);
	joint_trajectories_2[0].velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	joint_trajectories_2[0].positions = p_2.position;
	
	//Fill in the other positions in the trajectory, creating velocity goals between them.
	for(int n  = 0; n < data.size(); n++){
		joint_trajectories_1[n+1].time_from_start = ros::Duration(data[n][14] + 0.0);
		joint_trajectories_1[n+1].positions = {
			data[n][0],
			data[n][1],
			data[n][2], 
			data[n][3], 
			data[n][4], 
			data[n][5], 
			data[n][6]
		};
		float dt = joint_trajectories_1[n].time_from_start.toSec() - joint_trajectories_1[n + 1].time_from_start.toSec();
		/*joint_trajectories_1[n + 1].velocities = {
			(joint_trajectories_1[n].positions[0] - joint_trajectories_1[n + 1].positions[0]) / dt,
			(joint_trajectories_1[n].positions[1] - joint_trajectories_1[n + 1].positions[1]) / dt,
			(joint_trajectories_1[n].positions[2] - joint_trajectories_1[n + 1].positions[2]) / dt, 
			(joint_trajectories_1[n].positions[3] - joint_trajectories_1[n + 1].positions[3]) / dt, 
			(joint_trajectories_1[n].positions[4] - joint_trajectories_1[n + 1].positions[4]) / dt, 
			(joint_trajectories_1[n].positions[5] - joint_trajectories_1[n + 1].positions[5]) / dt, 
			(joint_trajectories_1[n].positions[6] - joint_trajectories_1[n + 1].positions[6]) / dt
		};*/
		joint_trajectories_1[n + 1].velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		/*for(int m = 0; m < 7; m++){
			if(joint_trajectories_1[n].positions[m] == joint_trajectories_1[n + 1].positions[m]){
				joint_trajectories_1[n + 1].velocities[m] = 0.0;
			}
		}*/
		
		joint_trajectories_2[n+1].time_from_start = ros::Duration(data[n][14] + 0.0);
		joint_trajectories_2[n+1].positions = {
			data[n][7 ],
			data[n][8 ],
			data[n][9 ], 
			data[n][10], 
			data[n][11], 
			data[n][12], 
			data[n][13]
		};
		/*joint_trajectories_2[n + 1].velocities = {
			(joint_trajectories_2[n].positions[0] - joint_trajectories_2[n + 1].positions[0]) / dt,
			(joint_trajectories_2[n].positions[1] - joint_trajectories_2[n + 1].positions[1]) / dt,
			(joint_trajectories_2[n].positions[2] - joint_trajectories_2[n + 1].positions[2]) / dt, 
			(joint_trajectories_2[n].positions[3] - joint_trajectories_2[n + 1].positions[3]) / dt, 
			(joint_trajectories_2[n].positions[4] - joint_trajectories_2[n + 1].positions[4]) / dt, 
			(joint_trajectories_2[n].positions[5] - joint_trajectories_2[n + 1].positions[5]) / dt, 
			(joint_trajectories_2[n].positions[6] - joint_trajectories_2[n + 1].positions[6]) / dt
		};*/
		joint_trajectories_2[n + 1].velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		/*for(int m = 7; m < 14; m++){
			if(joint_trajectories_2[n].positions[m] == joint_trajectories_2[n + 1].positions[m]){
				std::cout << "CLIPPING";
				joint_trajectories_2[n + 1].velocities[m] = 0.0;
			}
		}*/
	}
	
	
	std::cout << "positions:\n";
	for(int i = 0; i < joint_trajectories_2.size(); i++){
		std::cout << "	";
		for(int j = 0; j < 7; j++){
			std::cout << joint_trajectories_2[i].positions[j] << " ";
		}
		std::cout << "\n";
	}
	
	std::cout << "\nvelocities:\n";
	for(int i = 0; i < joint_trajectories_2.size(); i++){
		std::cout << "	";
		for(int j = 0; j < 7; j++){
			std::cout << joint_trajectories_2[i].velocities[j] << " ";
		}
		std::cout << "\n";
	}
	
	std::cout << "\ntimes:\n";
	for(int i = 0; i <joint_trajectories_2.size(); i++){
		std::cout << "	";
		std::cout << joint_trajectories_2[i].time_from_start;
		std::cout << "\n";
	}
	
	//Populate them into a joint trajectory
	trajectory_msgs::JointTrajectory psm_1_traj;
	psm_1_traj.header.stamp = ros::Time::now();
	psm_1_traj.joint_names = {
		"outer_yaw",
		"outer_pitch",
		"outer_insertion",
		"outer_roll",
		"outer_wrist_pitch",
		"outer_wrist_yaw",
		"jaw"
	};
	psm_1_traj.points = joint_trajectories_1;
	
	trajectory_msgs::JointTrajectory psm_2_traj;
	psm_2_traj.header.stamp = ros::Time::now();
	psm_2_traj.joint_names = {
		"outer_yaw",
		"outer_pitch",
		"outer_insertion",
		"outer_roll",
		"outer_wrist_pitch",
		"outer_wrist_yaw",
		"jaw"
	};
	psm_2_traj.points = joint_trajectories_2;
	
	//Wrap that joint trajectory in a trajectory goal.
	control_msgs::FollowJointTrajectoryGoal psm_1_goal;
	psm_1_goal.trajectory = psm_1_traj;
	control_msgs::FollowJointTrajectoryGoal psm_2_goal;
	psm_2_goal.trajectory = psm_2_traj;
	
	/*//And wrap the goal in a goal action.
	control_msgs::FollowJointTrajectoryActionGoal psm_1_act;
	psm_1_act.header = psm_1_traj.header;
	psm_1_act.goal_id.stamp = ros::Time::now();
	psm_1_act.goal_id.id = rand();
	psm_1_act.goal = psm_1_goal;
	control_msgs::FollowJointTrajectoryActionGoal psm_2_act;
	psm_2_act.header = psm_2_traj.header;
	psm_2_act.goal_id.stamp = ros::Time::now();
	psm_2_act.goal_id.id = rand();
	psm_2_act.goal = psm_2_goal;*/
	
	//Lock the trajectory goal messages
	actionlib::SimpleActionClient<
		control_msgs::FollowJointTrajectoryAction
	> action_client_1("/dv_control/PSM1/joint_traj_controller/follow_joint_trajectory", true);
	bool server_exists = action_client_1.waitForServer(ros::Duration(5.0));
	// something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
	ROS_INFO("Waiting for server 1: ");
	while (!server_exists && ros::ok()) {
		server_exists = action_client_1.waitForServer(ros::Duration(5.0));
		ROS_WARN("Could not connect to server; retrying...");
	}
	ROS_INFO("SERVER LINK 1 LATCHED");
	
	actionlib::SimpleActionClient<
		control_msgs::FollowJointTrajectoryAction
	> action_client_2("/dv_control/PSM2/joint_traj_controller/follow_joint_trajectory", true);
	server_exists = action_client_2.waitForServer(ros::Duration(5.0));
	// something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
	ROS_INFO("Waiting for server 2: ");
	while (!server_exists && ros::ok()) {
		server_exists = action_client_2.waitForServer(ros::Duration(5.0));
		ROS_WARN("Could not connect to server; retrying...");
	}
	ROS_INFO("SERVER LINK 2 LATCHED");
	
	ROS_INFO("Enabling joints...");
	std_msgs::String enable;
	enable.data = "DVRK_POSITION_JOINT";
	ros::Publisher enable_publisher_1 = nh.advertise<std_msgs::String>("/dvrk/PSM1/set_robot_state", 1, true);
	ros::Publisher enable_publisher_2 = nh.advertise<std_msgs::String>("/dvrk/PSM2/set_robot_state", 1, true);
	enable_publisher_1.publish(enable);
	enable_publisher_2.publish(enable);
	
	//Send our goals
	action_client_1.sendGoal(psm_1_goal, &doneCB_1);
	action_client_2.sendGoal(psm_2_goal, &doneCB_2);
	ROS_INFO("Waiting for goal feedback.");
	wfcb_1 = false;
	wfcb_2 = false;
	while(!(wfcb_1 && wfcb_2) && ros::ok()){
		ros::spinOnce();
	}

	return 0;
}
