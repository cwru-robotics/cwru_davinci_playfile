#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>

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
	
	//Convert into a trajectory message
	trajectory_msgs::JointTrajectory des_trajectory;
	double total_wait_time = 0.0;
	for (unsigned n = 0; n < data.size(); n++){
		trajectory_msgs::JointTrajectoryPoint trajectory_point;
		trajectory_point.positions.resize(14);
		double t_arrival;
		
		//Pack points, one at a time.
		for (int i=0;i<14;i++) {
			trajectory_point.positions[i] = data[n][i];
		}
		t_arrival = data[n][14];
		trajectory_point.time_from_start = ros::Duration(t_arrival);
		
		des_trajectory.points.push_back(trajectory_point);
		
		total_wait_time = t_arrival;
	}
	des_trajectory.header.stamp = ros::Time::now();
	
	//Add an ID.
	cwru_davinci_traj_streamer::trajGoal tgoal;
	tgoal.trajectory = des_trajectory;
	srand(time(NULL));
	tgoal.traj_id = rand();
	
	//Locate and lock the action server
	actionlib::SimpleActionClient<
		cwru_davinci_traj_streamer::trajAction
	> action_client("trajActionServer", true);
	bool server_exists = action_client.waitForServer(ros::Duration(5.0));
	// something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
	ROS_INFO("Waiting for server: ");
	while (!server_exists && ros::ok()) {
		server_exists = action_client.waitForServer(ros::Duration(5.0));
		ROS_WARN("Could not connect to server; retrying...");
	}
	ROS_INFO("SERVER LINK LATCHED");
	
	//Send our message:
	ROS_INFO("Sending trajectory with ID %u", tgoal.traj_id);
	action_client.sendGoal(tgoal);
	
	//action_client.getState();
	
	//Wait for it to finish.
	while(!action_client.waitForResult(ros::Duration(total_wait_time + 2.0)) && ros::ok()){
		ROS_WARN("CLIENT TIMED OUT- LET'S TRY AGAIN...");
		//Could add logic here to resend the request or take other actions if we conclude that
		//the original is NOT going to get served.
	}
	//Report back what happened.
	ROS_INFO(
		"Server state is %s, goal state for trajectory %u is %i",
		action_client.getState().toString().c_str(),
		action_client.getResult()->traj_id,
		action_client.getResult()->return_val
	);
	
	//This has to do with the intermittent "Total Recall" bug that breaks the trajectory interpolator
	//If you see this appear in your execution, you are running the program too soon after starting Gazebo.
	if(action_client.getState() ==  actionlib::SimpleClientGoalState::RECALLED){
		ROS_WARN("Server glitch. You may panic now.");
	}

	return 0;
}
