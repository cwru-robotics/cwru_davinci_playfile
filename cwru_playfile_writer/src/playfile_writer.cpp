#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include <cwru_davinci_interface/davinci_interface.h>

using namespace davinci_interface;

void write_record(const std::vector<std::vector<double> > & poses, double time, std::ofstream & file);

int main(int argc, char **argv) {

	double CAPTURE_TIME = -1.0;
	double CAPTURE_FREQ = -1.0;
	std::string CAPTURE_FILE;
	
	if(argc > 1 && argc < 5){
		CAPTURE_FILE = argv[1];
		if(argc > 2){
			CAPTURE_FREQ = strtod(argv[2], NULL);
			if(CAPTURE_FREQ <= 0.0){
				ROS_ERROR("Custom capture period is %f, it must be greater than zero!", CAPTURE_FREQ);
				return 1;
			}
			if(argc > 3){
				CAPTURE_TIME = strtod(argv[3], NULL);
				if(CAPTURE_FREQ <= 0.0){
					ROS_ERROR("Capture time is %f, it must be greater than zero!", CAPTURE_TIME);
					return 1;
				}
				if(CAPTURE_TIME < CAPTURE_FREQ){
					ROS_ERROR("Capture time is shorter than capture period, nothing will be recorded!");
					return 1;
				}
			}
		}
		else{
			CAPTURE_FREQ = 0.1;
		}
	}
	else{
		printf("Da Vinci playfile recorder. Usage:\n");
		printf("	playfile_writer [filename]\
		 [optional record period (default- 10 Hz)]\
		 [optional record duration (default- record until killed)]"
		);
		return 1;
	}
	
	std::ofstream outfile;
	outfile.open(CAPTURE_FILE.c_str());
	if(!outfile.is_open()){
		ROS_ERROR("Unable to open record file %s", CAPTURE_FILE.c_str());
		return 1;
	}

	ros::init(argc, argv, "playfile_writer");
	ros::NodeHandle nh;
	
	init_joint_feedback(nh);
	
	ROS_INFO("Ready to start capturing- enter '1' to begin.");
	char ans = '0';
	while(ans != '1' && ros::ok()){
		std::cin >> ans;
	}
	
	std::vector<std::vector<double> > mrp;
	get_fresh_robot_pos(mrp);
	write_record(mrp, 0.0, outfile);
	
	ros::spinOnce();
	
	ROS_INFO("Recording. Press Ctrl-C to end.");
	
	double total_time = 0.0;
	while(ros::ok() && (CAPTURE_TIME == -1.0 || total_time < CAPTURE_TIME)){
		ros::Duration sl(CAPTURE_FREQ);
		
		total_time = total_time + CAPTURE_FREQ;
		
		if(get_last_robot_pos(mrp)){
			outfile << "\n";
			write_record(mrp, total_time, outfile);
		}
		
		ros::spinOnce();
		
		sl.sleep();
	}
	
	outfile.close();
	
	return 0;
}

void write_record(const std::vector<std::vector<double> > & poses, double time, std::ofstream & file){
	for(int i = 0; i < 2; i++){
		for(int j = 0; j < 7; j++){
			file << poses[i][j];
			file << ", ";
		}
		file << "	";
	}
	file << time;
}
