# playfile_reader

Node to read a file encoding desired dual-arm joint-space motions, convert to a trajectory, and send as a goal request to the trajectory streamer action server.

Playfile_reader_jointspace takes in a list of successive joint angles ("records") as a Joint Space File.

Additional information on the Joint Space File format can be found in <a href="Joint_Grimoire.pdf">Joint_Grimoire.pdf</a>, section Joint Space File Format; example .jsp files are in <a href="play/jsp">playfile_reader/play/jsp</a>.

Playfile_reader_cartspace and playfile_reader_cameraspace both take in the positions and rotations of the DaVinci grippers, using inverse kinematics to produce and execute a full trajectory.

Playfile_reader_cartspace will read coordinates in the frame of Da Vinci's cameras- specifically, left_camera_optical_frame. Playfile_reader_cartspace uses the PSM coordinate spaces for the respective arms- one_psm_base_link and two_psm_base_link.
<WIP>

The file is read, checked for size consistency (though not for joint-range viability, nor speed viability)
The file is packed up as a "trajectory" message and delivered within a "goal" message to the trajectory-streamer action server.

All three readers can either read playfiles directly from a file path, e.g.:

`roslaunch cwru_playfile_reader playfile_jointspace ~/ros_ws/absolute/path/to/jointfile.jsp`

	
<code>cd ~/ros_ws
roslaunch playfile_reader playfile_jointspace relative/path/to/jointfile.jsp</code>

or from a ROS package

`roslaunch playfile_reader playfile_jointspace generic_package /play/jsp/jointfile.jsp`

## Example usage (updated March, 2017)
Start gazebo:

`roslaunch davinci_gazebo sticky_davinci_gazebo.launch`
or
`roslaunch cwru_davinci_gazebo davinci_yellow_config.launch`
(for the base-frame configuration corresponding to yellow tapes on our DaVinci)

Start trajectory streamer action server:

`rosrun cwru_davinci_traj_streamer davinci_traj_interpolator_as` 

Run a playfile reader.  Navigate to a directory that contains a playfile of interest.  E.g., some joint-space playfiles are in:
`roscd cwru_playfile_reader/play/jsp`

Run the joint-space playfile reader with file argument, e.g.
`rosrun cwru_playfile_reader playfile_jointspace example_wiggle_every_joint.jsp` 


ALTERNATIVE: Cartesian playfile. Read in a file of sequence of desired gripper poses, in Cartesian space.
Perform IK, pack joint-space solutions into a trajectory, and send as goal to trajectory streamer action server:

`rosrun playfile_reader playfile_cartspace testfile.csp`
[[REDO THIS SECTION WHEN THE GRIPPERSPACE FILES ARE IN BETTER CONDITION]]

//files must be designed to account for relative positions of PSM1 and PSM2 base frames w/rt world

//the following are w/rt to frame ONE_PSM_BASE_LINK
//entries 0-2 = origin of PSM1 gripper tip (a point mid-way between the gripper jaw tips)
//entries 3-5 = x-axis direction for PSM1 gripper-tip frame (x-axis points parallel to gripper-jaw rotation axis; y-axis points from fingertip to fingertip)
//entries 6-8 = z-axis direction for PSM1 gripper-tip frame (z-axis points from wrist to tip)
//entry 9 = angle of jaw opening (in radians)

// repeat next entries for PSM2:
//the following are w/rt to frame TWO_PSM_BASE_LINK
//entries 10-12 = origin of PSM1 gripper tip (a point mid-way between the gripper jaw tips)
//entries 13-15 = x-axis direction for PSM1 gripper-tip frame (x-axis points parallel to gripper-jaw rotation axis; y-axis points from fingertip to fingertip)
//entries 16-18 = z-axis direction for PSM1 gripper-tip frame (z-axis points from wrist to tip)
//entry 19 = angle of jaw opening (in radians)

//entry 20 = desired arrival time (from start of trajectory, in seconds)

//each line must contain all 21 values (in fixed order), separated by commas

Modified 9/27/15:
rosrun playfile_reader playfile_cameraspace test_cameraspace.csp
This playfile version assumes desired gripper poses are expressed w/rt left camera optical frame;
Uses tflistener to get transform from optical frame to each PSM base frame, then uses same
IK code;

Looks encouraging.  Gets out of reach for z ~>= 0.16 (along optical axis)
DO get significant gravity droop.  Alignment looks good (red dot on gripper tips
align w/ optical z axis) when gravity is reduced.  (Should improve gains and reduce
masses).

Can start up this way:
`roslaunch dvrk_model wsn_davinci_gazebo.launch`
which starts 2 psm's along with stereo cameras, a table and a 1mm blue bar
`rosrun davinci_traj_streamer davinci_traj_interpolator_as`
to get the trajectory interpolator running
And from playfiles directory, retract grippers with:
`rosrun playfile_reader playfile_cartspace retract.csp`
then can run a camera-space playfile with, e.g.:
`rosrun playfile_reader playfile_cameraspace test_cameraspace.csp`
which interprets a camera-space Cartesian file to drive the grippers.
View in rviz to see grippers from /davinci/left_camera/image_raw






    
