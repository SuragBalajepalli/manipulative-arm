//6/2/2018
//Surag Balajepalli
//subscribers to a 'virtual attractor' pose
//Performs accommodation control 
#include <irb120_accomodation_control/irb120_accomodation_control.h>
#include <cmath>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>


//global vars for subscribers
Eigen::VectorXd wrench_body_coords_ = Eigen::VectorXd::Zero(6);
Eigen::VectorXd joint_states_ = Eigen::VectorXd::Zero(6);
Eigen::VectorXd desired_ee_pos(6);
Eigen::MatrixXd accomodation_gain(6,6);
bool cmd = false; // this is spaghetti. Fix it!




Eigen::Matrix3d vectorHat(Eigen::Vector3d vector) { //for lack of a better name
		Eigen::Matrix3d hat_of_vector;
		hat_of_vector(0,0) = 0;
		hat_of_vector(0,1) = - vector(2);
		hat_of_vector(0,2) = vector(1);
		hat_of_vector(1,0) = vector(2);
		hat_of_vector(1,1) = 0;
		hat_of_vector(1,2) = - vector(0);
		hat_of_vector(2,0) = - vector(1);
		hat_of_vector(2,1) = vector(0);
		hat_of_vector(2,2) = 0;
		return hat_of_vector;
	}

Eigen::Vector3d decompose_rot_mat(Eigen::Matrix3d rot_mat) {
	//takes rot mat and decomposes it to phi_x, phi_y, phi_z
	Eigen::Vector3d vec_of_angles;
	vec_of_angles(0) = atan2(rot_mat(2,1),rot_mat(2,2));
	vec_of_angles(1) = atan2(-1 * rot_mat(2,0), sqrt(pow(rot_mat(2,1),2) + pow(rot_mat(2,2),2)));
	vec_of_angles(2) = atan2(rot_mat(1,0), rot_mat(0,0)); 
	return vec_of_angles;
}






void ftSensorCallback(const geometry_msgs::WrenchStamped& ft_sensor) {
	//subscribe to "robotiq_ft_wrench"
	//implement low pass filter - TODO
	
	
	wrench_body_coords_(0) = std::round(ft_sensor.wrench.force.x * 10) / 10;
	wrench_body_coords_(1) = std::round(ft_sensor.wrench.force.y * 10) / 10;
	wrench_body_coords_(2) = std::round(ft_sensor.wrench.force.z * 10) / 10;
	wrench_body_coords_(3) = std::round(ft_sensor.wrench.torque.x * 10) / 10;
	wrench_body_coords_(4) = std::round(ft_sensor.wrench.torque.y * 10) / 10;
	wrench_body_coords_(5) = std::round(ft_sensor.wrench.torque.z * 10) / 10;
	
}

void jointStateCallback(const sensor_msgs::JointState& joint_state) {
	//subscribe to "abb120_joint_state"
	for(int i = 0; i < 6; i++) joint_states_(i) = joint_state.position[i] ; //implement low pass filter- TODO

}

void virt_attr_CB(const geometry_msgs::Pose& des_pose) {
	cmd = true;
	desired_ee_pos(0) = des_pose.position.x;
	desired_ee_pos(1) = des_pose.position.y;
	desired_ee_pos(2) = des_pose.position.z;
	//How to convert quaternion to euler angles?
	//method to convert quaternion orientation to rotation matrix
	float a = des_pose.orientation.w;
	float b = des_pose.orientation.x;
	float c = des_pose.orientation.y;
	float d = des_pose.orientation.z;
		
	//seperately filling out the 3x3 rotation matrix
	Eigen::Matrix3d rotation_matrix;
	rotation_matrix(0,0) = pow(a,2) + pow(b,2) - pow(c,2) - pow(d,2);
	rotation_matrix(0,1) = 2 * (b * c - a * d);
	rotation_matrix(0,2) = 2 * (b * d + a * c);
	rotation_matrix(1,0) = 2 * (b * c + a * d);
	rotation_matrix(1,1) = pow(a,2) - pow(b,2) + pow(c,2) - pow(d,2);
	rotation_matrix(1,2) = 2 * (c * d - a * b);
	rotation_matrix(2,0) = 2 * (b * d - a * c);
	rotation_matrix(2,1) = 2 * (c * d + a * b);
	rotation_matrix(2,2) = pow(a,2) - pow(b,2) - pow(c,2) + pow(d,2); 

	Eigen::Vector3d angles = decompose_rot_mat(rotation_matrix);
	desired_ee_pos.tail(3) = angles;

}

void acc_gain_Cb(const std_msgs::Float64MultiArray& acc_gain_diag) {
	for(int i = 0; i<6; i++)accomodation_gain(i,i) = acc_gain_diag.data[i];
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "control_test");
	ros::NodeHandle nh;
	ros::Subscriber virt_attr_sub = nh.subscribe("virt_attr",1,virt_attr_CB);
	ros::Subscriber ft_sub = nh.subscribe("robotiq_ft_wrench", 1, ftSensorCallback);
	ros::Subscriber Ka_sub = nh.subscribe("Ka_diagonal",1, acc_gain_Cb);
	ros::Subscriber joint_state_sub = nh.subscribe("abb120_joint_state",1,jointStateCallback);
	ros::Publisher arm_publisher = nh.advertise<sensor_msgs::JointState>("abb120_joint_angle_command",1);
	ros::Publisher cart_log_pub = nh.advertise<geometry_msgs::Pose>("cartesian_logger",1); 
	ros::Publisher ft_pub = nh.advertise<geometry_msgs::Wrench>("transformed_ft_wrench",1);
	Irb120_fwd_solver irb120_fwd_solver;
	
	Eigen::VectorXd current_ee_pos(6);
	Eigen::VectorXd wrench_wrt_robot(6);
	Eigen::VectorXd desired_twist(6);
	Eigen::VectorXd result_twist(6);

	double dt_ = 0.1;
	int dbg;
	double MAX_JNT_VEL_NORM = 0.1;
	double MAX_JNT_VEL_NORM_WRIST = 1;
	bool is_nan;
	double K_virt = 0.1;
	

	sensor_msgs::JointState desired_joint_state;
	geometry_msgs::Pose cartesian_log;
	geometry_msgs::Pose virt_attr;
	geometry_msgs::Wrench transformed_wrench;
	desired_joint_state.position.resize(6);
	desired_joint_state.velocity.resize(6);
	
	
	//default values for accommodation gain
	accomodation_gain<<1,0,0,0,0,0,
						0,1,0,0,0,0,
						0,0,1,0,0,0,
						0,0,0,1,0,0,
						0,0,0,0,1,0,
						0,0,0,0,0,1;

	accomodation_gain *= -0.0005;
	
	//Begin tool description. Think of another way to make this happen
	double tool_mass = 0.5;
	double tool_length = 0.1;
	Eigen::Vector3d tool_length_vector;
	tool_length_vector<<0,0,tool_length; //For easier math, length vector described in tool frame
	Eigen::Vector3d f_g_r; //gravity vector in robot base frame
	f_g_r<<tool_mass*9.8,0,0;
	//gravity in tool frame, to be computer for every new joint state
	Eigen::Vector3d f_g_t;

	//compensation wrench 
	Eigen::VectorXd f_comp(6);
	//end tool description
	

	//static transform for sensor
	Eigen::Affine3d sensor_wrt_flange;
	Eigen::Matrix3d sensor_rot;
	Eigen::Vector3d sensor_origin;
	sensor_origin<<0,0,0.1; //approximately
	sensor_rot<<0,-1,0,
				1,0,0,
				0,0,1;
	sensor_wrt_flange.linear() = sensor_rot;
	sensor_wrt_flange.translation() = sensor_origin;


	//static Tf for tool
	Eigen::Affine3d tool_wrt_sensor;
	Eigen::Matrix3d tool_wrt_sensor_rot = Eigen::Matrix3d::Identity();
	Eigen::Vector3d tool_wrt_sensor_trans;
	tool_wrt_sensor_trans<<0,0,0.05;
	tool_wrt_sensor.linear() = tool_wrt_sensor_rot;
	tool_wrt_sensor.translation() = tool_wrt_sensor_trans;
	
	

	ros::Rate naptime(1/dt_);
	

	while(ros::ok()) {
		

		ros::spinOnce();


		//initialize jacobians
		Eigen::MatrixXd jacobian = irb120_fwd_solver.jacobian2(joint_states_);
		Eigen::Affine3d flange_wrt_robot = irb120_fwd_solver.fwd_kin_solve(joint_states_);
		Eigen::Affine3d sensor_wrt_robot = flange_wrt_robot * sensor_wrt_flange;
		Eigen::Affine3d tool_wrt_robot = sensor_wrt_robot * tool_wrt_sensor;
		
		Eigen::FullPivLU<Eigen::MatrixXd> lu_jac(jacobian);

		Eigen::MatrixXd jacobian_inv = lu_jac.inverse(); //what to do when matrix is non invertible?
		Eigen::MatrixXd jacobian_transpose = jacobian.transpose();
		
		
		
		
		//Find current ee pose
		current_ee_pos.head(3) = tool_wrt_robot.translation();
		current_ee_pos.tail(3) = decompose_rot_mat(tool_wrt_robot.linear()); 
		Eigen::Quaterniond quat(tool_wrt_robot.linear());
		//Update and transform force sensor output
		
		
		//to find gravity compensation force - make this more reusable
		//rigid body force vector transformations
		f_g_t = tool_wrt_robot.linear().transpose() * f_g_r;
		f_comp.head(3) = f_g_t;
		f_comp.tail(3) = tool_length_vector.cross(f_g_t);
		
		//compensate for these forces
		Eigen::Vector3d force_tool_frame =  wrench_body_coords_.head(3) - f_comp.head(3);
		Eigen::Vector3d moment_tool_frame =  wrench_body_coords_.tail(3) - f_comp.tail(3);
				
		wrench_wrt_robot.head(3) = sensor_wrt_robot.linear() * force_tool_frame;
		wrench_wrt_robot.tail(3) = (sensor_wrt_robot.linear() * moment_tool_frame);
		for(int i = 0; i < wrench_wrt_robot.size(); i++) wrench_wrt_robot(i) =  std::round(wrench_wrt_robot(i) * 10) / 10; //need a better low pass filter here
		
		//to do or not to do, that is the question
		if(!cmd) desired_ee_pos = current_ee_pos;
		
		//safety until I figure out how to quaternion effectively
		desired_ee_pos.tail(3) =current_ee_pos.tail(3) ;

		//effector of virtual attractor
		desired_twist = K_virt * (desired_ee_pos - current_ee_pos);
		
		//control law. Simple accommodation again
		Eigen::VectorXd result_twist =  desired_twist - accomodation_gain * wrench_wrt_robot;
		
		//twist into joint vels
		Eigen::VectorXd des_jnt_vel = jacobian_inv * result_twist;
		
		
		//Virtual attractor behavior algorithms are now implemented outside this node
		//Method for retracting virtual attractor to surface - Now implemented in an outside node
		/*
		if(abs(wrench_wrt_robot(0)) > X_FORCE_THRESH) {
		//ROS_INFO("Virtual attractor X set to %f", current_ee_pos(0));
			desired_ee_pos = current_ee_pos;
			desired_ee_pos(0) = current_ee_pos(0) + 0.01; // low attraction into the surface, to keep contact
	

			desired_ee_pos(2) = current_ee_pos(2) - 0.02; // high(er) attraction to a point on the surface but offset in the y axis by 5cm, a primitive search method, since we already know where the "goal" is
		}
		else {
			//ROS_INFO("Force in X is %f , no problem", wrench_wrt_robot(0));
			desired_ee_pos = current_ee_pos;
			desired_ee_pos(0) += 0.05;
		*/

		//clip vel command  and remove nan that might have made their way through jacobian inverse
		//nan in jnt vel means Jacobian is losing rank - Fix 1: Stop moving - Make vels 0;
		for(int i = 0; i < 6; i++) { if(isnan(des_jnt_vel(i))) des_jnt_vel<<0,0,0,0,0,0; }
			//if at singularity - just dont move. Redundant test
			//FullPivLu decomposition always provides an inverse - I think?
			
						
		//ensure that desired joint vel is within set limits
		if(des_jnt_vel.head(3).norm() > MAX_JNT_VEL_NORM) des_jnt_vel.head(3) = (des_jnt_vel.head(3) / des_jnt_vel.head(3).norm()) * MAX_JNT_VEL_NORM;
		if(des_jnt_vel.tail(3).norm() > MAX_JNT_VEL_NORM_WRIST) des_jnt_vel.tail(3) = (des_jnt_vel.tail(3) / des_jnt_vel.tail(3).norm()) * MAX_JNT_VEL_NORM_WRIST;

		//euler one step integration to calculate position from velocities
		Eigen::MatrixXd des_jnt_pos = joint_states_ + (des_jnt_vel * dt_);
		
		
		//stuff vel and pos command into Jointstate message and publish
		for(int i = 0; i < 6; i++) desired_joint_state.position[i] = std::round(des_jnt_pos(i) * 1000) /1000; //implement low pass filter here instead
		for(int i = 0; i < 6; i++) desired_joint_state.velocity[i] = std::round(des_jnt_vel(i) * 1000) /1000;
		arm_publisher.publish(desired_joint_state);
		
		//publishing cartesian coordinates of robot end effector
		cartesian_log.position.x = current_ee_pos(0);
		cartesian_log.position.y = current_ee_pos(1);
		cartesian_log.position.z = current_ee_pos(2);
		cartesian_log.orientation.w = quat.w();
		cartesian_log.orientation.x = quat.x();
		cartesian_log.orientation.y = quat.y();
		cartesian_log.orientation.z = quat.z();
		cart_log_pub.publish(cartesian_log);

		//publishing force torque values transformed into robot frame
		transformed_wrench.force.x = wrench_wrt_robot(0);
		transformed_wrench.force.y = wrench_wrt_robot(1);
		transformed_wrench.force.z = wrench_wrt_robot(2);
		transformed_wrench.torque.x = wrench_wrt_robot(3);
		transformed_wrench.torque.y = wrench_wrt_robot(4);
		transformed_wrench.torque.z = wrench_wrt_robot(5);
		ft_pub.publish(transformed_wrench);
		
		
		naptime.sleep();
		ros::spinOnce();
	}
}