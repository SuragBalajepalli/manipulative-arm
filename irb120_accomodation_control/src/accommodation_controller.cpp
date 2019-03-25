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
int dbg;


//global vars for subscribers
Eigen::VectorXd wrench_body_coords_ = Eigen::VectorXd::Zero(6);
Eigen::MatrixXd wrench_filter = Eigen::MatrixXd::Zero(10,6);
int filter_counter = 0;

Eigen::VectorXd joint_states_ = Eigen::VectorXd::Zero(6);
Eigen::VectorXd virt_attr_delta(3);
Eigen::Matrix3d virt_attr_rot;
Eigen::MatrixXd accomodation_gain(6,6);
bool cmd = false; // this is spaghetti. Fix it!
bool jnt_state_update = false;
Eigen::Quaterniond virt_quat;
double SMALL_ANGLE_THRES = 0.5;

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
	
	if (filter_counter > 9) { //if last 10 readings are recorded
		//low pass filter
		//Block of size (p,q), starting at (i,j); so block(i,j,p,q)
		wrench_filter.block(1,0,9,6) = wrench_filter.block(0,0,9,6); //move everything down
		wrench_filter.block(0,0,1,6) = wrench_body_coords_.transpose(); // top row is the new coordinates

		wrench_body_coords_ = wrench_filter.colwise().sum(); // sums up the columns into an array
		wrench_body_coords_ /= 10; // now we have an average
	}
	else {
		filter_counter++;
		wrench_filter.block(1,0,9,6) = wrench_filter.block(0,0,9,6);
		
		wrench_filter.block(0,0,1,6) = wrench_body_coords_.transpose(); 
		// and then wrench_body_coords_ is just exactly what the sensor reads
	}

}

void jointStateCallback(const sensor_msgs::JointState& joint_state) {
	jnt_state_update = true;
	//subscribe to "abb120_joint_state"
	for(int i = 0; i < 6; i++) joint_states_(i) = joint_state.position[i] ; //implement low pass filter- TODO

}

void virt_attr_CB(const geometry_msgs::PoseStamped& des_pose) {
	cmd = true;

	virt_attr_delta(0) = des_pose.pose.position.x;
	virt_attr_delta(1) = des_pose.pose.position.y;
	virt_attr_delta(2) = des_pose.pose.position.z;

	
	virt_quat.w() = des_pose.pose.orientation.w;
	virt_quat.x() = des_pose.pose.orientation.x;
	virt_quat.y() = des_pose.pose.orientation.y;
	virt_quat.z() = des_pose.pose.orientation.z;
			
	//seperately filling out the 3x3 rotation matrix
	virt_attr_rot = virt_quat.normalized().toRotationMatrix();

}

void acc_gain_Cb(const std_msgs::Float64MultiArray& acc_gain_diag) {
	for(int i = 0; i<6; i++) {
		accomodation_gain(i,i) = acc_gain_diag.data[i];

	}
		accomodation_gain *= 0.0001;
		//cout<<"acc gain"<<accomodation_gain<<endl;
}

Eigen::Vector3d delta_phi_from_rots(Eigen::Matrix3d source_rot, Eigen::Matrix3d dest_rot) {
	//Takes the source and destination orientations as rotation matrices
	//Computes angle of rotation required in each of X, Y, and Z axes to reconcile these rotations
	//Important: Useful for small angle approximation only


	//Todo: Implement method to check whether small angle approximation is appropriate. 

	//This is to ensure that it is 

	Eigen::Vector3d delta_phi;
	Eigen::Matrix3d delta_rot = dest_rot * source_rot.inverse();
	Eigen::AngleAxisd ang_ax(delta_rot);
	if (ang_ax.angle() > SMALL_ANGLE_THRES) {
		float d_ang = ang_ax.angle() / 10;
		Eigen::AngleAxisd detla_ang_ax(d_ang, ang_ax.axis());
		delta_rot = detla_ang_ax.toRotationMatrix();
	}
	delta_phi(0) = (delta_rot(2,1) - delta_rot(1,2)) / 2;
	delta_phi(1) = (delta_rot(0,2) - delta_rot(2,0)) / 2;
	delta_phi(2) = (delta_rot(1,0) - delta_rot(0,1)) / 2;
	return delta_phi;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "acc_controller");
	ros::NodeHandle nh;
	ros::Subscriber virt_attr_sub = nh.subscribe("virt_attr",1,virt_attr_CB);
	ros::Subscriber ft_sub = nh.subscribe("robotiq_ft_wrench", 1, ftSensorCallback);
	//ros::Subscriber Ka_sub = nh.subscribe("Ka_diagonal",1, acc_gain_Cb);
	ros::Subscriber joint_state_sub = nh.subscribe("abb120_joint_state",1,jointStateCallback);
	ros::Publisher arm_publisher = nh.advertise<sensor_msgs::JointState>("abb120_joint_angle_command",1);
	ros::Publisher cart_log_pub = nh.advertise<geometry_msgs::PoseStamped>("cartesian_logger",1); 
	ros::Publisher ft_pub = nh.advertise<geometry_msgs::Wrench>("transformed_ft_wrench",1);
	ros::Publisher virt_attr_after_tf = nh.advertise<geometry_msgs::PoseStamped>("tfd_virt_attr",1);
	Irb120_fwd_solver irb120_fwd_solver;
	
	Eigen::VectorXd current_ee_pos(6);
	Eigen::VectorXd wrench_wrt_robot(6);
	Eigen::VectorXd desired_force(6);
	Eigen::VectorXd result_twist(6);
	Eigen::VectorXd virt_attr_pos(6);
	double dt_ = 0.001;
	int dbg;
	double MAX_JNT_VEL_NORM = 1;
	
	double MAX_TWIST_NORM = 0.01;
	bool is_nan;
	double K_virt = 1000; //10000
	

	sensor_msgs::JointState desired_joint_state;
	geometry_msgs::PoseStamped cartesian_log, virt_attr_log;
	cartesian_log.header.frame_id = "map";
	virt_attr_log.header.frame_id = "map";
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

	accomodation_gain *= 0.0001;
	
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
	

	//Wait until there are some valid values of joint states from the robot controller
	while(!jnt_state_update) ros::spinOnce();
	//initialize current end effector position and use values recieved from hand controller as offsets to that value
	Eigen::Affine3d flange_wrt_robot = irb120_fwd_solver.fwd_kin_solve(joint_states_);
	Eigen::Affine3d sensor_wrt_robot = flange_wrt_robot * sensor_wrt_flange;
	Eigen::Affine3d tool_wrt_robot = sensor_wrt_robot * tool_wrt_sensor;
	Eigen::VectorXd initial_ee_pos = Eigen::VectorXd::Zero(6); 
	initial_ee_pos.head(3) = tool_wrt_robot.translation();
	initial_ee_pos.tail(3) = decompose_rot_mat(tool_wrt_robot.linear());
	//virt_attr_pos.tail(3) = decompose_rot_mat(tool_wrt_robot.linear());
	ros::Rate naptime(1/dt_);
	

	while(ros::ok()) {
		

		ros::spinOnce();


		//initialize jacobians
		Eigen::MatrixXd jacobian = irb120_fwd_solver.jacobian2(joint_states_);
		flange_wrt_robot = irb120_fwd_solver.fwd_kin_solve(joint_states_);
		sensor_wrt_robot = flange_wrt_robot * sensor_wrt_flange;
		tool_wrt_robot = sensor_wrt_robot * tool_wrt_sensor;
		Eigen::FullPivLU<Eigen::MatrixXd> lu_jac(jacobian);

		Eigen::MatrixXd jacobian_inv = jacobian.inverse(); //what to do when matrix is non invertible?
		Eigen::MatrixXd jacobian_transpose = jacobian.transpose();
		
		
		
		
		//Find current ee pose
		current_ee_pos.head(3) = tool_wrt_robot.translation();
		current_ee_pos.tail(3) = decompose_rot_mat(tool_wrt_robot.linear()); 
		Eigen::Quaterniond flange_quat(tool_wrt_robot.linear());
		//Update and transform force sensor output
		
		
		//to find gravity compensation force - make this more reusable
		//rigid body force vector transformations
		f_g_t = tool_wrt_robot.linear().transpose() * f_g_r;
		f_comp.head(3) = f_g_t;
		f_comp.tail(3) = tool_length_vector.cross(f_g_t);
		
		//compensate for these forces
		Eigen::Vector3d force_tool_frame =  wrench_body_coords_.head(3); //- f_comp.head(3);
		Eigen::Vector3d moment_tool_frame =  wrench_body_coords_.tail(3); //- f_comp.tail(3);
				
		wrench_wrt_robot.head(3) = sensor_wrt_robot.linear() * (wrench_body_coords_.head(3)); // - f_comp.head(3);
		wrench_wrt_robot.tail(3) = sensor_wrt_robot.linear() * (wrench_body_coords_.tail(3)); // - f_comp.tail(3);
		
		

		
		//safety until I figure out how to quaternion effectively
		//virt_attr_pos.tail(3) =current_ee_pos.tail(3) ;
		
		virt_attr_pos.head(3) = initial_ee_pos.head(3) + virt_attr_delta;
		
		Eigen::Matrix3d tf_virt_attr_rot_mat =  virt_attr_rot;
		
		
		
		//effect of virtual attractor
		desired_force.head(3) = K_virt * (virt_attr_pos.head(3) - current_ee_pos.head(3));
		//Representing rotations with 3 vals loses info
		//Instead, 
		//Find delta phi_x, phi_y, and phi_z  from source and destination rotation
		//Multiply with stiffness 
		desired_force.tail(3) = K_virt * (delta_phi_from_rots(tool_wrt_robot.linear(), tf_virt_attr_rot_mat));
		
		//control law. Simple accommodation again
		Eigen::VectorXd result_twist =   accomodation_gain * (desired_force + wrench_wrt_robot);
		
		
		if(result_twist.norm() > MAX_TWIST_NORM) result_twist = (result_twist / result_twist.norm()) * MAX_TWIST_NORM;
		//result_twist<<0,0.001,0,0,0,0;

		Eigen::VectorXd des_jnt_vel = jacobian_inv * result_twist;
		
		
		//clip vel command  and remove nan that might have made their way through jacobian inverse
		//nan in jnt vel means Jacobian is losing rank - Fix 1: Stop moving - Make vels 0;
		for(int i = 0; i < 6; i++) { if(isnan(des_jnt_vel(i))) des_jnt_vel<<0,0,0,0,0,0; }
			//if at singularity - just dont move. Redundant test
			//FullPivLu decomposition always provides an inverse - I think?
			
						
		//ensure that desired joint vel is within set limits
		if(des_jnt_vel.norm() > MAX_JNT_VEL_NORM) des_jnt_vel = (des_jnt_vel / des_jnt_vel.norm()) * MAX_JNT_VEL_NORM;
				
		

		//euler one step integration to calculate position from velocities
		Eigen::MatrixXd des_jnt_pos = joint_states_ + (des_jnt_vel * dt_);
		
		
		//stuff vel and pos command into Jointstate message and publish
		for(int i = 0; i < 6; i++) desired_joint_state.position[i] = std::round(des_jnt_pos(i) * 1000) /1000; //implement low pass filter here instead
		for(int i = 0; i < 6; i++) desired_joint_state.velocity[i] = std::round(des_jnt_vel(i) * 1000) /1000;
		arm_publisher.publish(desired_joint_state);
		
		//publishing cartesian coordinates of robot end effector
		cartesian_log.pose.position.x = current_ee_pos(0);
		cartesian_log.pose.position.y = current_ee_pos(1);
		cartesian_log.pose.position.z = current_ee_pos(2);
		cartesian_log.pose.orientation.w = flange_quat.w();
		cartesian_log.pose.orientation.x = flange_quat.x();
		cartesian_log.pose.orientation.y = flange_quat.y();
		cartesian_log.pose.orientation.z = flange_quat.z();
		cartesian_log.header.stamp = ros::Time::now();
		cart_log_pub.publish(cartesian_log);

		virt_attr_log.pose.position.x = virt_attr_pos(0);
		virt_attr_log.pose.position.y = virt_attr_pos(1);
		virt_attr_log.pose.position.z = virt_attr_pos(2);
		
		
		virt_attr_log.pose.orientation.w = virt_quat.w();
		virt_attr_log.pose.orientation.x = virt_quat.x();
		virt_attr_log.pose.orientation.y = virt_quat.y();
		virt_attr_log.pose.orientation.z = virt_quat.z();
		virt_attr_log.header.stamp = ros::Time::now();
		virt_attr_after_tf.publish(virt_attr_log);
		
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