//10/31/2018
//Surag Balajepalli
//Scratch pad for cwru_lib_abb (for lack of a better name)
#include <irb120_accomodation_control/irb120_accomodation_control.h>
#include <cmath>
#include <Eigen/QR>
#include <Eigen/Dense>

//global vars for subscribers
//Eigen::VectorXd wrench_spatial_coords_;
Eigen::VectorXd wrench_body_coords_ = Eigen::VectorXd::Zero(6);
Eigen::VectorXd joint_states_ = Eigen::VectorXd::Zero(6);


double dt_ = 0.01;
int dbg;
double MAX_JNT_VEL_NORM = 1;
bool is_nan;
double K_virt = 0.5;

Eigen::VectorXd desired_ee_twist(6);
Eigen::VectorXd desired_twist(6);
Eigen::VectorXd desired_ee_pos(6);
Eigen::MatrixXd accomodation_gain(6,6);
Eigen::VectorXd current_ee_pos(6);



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


Eigen::VectorXd pose_from_transform(Eigen::Affine3d transform) {
	Eigen::VectorXd pose(6);
	pose.head(3) = transform.translation();
	Eigen::Vector3d z_axis;
	z_axis<<0,0,1;
	pose.tail(3) = transform.linear().transpose() * z_axis;
	return pose;
 
}


Eigen::VectorXd transform_wrench(Eigen::VectorXd wrench_body_coords, Eigen::Affine3d flange_transform) { //from eqn 2.66 in MLS book
	Eigen::VectorXd wrench_spatial_coords;
	Eigen::Vector3d O_flange = flange_transform.translation();
	Eigen::Matrix3d O_flange_hat = vectorHat(O_flange);
	Eigen::MatrixXd wrench_transformation_matrix = Eigen::MatrixXd::Zero(6,6);
	wrench_transformation_matrix.block<3,3>(0,0) = flange_transform.linear().transpose();
	wrench_transformation_matrix.block<3,3>(3,3) = flange_transform.linear().transpose();
	wrench_transformation_matrix.block<3,3>(3,0) = - flange_transform.linear().transpose() * O_flange_hat;
	wrench_transformation_matrix = wrench_transformation_matrix.inverse();
	wrench_spatial_coords = wrench_transformation_matrix * wrench_body_coords;
	return wrench_spatial_coords;
}

Eigen::VectorXd transform_twist(Eigen::VectorXd twist_spat, Eigen::Affine3d transform) { //not being used
	Eigen::VectorXd twist_body;
	Eigen::Vector3d origin = transform.translation();
	Eigen::Matrix3d origin_hat = vectorHat(origin);
	Eigen::Matrix3d rot_mat = transform.linear();
	Eigen::MatrixXd twist_tf_matrix = Eigen::MatrixXd::Zero(6,6);
	twist_tf_matrix.block<3,3>(0,0) = rot_mat.transpose();
	twist_tf_matrix.block<3,3>(3,3) = rot_mat.transpose();
	twist_tf_matrix.block<3,3>(0,3) = - rot_mat.transpose() * origin_hat;
	twist_body = twist_tf_matrix.inverse() * twist_spat;

	return twist_body;
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



int main(int argc, char **argv) {
	ros::init(argc, argv, "control_test");
	ros::NodeHandle nh;
	ros::Subscriber joint_state_sub = nh.subscribe("abb120_joint_state",1,jointStateCallback);
	ros::Subscriber ft_sub = nh.subscribe("robotiq_ft_wrench", 1, ftSensorCallback);
	ros::Publisher arm_publisher = nh.advertise<sensor_msgs::JointState>("abb120_joint_angle_command",1);
	
	Irb120_fwd_solver irb120_fwd_solver;
	
	sensor_msgs::JointState desired_joint_state;
	desired_joint_state.position.resize(6);
	
	desired_twist<<0,0,0,0,0,0;
	accomodation_gain<<1,0,0,0,0,0,
						0,1,0,0,0,0,
						0,0,1,0,0,0,
						0,0,0,1,0,0,
						0,0,0,0,1,0,
						0,0,0,0,0,1;

	accomodation_gain *= 0.08;


	//where to? Make this whole thing a function
	desired_ee_pos<<0.6,0,0.75,-1,0,0;

	//static transform for sensor
	Eigen::Affine3d sensor_transform;
	Eigen::Matrix3d sensor_rot;
	Eigen::Vector3d sensor_origin;
	sensor_origin<<0,0,0.1; //approximately
	sensor_rot<<0,1,0,
				-1,0,0,
				0,0,-1;
	sensor_transform.linear() = sensor_rot;
	sensor_transform.translation() = sensor_origin;


	//static Tf for tool
	Eigen::Affine3d tool_to_sensor;
	Eigen::Matrix3d tool_to_sensor_rot = Eigen::Matrix3d::Identity();
	Eigen::Vector3d tool_to_sensor_trans;
	tool_to_sensor_trans<<0,0,-0.05;
	tool_to_sensor.linear() = tool_to_sensor_rot;
	tool_to_sensor.translation() = tool_to_sensor_trans;
	

	while(ros::ok()) {
		cout<<"----------------";

		ros::spinOnce();


		
		//initialize jacobians
		Eigen::MatrixXd jacobian = irb120_fwd_solver.jacobian2(joint_states_);
		Eigen::MatrixXd jacobian_inv = jacobian.inverse(); //what to do when matrix is non invertible?
		Eigen::MatrixXd jacobian_transpose = jacobian.transpose();
		//cout<<"jac inv"<<jacobian_inv<<endl;
		
		
		
		Eigen::Affine3d flange_affine = irb120_fwd_solver.fwd_kin_solve(joint_states_);

		//cout<<flange_affine.translation()<<endl;
		Eigen::Affine3d sensor_to_body = flange_affine * sensor_transform;
		Eigen::VectorXd wrench_spatial_coords = transform_wrench(wrench_body_coords_, sensor_to_body);
				
		Eigen::Affine3d tool_to_body = sensor_to_body * tool_to_sensor;

		//find current end effector pose
		current_ee_pos = pose_from_transform(tool_to_body);
		//desired_ee_pos = current_ee_pos;
		
		desired_twist = K_virt *  ( desired_ee_pos - current_ee_pos);

		cout<<"current_ee_pos:"<<endl<<current_ee_pos<<endl;
		cout<<"currrent_sensor_pos"<<endl<<sensor_to_body.translation()<<endl;
		cout<<"current flange pos"<<endl<<flange_affine.translation()<<endl;
		cin>>dbg;
		
		
		//control law
		//HERE! NEED TO CONVERT DESIRED TWIST TO FLANGE FRAME BEFORE PUTTING IN CONTROL LAW!
		//DESIRED TWIST IS CURRENTLY IN EE FRAME iE, TOOL FRAME
		//TO DO: MONDAY 12/03
		Eigen::MatrixXd des_jnt_vel = jacobian_inv * desired_twist - accomodation_gain * jacobian_transpose * wrench_spatial_coords  ;
		//Eigen::MatrixXd des_jnt_vel = jacobian_inv * (desired_twist - (accomodation_gain * wrench_spatial_coords));
		//cin>>dbg;
		

		//clip vel command  and remove nan that might have made their way through jacobian inverse
		cout<<"--------";
		for(int i = 0; i < 6; i++) { 
			if(isnan(des_jnt_vel(i))) {
				is_nan = true;
				//ROS_WARN("At SINGU");
			}

			else { is_nan = false;}
			if(des_jnt_vel.norm() > MAX_JNT_VEL_NORM) {
				des_jnt_vel = (des_jnt_vel / des_jnt_vel.norm()) * MAX_JNT_VEL_NORM;
			}
		}

		
		Eigen::MatrixXd des_jnt_pos = joint_states_ + (des_jnt_vel * dt_);
		des_jnt_pos<<0,0,0,0,0,0; //DEBUG ONLY
		//cout<<"pos desired"<<des_jnt_pos;
		cout<<"----------------";
		//stuff it into Jointstate message and publish
		for(int i = 0; i < 6; i++) desired_joint_state.position[i] = std::round(des_jnt_pos(i) * 1000) /1000; //implement low pass filter here instead
		if(!is_nan) arm_publisher.publish(desired_joint_state);
		ros::Rate naptime(100);
		ros::spinOnce();
		//subscribe to F/T sensor values
		//subscribe to joint states
		//publish to egm controller

	}
}