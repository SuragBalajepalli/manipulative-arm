//10/31/2018
// Simple node to attempt accommodation control on arm. Compiles, Logic seems sound, need to test
#include <irb120_accomodation_control/irb120_accomodation_control.h>


//global vars
//Eigen::VectorXd wrench_spatial_coords_;
Eigen::VectorXd wrench_body_coords_ = Eigen::VectorXd::Zero(6);
Eigen::VectorXd joint_states_ = Eigen::VectorXd::Zero(6);
double dt_ = 0.01;


Eigen::VectorXd desired_twist;

Eigen::MatrixXd accomodation_gain;


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


Eigen::VectorXd transform_wrench(Eigen::VectorXd wrench_body_coords, Eigen::Affine3d flange_transform) {
	Eigen::VectorXd wrench_spatial_coords;
	Eigen::Vector3d O_flange = flange_transform.translation();
	Eigen::Matrix3d O_flange_hat = vectorHat(O_flange);
	Eigen::MatrixXd wrench_transformation_matrix = Eigen::MatrixXd::Zero(6,6);
	wrench_transformation_matrix.block<3,3>(0,0) = flange_transform.linear();
	wrench_transformation_matrix.block<3,3>(3,3) = flange_transform.linear();
	wrench_transformation_matrix.block<3,3>(0,3) = O_flange_hat * flange_transform.linear();
	wrench_transformation_matrix = wrench_transformation_matrix.inverse();
	wrench_spatial_coords = wrench_transformation_matrix * wrench_body_coords;
	return wrench_spatial_coords;
}

void ftSensorCallback(const geometry_msgs::WrenchStamped& ft_sensor) {
	//subscribe to "robotiq_ft_wrench"
	wrench_body_coords_(0) = ft_sensor.wrench.force.x;
	wrench_body_coords_(1) = ft_sensor.wrench.force.y;
	wrench_body_coords_(2) = ft_sensor.wrench.force.z;
	wrench_body_coords_(3) = ft_sensor.wrench.torque.x;
	wrench_body_coords_(4) = ft_sensor.wrench.torque.y;
	wrench_body_coords_(5) = ft_sensor.wrench.torque.z;
}

void jointStateCallback(const sensor_msgs::JointState& joint_state) {
	//subscribe to "abb120_joint_state"
	for(int i = 0; i < 6; i++) joint_states_(i) = joint_state.position[i];

}


int main(int argc, char **argv) {
	ros::init(argc, argv, "control_test");
	ros::NodeHandle nh;
	Irb120_fwd_solver irb120_fwd_solver;
	ros::Subscriber joint_state_sub = nh.subscribe("abb120_joint_state",1,jointStateCallback);
	ros::Subscriber ft_sub = nh.subscribe("robotiq_ft_wrench", 1, ftSensorCallback);
	ros::Publisher arm_publisher = nh.advertise<sensor_msgs::JointState>("abb120_joint_angle_command",1);
	desired_twist<<0,0,0,0,0,0;
	accomodation_gain<<0,0,0,0,0,0,
					0,0,0,0,0,0,
					0,0,0,0,0,0,
					0,0,0,0,0,0,
					0,0,0,0,0,0,
					0,0,0,0,0,0;
	while(ros::ok()) {
		ros::spinOnce();

		//initialize jacobians
		Eigen::MatrixXd jacobian = irb120_fwd_solver.jacobian2(joint_states_);
		Eigen::MatrixXd jacobian_inv = jacobian.inverse();
		Eigen::MatrixXd jacobian_transpose = jacobian.transpose();

		//transform wrench to spatial coords
		Eigen::Affine3d flange_affine = irb120_fwd_solver.fwd_kin_solve(joint_states_);
		Eigen::MatrixXd wrench_spatial_coords = transform_wrench(wrench_body_coords_, flange_affine);
		
		//control law
		Eigen::MatrixXd pos_desired = joint_states_ + ((jacobian_inv * desired_twist - accomodation_gain * (jacobian_transpose * wrench_spatial_coords) ) * dt_);

		//stuff it into Jointstate message and publish
		sensor_msgs::JointState desired_joint_state;
		desired_joint_state.position.resize(6);
		for(int i = 0; i < 6; i++) desired_joint_state.position[i] = pos_desired(i);
		arm_publisher.publish(desired_joint_state);
		ros::Rate naptime(100);
		//subscribe to F/T sensor values
		//subscribe to joint states
		//publish to egm controller

	}
}