//This node reads egm feedback messages and publishes them onto ROS topic -
//Also, to keep the robot controller 'interested' in the node, it sends the recieved joint values back to the controller
//The above mess can be remedied by increasing \CommTimeout in the RAPID program. TODO!


#include <irb120_accomodation_control/irb120_accomodation_control.h>
#include <abb_libegm/egm_controller_interface.h>
#include <sys/socket.h>
#include <errno.h>
#include <cmath>

int g_seq_no = 0;
vector<double> g_desired_joint_angles;
vector<double> home_vec{0,0,0,5,5,5};
int dbg;

vector<double> rad2deg_vect(vector<double> input_vect) {
	vector<double> output_vect;
	for(int i = 0; i < input_vect.size(); i++) {
		output_vect.push_back(input_vect[i] * 180.0 / M_PI);
	}
	return output_vect;
}


void JointPosCallBack(const sensor_msgs::JointState::ConstPtr& joint_state_message) {
	g_desired_joint_angles = rad2deg_vect(joint_state_message->position);
	
}



void ShowRobotMessage(abb::egm::EgmRobot *rob_msg) {
	if(rob_msg->has_header()); 
	//write this function
	//check for header, seq_no, timestamp, and mtype
	//if yes, print out joint angs and cart pose
	//extend it to publish 
}

void CreateEgmSensorMessage(abb::egm::EgmSensor* sensor, vector<double> joint_cmd) {
	abb::egm::EgmHeader *header = new abb::egm::EgmHeader();
	header->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_CORRECTION);
	//header->set_seqno(g_seq_no++);
	sensor->set_allocated_header(header);
	//header->set_tm(GetTickCount());
	abb::egm::EgmPlanned *planned = new abb::egm::EgmPlanned();
	abb::egm::EgmJoints *joints = new abb::egm::EgmJoints();
	for(int i = 0; i < 6; i++) {
		joints->add_joints(joint_cmd[i]);
	}
	planned->set_allocated_joints(joints);
	sensor->set_allocated_planned(planned);

}


int main(int argc, char** argv) {
	ros::init(argc, argv, "custom_egm_test");
	ros::NodeHandle nh;
	ros::Publisher joint_state_publisher = nh.advertise<sensor_msgs::JointState>("abb120_joint_state",1);
	ros::Subscriber joint_command_subscriber = nh.subscribe<sensor_msgs::JointState>("abb120_joint_angle_command",1,&JointPosCallBack);
	double vals;
	sensor_msgs::JointState jointstate;
	jointstate.position.resize(6);
	char *ip_addr = "192.168.125.1";
	//char *msg = "R";
	std::string send_string;
	unsigned char buf[1024];
	int recvlen;
	int fd;
	int enable = 1;
	struct sockaddr_in serv_addr;
	struct sockaddr_in client_addr;
	socklen_t addrlen = sizeof(client_addr);
	struct timeval timeout;
	timeout.tv_sec = 2;
	timeout.tv_usec = 0;
	const unsigned short port_number = 6510;
	const unsigned short port_number_robot = 59124;
	memset((char *)&serv_addr, sizeof(serv_addr), 0);
	//memset((char *)&serv_addr, sizeof(client_addr), 0);
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_addr.sin_port = htons(port_number);
	//serv_addr.sin_addr.s_addr = inet_addr(ip_addr);
	//client_addr.sin_family = AF_INET;
	//client_addr.sin_addr.s_addr = inet_addr(ip_addr);
	//client_addr.sin_port = htons(port_number_robot);
	abb::egm::EgmRobot *robot_message = new abb::egm::EgmRobot();
	abb::egm::EgmSensor *sensor_message = new abb::egm::EgmSensor();
	vector<float> zero_joint_vector(6,0);
		if((fd = socket(AF_INET,SOCK_DGRAM,0)) < 0) {
			ROS_WARN("Cannot create socket, go home");
		}
		if(bind(fd,(struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
	

			ROS_WARN("Cannot bind, go home\n");
			perror("error is : ");
		}
		//if(connect(fd,(struct sockaddr*)&serv_addr,sizeof(serv_addr)) <0) { ROS_INFO("error");}
		if(setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,(char *)&timeout,sizeof(timeout)) < 0) ROS_INFO("setsockopt failed");
		if(setsockopt(fd,SOL_SOCKET,SO_REUSEADDR, &enable,sizeof(int)) < 0) ROS_INFO("Set reuse failed");
		if(setsockopt(fd,SOL_SOCKET,SO_REUSEPORT, &enable,sizeof(int)) < 0) ROS_INFO("Set reuse failed");
		if(setsockopt(fd,SOL_SOCKET,IP_FREEBIND, &enable,sizeof(int)) < 0) ROS_INFO("Set reuse failed");




	g_desired_joint_angles = home_vec;
	while(ros::ok()) {
		ROS_INFO("Waiting for message");
		recvlen = recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *)&client_addr, &addrlen);

		if(recvlen > 0) {
			ros::spinOnce();
			ROS_INFO("recvd something!");
			//buf[recvlen] = 0;
			ROS_INFO_STREAM("recv message is of length:" <<recvlen);
			if(robot_message->ParseFromArray(buf,recvlen)) { 
				ROS_INFO("Successfully parsed string");
				if(robot_message->has_feedback()) {
				//next line is just for bs sake
				//ROS_INFO_STREAM("please work: "<<robot_message->feedback().joints().joints(2));
					for(int i = 0; i < 6; i++) jointstate.position[i] = robot_message->feedback().joints().joints(i);
					
					joint_state_publisher.publish(jointstate);
					//for(int i =0; i < 6; i++) jointstate.position[i] += 0.05;
					g_seq_no = robot_message->header().seqno();
					ROS_INFO("need to keep you interested in me");
					/* Populate EgmSensor msg with same joint angles under 'planned'*/
					ros::spinOnce();
					
					

					CreateEgmSensorMessage(sensor_message, g_desired_joint_angles);
					
					sensor_message->SerializeToString(&send_string);
					//ROS_WARN("CRASH");
					if(sendto(fd, send_string.c_str(), send_string.length(),0,(struct sockaddr*)&client_addr,sizeof(client_addr)) < 0 ) {
						ROS_WARN("Could not send");
					}
					else {
						ROS_INFO("Sent Successfully");
					}
				}
			}
			else {
				ROS_WARN("bad message recvd");
			}
		}
		else { 
			ROS_INFO("No response");
		}
	
	}
		close(fd);
}