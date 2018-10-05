//This node reads egm feedback messages and publishes them onto ROS topic -
//Also, to keep the robot controller 'interested' in the node, it sends the recieved joint values back to the controller
//The above mess can be remedied by increasing \CommTimeout in the RAPID program. TODO!


#include <irb120_accomodation_control/irb120_accomodation_control.h>
#include <abb_libegm/egm_controller_interface.h>
#include <sys/socket.h>
#include <errno.h>


void ShowRobotMessage(abb::egm::EgmRobot *rob_msg) {
	if(rob_msg->has_header()); 
	//write this function
	//check for header, seq_no, timestamp, and mtype
	//if yes, print out joint angs and cart pose
	//extend it to publish 
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "libegm_interface_test");
	ros::NodeHandle nh;
	ros::Publisher joint_state_publisher = nh.advertise<sensor_msgs::JointState>("abb120_joint_state",1);
	double vals;
	sensor_msgs::JointState jointstate;
	jointstate.position.resize(6);
	char *ip_addr = "192.168.125.1";
	//char *msg = "R";
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
	const unsigned short port_number_robot = 53652;
	memset((char *)&serv_addr, sizeof(serv_addr), 0);
	memset((char *)&serv_addr, sizeof(client_addr), 0);
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_addr.sin_port = htons(port_number);
	//serv_addr.sin_addr.s_addr = inet_addr(ip_addr);
	client_addr.sin_family = AF_INET;
	client_addr.sin_addr.s_addr = inet_addr(ip_addr);
	client_addr.sin_port = htons(port_number_robot);
	abb::egm::EgmRobot *robot_message = new abb::egm::EgmRobot();
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
					for(int i = 0; i < 6; i++) {
						jointstate.position[i] = robot_message->feedback().joints().joints(i+1);
					}
					joint_state_publisher.publish(jointstate);
					ROS_INFO("need to keep you interested in me");
					/* Populate EgmSensor msg with same joint angles under 'planned'
					if(sendto(fd, buf, sizeof(buf),0,(struct sockaddr*)&client_addr,addrlen) < 0 ) {
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