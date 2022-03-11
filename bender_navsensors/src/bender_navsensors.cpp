#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <std_msgs/String.h>

mavros_msgs::State state;
void get_state(const mavros_msgs::State::ConstPtr& msg)
{
	state = *msg;
}

int main (int argc, char **argv)
{

	ros::init(argc, argv, "navsensors");
	ros::NodeHandle nh;

	ros::Subscriber state_listener = nh.subscribe<mavros_msgs::State>("mavros/state", 10, get_state);

	ros::ServiceClient arming = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

	ros::Publisher state_pub = nh.advertise<mavros_msgs::State>("bender_navsensors/ns_state", 10);

	ros::Rate loop_rate(20);

	mavros_msgs::CommandBool arm;
	arm.request.value = true;

	while (ros::ok())
	{
		state_pub.publish(state);

		if (!state.armed)
		{
			arming.call(arm);
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
