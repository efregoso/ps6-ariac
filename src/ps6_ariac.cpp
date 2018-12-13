// rosservice start_competition "rosservice call /ariac/start_competition"
// Subscribe to the logical camera 2 topic "rostopic echo /ariac/logical_camera_2"
// Monitor the z value of "shipping-box"
// Start the conveyor belt "rosservice call /ariac/conveyor/control "power: 100""
// Whileloop until near zero. 
// Stop the conveyor belt "rosservice call /ariac/conveyor/control "power: 0""
// Wait for five seconds
// Start the conveyor belt. "rosservice call /ariac/conveyor/control "power: 100""
// At the very end, call droid to pick up shipping box "rosservice call /ariac/drone "shipment_type: order_0_shipment_0""

#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/DroneControl.h>

bool conveyor_started = false;
bool box_under_camera = false;

/// Start the competition by waiting for and then calling the start ROS Service.
void start_competition(ros::NodeHandle & node) {
	// Create a Service client for the correct service, i.e. '/ariac/start_competition'.
	ros::ServiceClient start_client = node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
	// If it's not already ready, wait for it to be ready.
	// Calling the Service using the client before the server is ready would fail.
	if (!start_client.exists()) {
		ROS_INFO("Waiting for the competition to be ready...");
		start_client.waitForExistence();
		ROS_INFO("Competition is now ready.");
	}
	ROS_INFO("Requesting competition start...");
	std_srvs::Trigger srv;  // Combination of the "request" and the "response".
	start_client.call(srv);  // Call the start Service.
	if (!srv.response.success) {  // If not successful, print out why.
		ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
	} else {
		ROS_INFO("Competition started!");
	}
}


/// Example class that can hold state and provide methods that handle incoming data.
class MyCompetitionClass {
	public:
		explicit MyCompetitionClass(ros::NodeHandle & node): current_score_(0), has_been_zeroed_(false) {
			joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm/command", 10);
		}

	/// Called when a new message is received.
	void current_score_callback(const std_msgs::Float32::ConstPtr & msg) {
		 if (msg->data != current_score_) {
		 	ROS_INFO_STREAM("Score: " << msg->data);
		 }
		 current_score_ = msg->data;
	}

	/// Called when a new message is received.
	void competition_state_callback(const std_msgs::String::ConstPtr & msg) {
		 if (msg->data == "done" && competition_state_ != "done") {
		 	ROS_INFO("Competition ended.");
		 }
		 competition_state_ = msg->data;
	}

	/// Called when a new Order message is received.
	void order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
		ROS_INFO_STREAM("Received order:\n" << *order_msg);
		received_orders_.push_back(*order_msg);
	}


	/// Called when a new JointState message is received.
	void joint_state_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg) {
		 ROS_INFO_STREAM_THROTTLE(10,"Joint States (throttled to 0.1 Hz):\n" << *joint_state_msg);
		 // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
		 current_joint_states_ = *joint_state_msg;
		 if (!has_been_zeroed_) {
		   has_been_zeroed_ = true;
		   ROS_INFO("Sending arm to zero joint positions...");
		   send_arm_to_zero_state();
		 }
	}


	/// Create a JointTrajectory with all positions set to zero, and command the arm.
	void send_arm_to_zero_state() {
		 // Create a message to send.
		 trajectory_msgs::JointTrajectory msg;

		 // Fill the names of the joints to be controlled.
		 // Note that the vacuum_gripper_joint is not controllable.
		 msg.joint_names.clear();
		 msg.joint_names.push_back("iiwa_joint_1");
		 msg.joint_names.push_back("iiwa_joint_2");
		 msg.joint_names.push_back("iiwa_joint_3");
		 msg.joint_names.push_back("iiwa_joint_4");
		 msg.joint_names.push_back("iiwa_joint_5");
		 msg.joint_names.push_back("iiwa_joint_6");
		 msg.joint_names.push_back("iiwa_joint_7");
		 msg.joint_names.push_back("linear_arm_actuator_joint");
		 // Create one point in the trajectory.
		 msg.points.resize(1);
		 // Resize the vector to the same length as the joint names.
		 // Values are initialized to 0.
		 msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
		 // How long to take getting to the point (floating point seconds).
		 msg.points[0].time_from_start = ros::Duration(0.001);
		 ROS_INFO_STREAM("Sending command:\n" << msg);
		 joint_trajectory_publisher_.publish(msg);
	}


	/// Called when a new LogicalCameraImage message is received.
	void logical_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {

		if (conveyor_started) {
			ROS_INFO_STREAM_THROTTLE(5, "Logical camera new message received.");
			ROS_INFO("received value is: %f", image_msg->models.size());

			if (image_msg->models.size() > 0) {

				float z_coord = image_msg->models[0].pose.position.z;
				ROS_INFO("Z-coord: %f ", z_coord);

				if (z_coord < 0.01 && z_coord > -0.01){
					box_under_camera = true;
				}
			}
			
		}
	}

	void drone_callback(const std_msgs::String::ConstPtr & image_msg) {
		
		ROS_INFO_STREAM_THROTTLE(5, "New drone message received.");
		ROS_INFO("received value is: %f", image_msg);

	}

	/// Called when a new Proximity message is received.
	void break_beam_callback(const osrf_gear::Proximity::ConstPtr & msg) {
		 if (msg->object_detected) {  // If there is an object in proximity.
		 	ROS_INFO("Break beam triggered.");
		 }
	}

	private:
		std::string competition_state_;
		double current_score_;
		ros::Publisher joint_trajectory_publisher_;
		std::vector<osrf_gear::Order> received_orders_;
		sensor_msgs::JointState current_joint_states_;
		bool has_been_zeroed_;

	public:
		osrf_gear::ConveyorBeltControl conveyor_srv_;
		ros::ServiceClient conveyor_client_;
};


void proximity_sensor_callback(const sensor_msgs::Range::ConstPtr & msg) {
	if ((msg->max_range - msg->range) > 0.01) {  // If there is an object in proximity.
		ROS_INFO_THROTTLE(1, "Proximity sensor sees something.");
	}
}

void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr & msg) {
	size_t number_of_valid_ranges = std::count_if(msg->ranges.begin(), msg->ranges.end(), std::isfinite<float>);
	if (number_of_valid_ranges > 0) {
		ROS_INFO_THROTTLE(1, "Laser profiler sees something.");
	}
}
     
     
int main(int argc, char ** argv) {
       // Last argument is the default name of the node.
       ros::init(argc, argv, "ps6_ariac_node");
     
	// Create node
       ros::NodeHandle n;

   	// Instance of custom class from above.
 	MyCompetitionClass comp_class(n);

	// Start a client call to the ConveyorBeltService
	comp_class.conveyor_client_ = n.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
	//osrf_gear::ConveyorBeltControl conveyor_srv;

	// Subscribe to the '/ariac/logical_camera_2' topic.
 	ros::Subscriber logical_camera_subscriber = n.subscribe("/ariac/logical_camera_2", 1, &MyCompetitionClass::logical_camera_callback, &comp_class);

	// Start competition
       ROS_INFO("Setup complete.");
       start_competition(n);

	// Calling the Service using the client before the server is ready would fail.
	if (!comp_class.conveyor_client_.exists()) {
		ROS_INFO("Waiting for the conveyor client to open...");
		comp_class.conveyor_client_.waitForExistence();
		ROS_INFO("Conveyor client is now ready.");
	}
	ROS_INFO("Requesting conveyor belt start...");
	
	ROS_INFO("Waiting for a bit beforehand...");
	ros::Duration(5).sleep();

	double power = 100.0;
	comp_class.conveyor_srv_.request.power = power;

	comp_class.conveyor_client_.call(comp_class.conveyor_srv_);  // Call the start Service.
	if (!comp_class.conveyor_srv_.response.success) {  // If not successful, print out why.
		ROS_ERROR_STREAM("Failed to start the conveyor belt.");
	} else {
		ROS_INFO("Conveyor belt started!");
		conveyor_started = true;
	}

	// spin until a message is received that changes the state variable
	while (!box_under_camera){
		ros::spinOnce();
	}

	// Once box is under camera, stop the Conveyor Belt
	power = 0.0;
	comp_class.conveyor_srv_.request.power = power;

	ROS_INFO("Stopping the conveyor belt...");
	comp_class.conveyor_client_.call(comp_class.conveyor_srv_);  // Call the start Service.
	if (!comp_class.conveyor_srv_.response.success) {  // If not successful, print out why.
		ROS_ERROR_STREAM("Failed to stop the conveyor belt.");
	} else {
		ROS_INFO("Conveyor belt stopped!");
	}
	

	// Wait five seconds. 
	ROS_INFO("Waiting five seconds.");
	ros::Duration(5).sleep();

	// Deposit package at end of conveyor belt.  (Detect when an order has arrived)
	ROS_INFO("Resuming conveyor belt until an order has arrived for drone.");
	power = 100.0;
	comp_class.conveyor_srv_.request.power = power;

	comp_class.conveyor_client_.call(comp_class.conveyor_srv_);  // Call the start Service.
	if (!comp_class.conveyor_srv_.response.success) {  // If not successful, print out why.
		ROS_ERROR_STREAM("Failed to stop the conveyor belt.");
	} else {
		ROS_INFO("Conveyor belt stopped!");
	}

	// Check somehow that an order has arrived.
	// Wait for a while. 
	ROS_INFO("Waiting for 15 seconds for drone can pick up shipment.");
	ros::Duration(15).sleep();

	// Call drone to pick it up 
	ROS_INFO("Sending drone to pick up shipment.");
	osrf_gear::DroneControl drone_srv;
	ros::ServiceClient drone_client = n.serviceClient<osrf_gear::DroneControl>("/ariac/drone");

	// Calling the Service using the client before the server is ready would fail.
	if (drone_client.exists()) {
		ROS_INFO("Waiting for the drone client to open...");
		drone_client.waitForExistence();
		ROS_INFO("Drone client is now ready.");
	}
	ROS_INFO("Requesting drone...");
	drone_client.call(drone_srv);  // Call the drone Service.
	if (!drone_srv.response.success) {  // If not successful, print out why.
		ROS_ERROR_STREAM("Failed to start the drone.");
	} else {
		ROS_INFO("Drone started!");
	}


	// Wait for drone
	ROS_INFO("Waiting for drone to collect shipment.");
	ros::Duration(15).sleep();

	//Print success message
	ROS_INFO("Success.");

        return 0;
}
