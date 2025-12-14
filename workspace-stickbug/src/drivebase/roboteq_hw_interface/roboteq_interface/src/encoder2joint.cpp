#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include <hw_interface_plugin_roboteq/RoboteqData.h>

#define MAX_ENCODER_COUNTS 0x7FFFFFFF
#define PI M_PI

/*################################################ OVERVIEW ###########################################
 reads in the encoder counts and feedback from the ROBOTEQ, and publishes a joint state message with both
 position and velocity  
 

*/
//################################################ GLOBAL VARIABLES ###################################

// Declare sensor data variables
int32_t fl_encoder_counts = 0;        // front left encoder counts
int32_t fl_encoder_counts_prev = 0;   // front left previous encoder counts
int32_t fl_encoder_feedback = 0;      // front left motor velocity [rpm]
int32_t fl_delta_counts = 0;          // front left change in enoder counts 
int32_t b_encoder_counts = 0;         // back encoder counts
int32_t b_encoder_counts_prev = 0;    // back previous encoder counts
int32_t b_encoder_feedback = 0;       // back motor velocity [rpm] 
int32_t b_delta_counts = 0;           // back change in encoder counts
int32_t fr_encoder_counts = 0;        // front right encoder counts
int32_t fr_encoder_counts_prev = 0;   // front right previous encoder counts
int32_t fr_encoder_feedback = 0;      // front right motor velocity [rpm]
int32_t fr_delta_counts = 0;          // front right change in encoder counts


double encoder_counts_per_revolution; // total encoder counts per revolution from encoder spec [count/rev]
double gear_ratio;                    // gear ratio of the motor from motor spec 
double output_count_per_rev;          // output shaft counts per revolution [count/rev]

bool first_callback = true;           // flag to ignore first callback so dt is not zero

//########################################## FUNCTIONS ##################################################

// change in encoder counts, accounting for overlap----------------------------------------------
int32_t deltaCounts(int32_t counts, int32_t counts_prev)
{  
	int64_t temp_counts = (int64_t)counts;
	int64_t temp_counts_prev = (int64_t)counts_prev;
	int64_t temp_delta_counts = temp_counts - temp_counts_prev;
	if(temp_delta_counts > MAX_ENCODER_COUNTS) // Wrap around going backwards
	{
		temp_counts += 2L*MAX_ENCODER_COUNTS;
		temp_delta_counts = temp_counts - temp_counts_prev;
	}
	else if(temp_delta_counts < -MAX_ENCODER_COUNTS) // Wrap around going forwards
	{
		temp_counts -= 2L*MAX_ENCODER_COUNTS;
		temp_delta_counts = temp_counts - temp_counts_prev;
	}
	return (int32_t)temp_delta_counts;
}

// roboteq data subscriber to to get encoder counts-------------------------------------------------
void roboteq_Callback(const hw_interface_plugin_roboteq::RoboteqData::ConstPtr& msg)
{
	ROS_INFO("made it to roboteq callback");
	fl_encoder_counts_prev = fl_encoder_counts;
	fl_encoder_counts = msg->encoder_counter_absolute.at(2);
	fl_encoder_feedback = msg->feedback.at(2);

    fr_encoder_counts_prev = fr_encoder_counts;
	fr_encoder_counts = msg->encoder_counter_absolute.at(1);
	fr_encoder_feedback = msg->feedback.at(1);
    
    b_encoder_counts_prev = b_encoder_counts;
	b_encoder_counts = msg->encoder_counter_absolute.at(0);
	b_encoder_feedback = msg->feedback.at(0);

	if(first_callback)
	{
		fl_delta_counts = 0;
        fr_delta_counts = 0;
        b_delta_counts = 0;
		first_callback = false;
	}
	else
	{
		fl_delta_counts = deltaCounts(fl_encoder_counts, fl_encoder_counts_prev);
        fr_delta_counts = deltaCounts(fr_encoder_counts, fr_encoder_counts_prev);
        b_delta_counts = deltaCounts(b_encoder_counts, b_encoder_counts_prev);
	}
}

//############################################## MAIN FUNCTION #############################################
int main(int argc, char** argv)
{
	// initalize ros node
    ros::init(argc, argv, "stickbug_joint_pub_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

	// read in parameters -------------------------------------------------------------------------------
	std::string roboteq_topic_name;
	std::string joint_state_out_topic_name;
	std::string ns = ros::this_node::getNamespace();

    if(ros::param::get(ns + "/encoder_counts_per_revolution",encoder_counts_per_revolution)==false)
	{
		ROS_FATAL("No parameter %s/encoder_counts_per_revolution specified",ns.c_str());
		ros::shutdown();
		exit(1);
	}
    if(ros::param::get(ns + "/gear_ratio",gear_ratio)==false)
	{
		ROS_FATAL("No parameter 'gear_ratio' specified");
		ros::shutdown();
		exit(1);
	}
    if(ros::param::get(ns + "/roboteq_topic_name",roboteq_topic_name)==false)
	{
		ROS_FATAL("No parameter 'roboteq_topic_name' specified");
		ros::shutdown();
		exit(1);
	}
    if(ros::param::get(ns + "/joint_state_out_topic_name",joint_state_out_topic_name)==false)
	{
		ROS_FATAL("No parameter 'joint_state_out_topic_name' specified");
		ros::shutdown();
		exit(1);
	}
	ROS_INFO("%s",roboteq_topic_name.c_str());
	//----------------------------------------------------------------------------------------------

	// publishers and subsribers 
    ros::Subscriber roboteq_sub = n.subscribe(roboteq_topic_name, 1, roboteq_Callback);
    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>(joint_state_out_topic_name, 1); 
    
	// initialize joint state message 
    sensor_msgs::JointState joint_state_msg;
	joint_state_msg.name.resize(3);
	joint_state_msg.name.at(0) = "flw";
	joint_state_msg.name.at(1) = "frw";
	joint_state_msg.name.at(2) = "bw";
	joint_state_msg.position.resize(3,0.0);
	joint_state_msg.velocity.resize(3,0.0);
	joint_state_msg.effort.resize(3,0.0);

    output_count_per_rev = encoder_counts_per_revolution * 4.0 * gear_ratio;

	//-------------------------------- MAIN LOOP ---------------------------------------
    while(ros::ok())
	{
		// skip first callback so dt is not zero
        if(first_callback){
            ROS_INFO("skipping first callback");
			ros::spinOnce();
			loop_rate.sleep();
            continue;
        }
		
		// fill joint state message
		joint_state_msg.header.stamp = ros::Time::now();

		joint_state_msg.position.at(0) = fl_encoder_counts/output_count_per_rev*2.0*PI;
		joint_state_msg.position.at(1) = fr_encoder_counts/output_count_per_rev*2.0*PI;
		joint_state_msg.position.at(2) = b_encoder_counts/output_count_per_rev*2.0*PI;

		joint_state_msg.velocity.at(0) = ((double)fl_encoder_feedback)*(1.0/gear_ratio)*(2.0*PI)*(1.0/60.0);
		joint_state_msg.velocity.at(1) = ((double)fr_encoder_feedback)*(1.0/gear_ratio)*(2.0*PI)*(1.0/60);
		joint_state_msg.velocity.at(2) = ((double)b_encoder_feedback)*(1.0/gear_ratio)*(2.0*PI)*(1.0/60.0);
		
		// publish message 
		joint_state_pub.publish(joint_state_msg);

        // Zero out deltas in case sensor callbacks do not occur before the next loop
		// Don't want to keep adding old data, only want the newest data
		fl_delta_counts = 0;
		b_delta_counts = 0;
		fr_delta_counts = 0;

		// ros continue
        ros::spinOnce();
		loop_rate.sleep();

    }
    return 0;

}