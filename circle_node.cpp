#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

int flag1 = 1;
int flag2 = 0;
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "first_node");
    ros::NodeHandle nh;

    // set the uav's mode and enable it
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // def the publisher
    ros::Publisher pt_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::Publisher local_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // variable init
    mavros_msgs::PositionTarget raw_data;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 3.2;

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    // enable the UAV
    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if (flag1 == 1 && ros::Time::now() - last_request > ros::Duration(5.0))
        {
            ROS_INFO("TASK1");
            flag1 = 0;
            flag2 = 1;
            pose.pose.position.x = 10;
            pose.pose.position.y = 3;
            pose.pose.position.z = 3;
            last_request = ros::Time::now();
        }
        if (flag2 == 1 && ros::Time::now() - last_request > ros::Duration(5.0))
        {
            ROS_INFO("TASK2");
            flag2 = 3;
            raw_data.coordinate_frame = raw_data.FRAME_BODY_NED;
            raw_data.type_mask = 1 + 2 + 1024;
            raw_data.velocity.x = 2;
            raw_data.position.z = 3;
            raw_data.yaw_rate = M_PI / 6;
            last_request = ros::Time::now();
        }

        if (flag2 == 3)
        {
            pt_pub.publish(raw_data);
        }
        else
        {
            local_pub.publish(pose);
        }
        // init_pos_pub.publish(tkfpos);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
