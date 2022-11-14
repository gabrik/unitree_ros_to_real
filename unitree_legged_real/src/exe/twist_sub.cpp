#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <geometry_msgs/Twist.h>

extern "C" {
#include <zenoh-pico.h>
}

using namespace UNITREE_LEGGED_SDK;
class Custom
{
public:
    UDP low_udp;
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};

public:
    Custom()
        :
        // low_udp(LOWLEVEL),
        low_udp(LOWLEVEL, 8091, "192.168.123.10", 8007),
        high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);
    }

    void highUdpSend()
    {
        // printf("high udp send is running\n");

        high_udp.SetSend(high_cmd);
        high_udp.Send();
    }

    void lowUdpSend()
    {

        low_udp.SetSend(low_cmd);
        low_udp.Send();
    }

    void lowUdpRecv()
    {

        low_udp.Recv();
        low_udp.GetRecv(low_state);
    }

    void highUdpRecv()
    {
        // printf("high udp recv is running\n");

        high_udp.Recv();
        high_udp.GetRecv(high_state);
    }
};

Custom custom;

ros::Subscriber sub_cmd_vel;
ros::Publisher pub_high;

long cmd_vel_count = 0;


void cmdZVelCallback(const z_sample_t *sample, void *arg)
{

    uint8_t *buff = NULL;
    uint32_t serialized_size = 0;

    boost::shared_ptr<geometry_msgs::Twist> c_msg(new geometry_msgs::Twist());
    // Deserialization
    boost::shared_array<uint8_t> de_buffer((uint8_t*) sample->payload.start);
    ros::serialization::IStream de_stream(de_buffer.get(), (uint32_t)sample->payload.len);
    ros::serialization::deserialize(de_stream, *c_msg.get());
    //


    printf("[Zenoh] cmdVelCallback is running!\t%ld\n", cmd_vel_count);


    custom.high_cmd = rosMsg2Cmd(c_msg);

    printf("[Zenoh]  cmd_x_vel = %f\n", custom.high_cmd.velocity[0]);
    printf("[Zenoh]  cmd_y_vel = %f\n", custom.high_cmd.velocity[1]);
    printf("[Zenoh]  cmd_yaw_vel = %f\n", custom.high_cmd.yawSpeed);

    unitree_legged_msgs::HighState high_state_ros;

    high_state_ros = state2rosMsg(custom.high_state);

    // Serializing
    serialized_size = ros::serialization::serializationLength(high_state_ros);

    boost::shared_array<uint8_t> buffer(new uint8_t[serialized_size]);
    ros::serialization::OStream stream(buffer.get(), serialized_size);
    ros::serialization::serialize(stream, high_state_ros);
    buff = stream.getData();
    //

    // Print the hex
    // printf("Here is the message(HighState):\n");
    // for (int i = 0; i < serialized_size; i++)
    // {
    //     printf("%02X", buff[i]);
    // }
    // printf("\n");

    // Publish here on Zenoh

    printf("[Zenoh]  cmdVelCallback ending!\t%ld\n\n", cmd_vel_count++);
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{

    uint8_t *buff = NULL;
    uint32_t serialized_size = 0;

    printf("cmdVelCallback is running!\t%ld\n", cmd_vel_count);

    custom.high_cmd = rosMsg2Cmd(msg);

    printf("cmd_x_vel = %f\n", custom.high_cmd.velocity[0]);
    printf("cmd_y_vel = %f\n", custom.high_cmd.velocity[1]);
    printf("cmd_yaw_vel = %f\n", custom.high_cmd.yawSpeed);

    unitree_legged_msgs::HighState high_state_ros;

    high_state_ros = state2rosMsg(custom.high_state);

    // Serializing
    serialized_size = ros::serialization::serializationLength(high_state_ros);

    boost::shared_array<uint8_t> buffer(new uint8_t[serialized_size]);

    ros::serialization::OStream stream(buffer.get(), serialized_size);
    ros::serialization::serialize(stream, high_state_ros);
    buff = stream.getData();
    //

    // pub_high.publish(high_state_ros);


    // Print the hex
    printf("Here is the message(HighState):\n");
    for (int i = 0; i < serialized_size; i++)
    {
        printf("%02X", buff[i]);
    }
    printf("\n");

    // Publish here on Zenoh

    printf("cmdVelCallback ending!\t%ld\n\n", cmd_vel_count++);
}

int main(int argc, char **argv)
{

    // Zenoh config
	const char *keyexpr = "ros1/cmd_vel";
	const char *mode = "client";
	const char *locator = "tcp/127.0.0.1:7447";
	///



	/// Zenoh init
	z_owned_config_t config = z_config_default();
    zp_config_insert(z_config_loan(&config), Z_CONFIG_MODE_KEY, z_string_make(mode));
    zp_config_insert(z_config_loan(&config), Z_CONFIG_PEER_KEY, z_string_make(locator));

    printf("Opening session...\n");
    z_owned_session_t s = z_open(z_config_move(&config));
    if (!z_session_check(&s)) {
        printf("Unable to open session!\n");
        return -1;
    }

    // Start read and lease tasks for zenoh-pico
    if (zp_start_read_task(z_session_loan(&s), NULL) < 0 || zp_start_lease_task(z_session_loan(&s), NULL) < 0) {
        printf("Unable to start read and lease tasks");
        return -1;
    }
	///

    ros::init(argc, argv, "twist_sub");

    ros::NodeHandle nh;

    pub_high = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);

    sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmdVelCallback);


    // Subscibe here on Zenoh
    z_owned_closure_sample_t callback = z_closure_sample(cmdZVelCallback, NULL, NULL);
    printf("Declaring Subscriber on '%s'...\n", keyexpr);
    z_owned_subscriber_t sub =
        z_declare_subscriber(z_session_loan(&s), z_keyexpr(keyexpr), z_closure_sample_move(&callback), NULL);
    if (!z_subscriber_check(&sub)) {
        printf("Unable to declare subscriber.\n");
        return -1;
    }
    //

    LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
    LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();

    ros::spin();

    return 0;
}
