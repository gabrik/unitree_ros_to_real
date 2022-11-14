#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <zenoh-pico.h>

int getch()
{
	int ch;
	struct termios oldt;
	struct termios newt;

	// Store old settings, and copy to new settings
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;

	// Make required changes and apply the settings
	newt.c_lflag &= ~(ICANON | ECHO);
	newt.c_iflag |= IGNBRK;
	newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
	newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
	newt.c_cc[VMIN] = 0;
	newt.c_cc[VTIME] = 1;
	tcsetattr(fileno(stdin), TCSANOW, &newt);

	// Get the current character
	ch = getchar();

	// Reapply old settings
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

	return ch;
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


	ros::init(argc, argv, "keyboard_input_node");

	ros::NodeHandle nh;

	ros::Rate loop_rate(500);

	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	geometry_msgs::Twist twist;

	// Zenoh declaring publisher
	printf("Declaring publisher for '%s'...\n", keyexpr);
    z_owned_publisher_t zpub = z_declare_publisher(z_session_loan(&s), z_keyexpr(keyexpr), NULL);
    if (!z_publisher_check(&zpub)) {
        printf("Unable to declare publisher for key expression!\n");
        return -1;
    }

	//

	long count = 0;

	while (ros::ok())
	{
		twist.linear.x = 0.0;
		twist.linear.y = 0.0;
		twist.linear.z = 0.0;
		twist.angular.x = 0.0;
		twist.angular.y = 0.0;
		twist.angular.z = 0.0;

		int ch = 0;

		ch = getch();

		printf("%ld\n", count++);
		printf("ch = %d\n\n", ch);

		switch (ch)
		{
		case 'q':
			printf("already quit!\n");
			return 0;

		case 'w':
			twist.linear.x = 0.5;
			printf("move forward!\n");
			break;

		case 's':
			twist.linear.x = -0.5;
			printf("move backward!\n");
			break;

		case 'a':
			twist.linear.y = 0.5;
			printf("move left!\n");
			break;

		case 'd':
			twist.linear.y = -0.5;
			printf("move right!\n");
			break;

		case 'j':
			twist.angular.z = 1.0;
			printf("turn left!\n");
			break;

		case 'l':
			twist.angular.z = -1.0;
			printf("turn right!\n");
			break;

		default:
			printf("Stop!\n");
			break;
		}

		pub.publish(twist);


		// Serialization
		uint8_t *buff = NULL;
    	uint32_t serialized_size = 0;
		serialized_size = ros::serialization::serializationLength(twist);

		boost::shared_array<uint8_t> buffer(new uint8_t[serialized_size]);

		ros::serialization::OStream stream(buffer.get(), serialized_size);
		ros::serialization::serialize(stream, twist);
		buff = stream.getData();

		// Print the hex
		// printf("Here is the message (Twist)\n");
		// for (int i = 0; i < serialized_size; i++)
		// {
		// 	printf("%02X", buff[i]);
		// }
		// printf("\n");
		//

		// Zenoh publish
		z_publisher_put(z_publisher_loan(&zpub), (const uint8_t *)buff, serialized_size, NULL);
		//

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
