/* A simple server in the internet domain using TCP
   The port number is passed as an argument */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/stream.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>

namespace io = boost::iostreams;
using namespace std;

ros::Publisher pub;


void handle_client(int clientfd) {


    io::stream_buffer<io::file_descriptor_source> fpstream (clientfd, boost::iostreams::close_handle);
    std::istream in (&fpstream);

    ROS_INFO("Speech client connected");

    std::string line;
    while (in)
    {
        std::getline (in, line);

        if (!in) break;

        std_msgs::String msg;
        msg.data = line;
        pub.publish(msg);
    }

    ROS_INFO("Speech client disconnected");
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "speech_proxy");
    ros::NodeHandle nh, nhp("~");
    pub = nh.advertise<std_msgs::String> ("command", 1);

    int serverfd, portno, clientfd;
    socklen_t clilen;
    struct sockaddr_in serv_addr, cli_addr;
    int n;

    nhp.param<int>("listen_port", portno, 5000);

    serverfd = socket(AF_INET, SOCK_STREAM, 0);

    if (serverfd < 0) {
        ROS_ERROR("Error when opening socket");
        return -1;
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);

    if (bind(serverfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        ROS_ERROR("Error on binding to port"); 
        return -2;
    }

    ROS_INFO("Server ready on port %d", portno);

    while(1) {

        listen(serverfd, 5);
        clilen = sizeof(cli_addr);
        clientfd = accept(serverfd, (struct sockaddr *) &cli_addr, &clilen);

        if (clientfd < 0) 
            break;

        (new boost::thread(&handle_client, clientfd));

        ros::spinOnce();
    }

    close(serverfd);

    return 0; 
}
