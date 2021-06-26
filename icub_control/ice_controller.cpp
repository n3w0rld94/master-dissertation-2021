/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */
 
#include <yarp/os/all.h>
#include <yarp/rosmsg/std_msgs/Float64.h>
#include <yarp/rosmsg/std_msgs/String.h>
 
using yarp::os::Network;
using yarp::os::Node;
using yarp::os::Publisher;
using yarp::os::Subscriber;
using yarp::rosmsg::std_msgs::Float64;
using yarp::rosmsg::std_msgs::String;
 
namespace {
    YARP_LOG_COMPONENT(TALKER, "yarp.example.ros.talker");
    YARP_LOG_COMPONENT(LISTENER, "yarp.example.ros.listener");
    constexpr double loop_delay = 0.1;
}
 
int main(int argc, char* argv[])
{
    YARP_UNUSED(argc);
    YARP_UNUSED(argv);
 
    Network yarp;
 
    /* creates a node called /yarp/talker */
    Node node("/yarp/talker");
    // Node bnode("/yarp/listener");
 
    /* subscribe to topic chatter */
    Publisher<String> publisher;
    if (!publisher.topic("/icub_sensory_data")) {
        yCError(TALKER) << "Failed to create publisher to /chatter";
        return -1;
    }

 
    /* subscribe to topic chatter */
    Subscriber<Float64> subscriber;
    if (!subscriber.topic("/rat_control_commands")) {
        yCError(LISTENER) << "Failed to subscriber to /chatter";
        return -1;
    }
 
    while (true) {
        /* prepare some data */
        String adata;
        adata.data = "Hello from YARP";
        publisher.write(adata);

        Float64 bdata;
        subscriber.read(bdata);
        yCInfo(LISTENER) << "Received:" << bdata.data;
 
        /* wait some time to avoid flooding with messages */
        yarp::os::Time::delay(loop_delay);
    }
 
    return 0;
}