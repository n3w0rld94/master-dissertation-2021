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
    YARP_LOG_COMPONENT(CONTROLLER, "icub.controller");
    constexpr double loop_delay = 0.1;
}
 
int main(int argc, char* argv[])
{
    YARP_UNUSED(argc);
    YARP_UNUSED(argv);
 
    Network yarp;
    const int PERIOD = 2;
    int counter = 0;
 
    /** Creates a ROS node (Used to tag YARP ports with 
     *   mandatory metadata for ROS compatibility) 
    **/
    Node node("/yarp/icub");
 
    /* subscribe to topic chatter */
    Publisher<Float64> publisher;
    if (!publisher.topic("/icub_sensory_data")) {
        yCError(CONTROLLER) << "Failed to create publisher to /chatter";
        return -1;
    }

 
    /* subscribe to topic chatter */
    Subscriber<Float64> subscriber;
    if (!subscriber.topic("/rat_control_commands")) {
        yCError(CONTROLLER) << "Failed to subscriber to /chatter";
        return -1;
    }
 
    while (true) {
        Float64 adata;

        if (counter % PERIOD == 0){
            adata.data = 1;
        } else {
            adata.data = 0;
        }

        publisher.write(adata);

        Float64 bdata;
        subscriber.read(bdata);
        yCInfo(CONTROLLER) << "Received:" << bdata.data;

        counter++;
 
        /* wait some time to avoid flooding with messages */
        yarp::os::Time::delay(loop_delay);
    }
 
    return 0;
}