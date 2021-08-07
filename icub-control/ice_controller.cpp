/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <stdio.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/rosmsg/std_msgs/Float64.h>
#include <yarp/rosmsg/std_msgs/String.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using yarp::os::Network;
using yarp::os::Node;
using yarp::os::Publisher;
using yarp::os::Subscriber;
using yarp::rosmsg::std_msgs::Float64;
using yarp::rosmsg::std_msgs::String;

namespace
{
    YARP_LOG_COMPONENT(CONTROLLER, "icub.controller");
    constexpr double loop_delay = 0.1;
}

Property getPolyDriverOptions()
{
    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", "/motor/client");
    options.put("remote", "/icubSim/right_arm");

    return options;
}

void pinch(IPositionControl *pos, bool rest = false)
{
    int thumbOpposeJoint = 8;
    int thumbDistalJoint = 10;
    int indexProximalJoint = 11;
    int indexDistalJoint = 12;

    if (rest)
    {
        pos->positionMove(thumbOpposeJoint, 0);
        pos->positionMove(thumbDistalJoint, 0);
        pos->positionMove(indexProximalJoint, 0);
        pos->positionMove(indexDistalJoint, 0);
        yarp::os::Time::delay(1);
    }

    else
    {
        pos->positionMove(thumbOpposeJoint, 90);
        pos->positionMove(thumbDistalJoint, 30);
        pos->positionMove(indexProximalJoint, 45);
        pos->positionMove(indexDistalJoint, 90);
        yarp::os::Time::delay(1);
    }
}

void wristFlexion(IPositionControl *pos, bool rest = false)
{
    int wristPitchJoint = 5;

    if (rest)
    {
        pos->positionMove(wristPitchJoint, 0);
    }
    else
    {
        pos->positionMove(wristPitchJoint, -90);
        yarp::os::Time::delay(0.5);
        pos->positionMove(wristPitchJoint, 90);
        yarp::os::Time::delay(0.5);
    }
}

int main(int argc, char *argv[])
{
    YARP_UNUSED(argc);
    YARP_UNUSED(argv);

    Network yarp;
    const int PERIOD = 2;
    int counter = 0;

    // Control setup
    BufferedPort<Bottle> targetPort;
    targetPort.open("/task7/target/in");

    Property options = getPolyDriverOptions();
    PolyDriver rightArm(options);

    if (!rightArm.isValid())
    {
        printf("Unable to connect to robot right arm\n"); //if the right arm is not available/connected, then print this phrase
        return 1;
    }

    IPositionControl *pos; // Control board device for position control
    IVelocityControl *vel; // Control board device for velocity control
    IEncoders *enc;
    IControlMode *con; // Setting control mode in control board
    rightArm.view(pos);
    rightArm.view(vel);
    rightArm.view(enc);
    rightArm.view(con);

    if (pos == NULL || vel == NULL || enc == NULL || con == NULL)
    {
        printf("Cannot get interfaces to robot right arm\n");
        rightArm.close();
        return 1;
    }

    int i;
    int num_joints = 0;
    pos->getAxes(&num_joints); // Get the number of joints of the right arm

    // Set all the motors control mode to position control.
    for (i = 0; i <= num_joints; i++)
    {
        con->setControlMode(i, VOCAB_CM_POSITION);
    }


    Bottle *flagMessage;

    /** Creates a ROS node (Used to tag YARP ports with mandatory metadata for ROS compatibility) **/
    Node node("/yarp/icub");

    /* publisher for icub sensory data */
    Publisher<Float64> publisher;
    if (!publisher.topic("/icub_sensory_data"))
    {
        yCError(CONTROLLER) << "Failed to create publisher to /chatter";
        return -1;
    }

    /* Subscribe to RAT Model */
    Subscriber<Float64> subscriber;
    if (!subscriber.topic("/rat_control_commands"))
    {
        yCError(CONTROLLER) << "Failed to subscriber to /chatter";
        return -1;
    }

    // Control loop, sensory data stubbed
    while (true)
    {
        Float64 adata;

        if (counter % PERIOD == 0)
        {
            adata.data = 1;
        }
        else
        {
            adata.data = 0;
        }

        publisher.write(adata);

        Float64 bdata;
        subscriber.read(bdata);
        yCInfo(CONTROLLER) << "Received:" << bdata.data;
        
        float valueReceived = bdata.data;
        printf("received number: %f", valueReceived);
        printf("\n");

        bool shouldRest = valueReceived < 240;
        pinch(pos, shouldRest);
        counter++;

        /* wait some time to avoid flooding with messages */
        yarp::os::Time::delay(loop_delay);
    }

    return 0;
}

// find_package(YARP COMPONENTS os rosmsg REQUIRED)
// add_executable(ice_controller)
// target_sources(ice_controller PRIVATE ice_controller.cpp)
// target_link_libraries(
// 	ice_controller 
// 	PRIVATE 
// 		YARP::YARP_os 
// 		YARP::YARP_init 
// 		YARP::YARP_rosmsg
// )