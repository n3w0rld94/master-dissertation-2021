#include <string>
#include <cstdio>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

using namespace std;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

int main()
{
    Network::init();

    Property options;
    options.put("robot", "icub"); // typically from the command line.
    options.put("device", "remote_controlboard");
    options.put("local", "/test/client");        //local port names
    options.put("remote", "/icubSim/right_arm"); //where we connect to

    PolyDriver robotDevice(options);
    if (!robotDevice.isValid())
    {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 1;
    }

    IPositionControl *pos;
    IVelocityControl *vel;
    IPidControl *pid;
    IAmplifierControl *amp;
    IEncoders *enc;

    robotDevice.view(pos);
    robotDevice.view(vel);
    robotDevice.view(enc);
    robotDevice.view(pid);
    robotDevice.view(amp);

    if (pos == 0)
    {
        printf("Error getting IPositionControl interface.\n");
        return 1;
    }

    if (vel == 0)
    {
        printf("Error getting IVelocityControl interface.\n");
        return 1;
    }

    if (enc == 0)
    {
        printf("Error getting IEncoders interface.\n");
        return 1;
    }

    int joints = 0;
    pos->getAxes(&joints);

    Vector temporary;
    Vector encoders;
    Vector command_position;
    Vector command_velocity;

    temporary.resize(joints);
    encoders.resize(joints);
    command_position.resize(joints);
    command_velocity.resize(joints);

    /* we need to set reference accelerations used to generate the velocity */
    /* profile, here 50 degrees/sec^2 */
    for (int i = 0; i < joints; i++)
    {
        temporary[i] = 50.0;
    }

    pos->setRefAccelerations(temporary.data());
    // enable the amplifier and the pid controller on each joint
    for (int i = 0; i < joints; i++)
    {
        amp->enableAmp(i);
        pid->enablePid(VOCAB_PIDTYPE_POSITION, i);
    }

    double *q = new double[6];

    bool availableEncoders = enc->getEncoders(q);

    if (!availableEncoders)
    {
        fprintf(stderr, "Error reading encoders, check connectivity with the robot\n");
    }
    else
    {
        /* use encoders */
    }

    // position control

    for (int i = 0; i < joints; i++)
    {
        temporary[i] = 40.0;
    }

    pos->setRefSpeeds(temporary.data());

    bool success = pos->positionMove(command_position.data());

    // Velocity control
    success = vel->velocityMove(command_velocity.data());

    // CLOSE THE DEVICE

    robotDevice.close();
}
