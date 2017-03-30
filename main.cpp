#include <yarp/os/all.h>
#include <ocra/control/Trajectory/TimeOptimalTrajectory.h>
#include <Eigen/Dense>
#include <iostream>

ocra::TimeOptimalTrajectory generateTrajectory(const yarp::os::ResourceFinder& rf)
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    if (rf.check("x")) {
        x = rf.find("x").asDouble();
    }

    if (rf.check("y")) {
        y = rf.find("y").asDouble();
    }

    if (rf.check("z")) {
        z = rf.find("z").asDouble();
    }

    Eigen::Vector3d startWaypoint(0,0,0);
    Eigen::Vector3d middleWaypoint(x,y,z);
    Eigen::Vector3d endWaypoint(2,-2,0);

    std::cout << "Generating CoM trajectory with the following waypoints:" << std::endl;
    std::cout << "Start: " << startWaypoint.transpose() << std::endl;
    std::cout << "Middle: " << middleWaypoint.transpose() << std::endl;
    std::cout << "End: " << endWaypoint.transpose() << std::endl;

    std::list<Eigen::VectorXd> waypoints;
    waypoints.push_back(startWaypoint);
    waypoints.push_back(middleWaypoint);
    waypoints.push_back(endWaypoint);

    ocra::TimeOptimalTrajectory trajectory;
    trajectory.setWaypoints(waypoints);
    return trajectory;
}


int main(int argc, char *argv[]) {
    yarp::os::ResourceFinder rf;
    if (!rf.configure(argc, argv)) {
        std::cout << "[ERROR] Failed to configure ResourceFinder." << std::endl;
        return -1;
    }

    ocra::TimeOptimalTrajectory trajectory = generateTrajectory(rf);

    yarp::os::Network yarp;

    std::string timeOutPortName("/time:o");
    if(rf.check("timePort")) {
        timeOutPortName = rf.find("timePort").asString();
    }

    std::string comInPortName("/com:i");
    if(rf.check("comPort")) {
        comInPortName = rf.find("comPort").asString();
    }


    std::string timePortName("/com-traj-gen/time:i");
    yarp::os::Port timePort;
    timePort.open(timePortName);

    std::string comRefPortName("/com-traj-gen/comRef:o");
    yarp::os::Port comRefPort;
    comRefPort.open(comRefPortName);

    bool timePortConnected = false;
    bool comPortConnected = false;

    std::cout << "Waiting for ports to be connected..." << std::endl;
    while (!timePortConnected || !comPortConnected) {
        if (!timePortConnected) {
            timePortConnected = yarp.connect(timeOutPortName, timePortName);
            if(timePortConnected) {
                std::cout << "Time port connected." << std::endl;
            }
        }

        if (!comPortConnected) {
            comPortConnected = yarp.connect(comRefPortName, comInPortName);
            if(comPortConnected) {
                std::cout << "CoM port connected." << std::endl;
            }
        }

        yarp::os::Time::delay(0.01);
    }
    std::cout << "Ports connected trajectory will commence." << std::endl;

    int comRefDim = 9;
    Eigen::VectorXd desiredComTrajectoryValues(comRefDim);
    yarp::os::Bottle timeBottle, comBottle;
    bool isFirstTimeRead = true;
    double startTime;
    double relativeTime = 0.0;
    double trajDuration = trajectory.getDuration();
    while (relativeTime<=trajDuration) {

        timePort.read(timeBottle);
        if (isFirstTimeRead) {
            startTime = timeBottle.get(0).asDouble();
            isFirstTimeRead = false;
        }
        relativeTime = timeBottle.get(0).asDouble() - startTime;

        std::cout << "relativeTime: " << relativeTime << std::endl;

        desiredComTrajectoryValues << Eigen::Map<Eigen::VectorXd>(trajectory.getDesiredValues(relativeTime).data(), comRefDim);
        for (int i=0; i<comRefDim; ++i)
        {
            comBottle.addDouble(desiredComTrajectoryValues(i));
        }
        comRefPort.write(comBottle);

        timeBottle.clear();
        comBottle.clear();
    }
    std::cout << "Finished." << std::endl;
    timePort.close();
    comRefPort.close();
    return 0;
}
