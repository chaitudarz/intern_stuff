#ifdef CRATE_BUILD_ROS

#include "ros/ros.h"
#include "egopath2rviz.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv,"EgoTrajectory2RViz");
    ros::NodeHandle n;
    egotraj2rviz::EgoTrajToRViz et;

    boost::function<void(const SimpleSim_Messages_ROS::StaticSimDataConstPtr&
    ssd)> ssdCallback =
            [&et](const auto& ssd) {
                et.StaticDataCallback(ssd);
            };

    boost::function<void(const SimpleSim_Messages_ROS::SimFrameConstPtr&
    ssd)> sfCallback =
            [&et](const auto& sf) {
                et.SimFrameCallback(sf);
            };



    ros::Subscriber subSd = n.subscribe("/simplesim/static",16, ssdCallback);
    ros::Subscriber subEg = n.subscribe("/simplesim/simdata", 8, sfCallback);

    et.posList_ = ::std::vector<::IfCollection::Float3D>();

    ros::spin();

    return 0;
}

#endif  // CRATE_BUILD_ROS