#ifndef TRAJ2RVIZ_H
#define TRAJ2RVIZ_H

#include "ros/ros.h"
#include "SimpleSim_Messages_ROS/SimFrame.h"
#include "SimpleSim_Messages_ROS/StaticSimData.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <tf/transform_broadcaster.h>

#include <vector>
#include <string>
#include "IfCollection_CPP/interface_types.h"
#include "IfCollection_CPP/DynamicObjectList.h"
#include "boost/bind.hpp"

namespace egotraj2rviz {

    class EgoTrajToRViz {





        // public methods
    public:

        EgoTrajToRViz();
        ~EgoTrajToRViz();
        void StaticDataCallback(const SimpleSim_Messages_ROS::StaticSimDataConstPtr &ssd);
        void SimFrameCallback(const SimpleSim_Messages_ROS::SimFrameConstPtr &sf);

        // private methods
    private:

        void GetParams();


        // public members
    public:

        ros::NodeHandle nh_{"~"};
        ros::Publisher pub_;
        ::std::vector<::IfCollection::Float3D> posList_;

        // private members
    private:

        double threshold_ = 0.0f;                               // threshold --> total distance summed up-to 50m for default .
        double deltaThreshold_ = 0.0f;                              // deltaThreshold_ --> for the step_ size incremental for the markers
        double max_ = 50.0;
        double step_;

        // max_ / 50.0;

        ::std::string mode_ = "length";                              //  default is length based or we can define in param to choose length or time
        ::std::string frame_id__;
        bool isInitialize_ = false;                          // in coding lang the basic initialize is always zero, it mean false.
        bool masterMode_ = false;
        int arrIdx_ = 99;
        ::std::uint8_t idToRead_ = 0;
        double CurrTime_ = -900000.0f;

    };

} // end of namespace

#endif