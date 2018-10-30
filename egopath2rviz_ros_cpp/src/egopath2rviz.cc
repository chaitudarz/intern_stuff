/*
    EgoPath2Rviz_ROS_CPP visualization for the SimFrame

    Copyright (c) 2018 Zukunft Mobility GmbH
*/

#include "egopath2rviz.h"
#define FIXED_FRAME "map"


namespace egotraj2rviz {
    

    EgoTrajToRViz::EgoTrajToRViz() {
        pub_ = nh_.advertise<visualization_msgs::MarkerArray>("EgoTrajectory", 1);
    }

    EgoTrajToRViz::~EgoTrajToRViz() {}

    void EgoTrajToRViz::GetParams() {
        std::string tMsg;

        nh_.param(ros::this_node::getName() + "/plottingType", mode_, ::std::string("length"));

        std::transform(mode_.begin(), mode_.end(), mode_.begin(), ::tolower);   // make mode lower case
        double tMax = (::std::strcmp(mode_.c_str(), "length") == 0) ? 50.0 : 10.0;
        nh_.param(ros::this_node::getName() + "/value", max_, tMax);
        step_ = max_ / 10.0;

        if (0 == mode_.compare("length"))
            tMsg = "EgoTrajToRViz: Plot last " + std::to_string(max_) + " meters.";
        else
            tMsg = "EgoTrajToRViz: Plot last " + std::to_string(max_) + " seconds.";

        ROS_INFO(tMsg.c_str());


        std::string objMode;
        nh_.param(ros::this_node::getName() + "/objMode", objMode, std::string("master"));

        if (0 == objMode.compare("master")) {
            masterMode_ = true;
            ROS_INFO("EgoTrajToRViz: MASTER mode");
        }
        else {
            masterMode_ = false;

            //for idtoread_ input, the param is set to be in temp as --> int, bcoz getparam accepts only int,bool,double,str.
            // so it store the input to temp and next change temp to uint8_t

            int temp = -1;
            nh_.getParam(ros::this_node::getName() + "/objId", temp);
            idToRead_ = static_cast<uint8_t >(temp);
            tMsg = "EgoTrajToRViz: listen to object with Id "  + std::to_string(temp);
            ROS_INFO(tMsg.c_str());
        }
    }

    void EgoTrajToRViz::StaticDataCallback(const SimpleSim_Messages_ROS::StaticSimDataConstPtr &ssd) {

        isInitialize_ = false;
        //maybe (re-)initialized some globals

        GetParams();
        // Loop over objects

        // if master store id to arrIDx_
        for (auto it = ssd->attrs.begin(); it != ssd->attrs.end(); ++it) {
            if (masterMode_ && it->isMaster) {
                arrIdx_ = ::std::distance(ssd->attrs.begin(), it);
               // frame_id__ = "SiSiObj_" + std::to_string(it->id) + "_" + std::string(it->name.data.c_str());
                isInitialize_ = true;

                break;
            } else if (!masterMode_ && it->id == idToRead_) {
                // search for a match to idToRead_
                arrIdx_ = ::std::distance(ssd->attrs.begin(), it);
              //  frame_id__ = "SiSiObj_" + std::to_string(it->id) + "_" + std::string(it->name.data.c_str());
                isInitialize_ = true;



                break;
            }

        }
        if (!isInitialize_) {
            ROS_ERROR (
                    "EgoTrajToRViz: The given idToRead or Master is not found in Static Sim data. Update the data and re-launch the program");
        } //else
           // ROS_INFO((std::string("EgoTrajToRViz: set frame_id to: ") + frame_id__).c_str());

    }


    void EgoTrajToRViz::SimFrameCallback(const SimpleSim_Messages_ROS::SimFrameConstPtr &sf) {

        if (!isInitialize_) {
            return;
        }

        visualization_msgs::MarkerArray markerArray;
        //tf::TransformBroadcaster pos_broadcaster;

        ::IfCollection::Float3D currPos(sf->objects.at(arrIdx_).pos.x, sf->objects.at(arrIdx_).pos.y,
                                        sf->objects.at(arrIdx_).pos.theta);

        // Check if simTime is goign back


        if (sf->simTime < CurrTime_) {
            posList_.clear();
            threshold_ = 0.0f;
            deltaThreshold_ = 0.0f;
//        CurrTime_ = sf->simTime;
        }


        if (posList_.size() > 0) {
            if (strcmp(mode_.c_str(), "length") == 0) {
                float deltaX = (posList_.back().x - currPos.x);
                float deltaY = (posList_.back().y - currPos.y);

                threshold_ += ::std::sqrt(
                        deltaX * deltaX +
                        deltaY * deltaY);                 // distance to prev pt posList_.back().x, posList_.back().y ;
                deltaThreshold_ += ::std::sqrt(
                        deltaX * deltaX +
                        deltaY * deltaY);                 // distance to prev pt posList_.back().x, posList_.back().y ;

            } else if (strcmp(mode_.c_str(), "time") == 0) {
                deltaThreshold_ +=  (sf->simTime - threshold_);
                threshold_ +=  (sf->simTime - threshold_);

                ::std::cout << "val" << "\t" << max_ << "mode" << "\t" << mode_ << ::std::endl;
            }
            else {
                ROS_ERROR(("EgoTrajToRViz: unsupported mode: '" + mode_ + "'. Use [time|length]").c_str());
                // do nothing
                return;
            }

            for (size_t iter = 0; iter < posList_.size(); iter++) {
                auto pt = posList_.at(iter);

                visualization_msgs::Marker marker;

                //marker.header.frame_id = frame_id__;
                //Here, we are considered the fixed frame id as Map,
                // other possibility using the transform broadcaster and listener, map as parent id and object as child id.


                marker.header.frame_id = FIXED_FRAME;
                marker.header.stamp = ros::Time::now();

                marker.ns = "EgoTrajectory";
                marker.type = visualization_msgs::Marker::ARROW;
                marker.action = visualization_msgs::Marker::ADD;

                marker.scale.x = 0.3;
                marker.scale.y = 0.15;
                marker.scale.z = 0.10;

                marker.color.r = 0.75f + ((0.5f * iter) / posList_.size());
                marker.color.g = 0.75f;
                marker.color.b = 0.0;
                marker.color.a = 1.0f;

                marker.id = static_cast<int>(iter);
                marker.pose.position.x = pt.x;
                marker.pose.position.y = pt.y;
                marker.pose.position.z = 0.25; // pose of the marker from the base.

                //tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, sf->objects.at(arrIdx_).pos.theta);
                tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, pt.z);   // pt.z value is from the float3D
                q.normalize();

                marker.pose.orientation.x = q.getX();
                marker.pose.orientation.y = q.getY();
                marker.pose.orientation.z = q.getZ();
                marker.pose.orientation.w = q.getW();

                marker.lifetime = ros::Duration();


                markerArray.markers.push_back(marker);
            }
        }


        if (posList_.empty()) {
            posList_.push_back(currPos);
            if (strcmp(mode_.c_str(), "time") == 0) {
                threshold_ = sf->simTime;
            }
        }
        if ((deltaThreshold_ >= step_))  // step_ size, should be set the threshold values with the parameter
        {
            deltaThreshold_ = 0.0;
            posList_.push_back(currPos);

            if (threshold_ >= max_) // set threshold values with the parameter
            {
                posList_.erase(posList_.begin());
            }

        }
        CurrTime_ = sf->simTime;

        pub_.publish(markerArray);
    }

} // end of namespace


