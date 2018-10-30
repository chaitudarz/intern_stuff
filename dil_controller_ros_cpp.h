
#ifndef _DIL_H_
#define _DIL_H_

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "SimpleSim_Messages_ROS/DriverControl.h"
#include "vector"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "SimpleSim_Messages_ROS/SimFrame.h"


        //Time variables
        double CurrTime_ = 0.0f;
        double Begin_ = 0.0f;
        double DeltaTime_ = 0.0f;

        //Drag Force
        const double    DragCoeff_ = 0.24;                                    // Drag Coeff = 0.24 for Audi A6, this can be set as parameter
        const double    Airdensity_ = 1.28;                                    //  Air density = 1.2 kg/m3,
        const double    AreaOfCar_ = 2.6005;                                  // Reference area of Audi A6,
        double          DragForce_ = 0;                                      // Drag force Fd = 0.5*Ad*Ra*Cd*V2(velocity2), velocity is from the vehicle.

        //Force = m * a
        double          Force_ = 0;
        const double    Mass_ = 1480;                         // Audi A6 weight = 1480kg
        double          Current_Velocity_;
        double          Previous_Vel_ ;
        double          Displ_;
        double          Speed_;
        double scaled_acceleration;
        double scaled_brake;
        double actual_movement_to_vehicle;

        //Roll Resistance to tire rolling coefficient
        double          ForceRoll_ = 0;                                   //  = Roll resistance coeff (RRC) * m * g ==> 0.15*m*9.81m/s2
        const double    gravity_due_acc = 9.8;
        const double    RRC = 0.15;                                       // RRC Roll resistance coeff constant value





        ros::Publisher mov_pub;

#endif