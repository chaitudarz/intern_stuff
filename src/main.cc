#include "dil_controller_ros_cpp.h"



    class DilController {
    public:
        DilController();

    private:
        void joyxboxCallback(const sensor_msgs::Joy::ConstPtr &joy_xbox);
        void joyLogitechCallback(const sensor_msgs::Joy::ConstPtr &joy_Logitech);
        //void simFrameCallback(const SimpleSim_Messages_ROS::SimFrameConstPtr &sf);
        double EngineModel(double acceleration);

        ros::NodeHandle nh_;

        ros::Publisher vel_pub_;
        ros::Subscriber joy_xbox_sub_;
        ros::Subscriber joy_Logitech_sub_;
        //ros::Subscriber simFrame_sub;

    };


    DilController::DilController() :
 {

        vel_pub_ = nh_.advertise<SimpleSim_Messages_ROS::DriverControl>("/mov_vel", 1);


        joy_xbox_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &DilController::joyxboxCallback, this);

        joy_Logitech_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &DilController::joyLogitechCallback, this);

        //simFrame_sub = nh_.subscribe<SimpleSim_Messages_ROS::SimFrame>("/SimFrameData", &DilController::simFrameCallback, this);

    }



    /** Engine Model */

    double DilController::EngineModel(double velocity)
    {


        DeltaTime_ = (Begin_- CurrTime_);
        Current_Velocity_ = velocity;
        Force_ = Mass_ * ((Current_Velocity_- Previous_Vel_)/(DeltaTime_));                          //F=m*a -----> a=(change in acc)/(change in time).
        Displ_ = ((Previous_Vel_ + Current_Velocity_) * 0.5)*DeltaTime_;                            // displacement without acceleration, displacement =  (1/2*(V_int + V_final)) * time.
        Speed_ =  Current_Velocity_;                                                                // both speed and velocity are same. speed = distance/ time ..&&.. velocity = distance/time
        DragForce_ = 0.5 * Airdensity_ * DragCoeff_ * AreaOfCar_ * (Speed_ * Speed_) ;
        ForceRoll_ = RRC * Mass_ * gravity_due_acc;

        Previous_Vel_ = Current_Velocity_;
        CurrTime_ = ros::Time::now().toSec();
        if(Force_ != 0)
        return  ((Force_ - (DragForce_ + ForceRoll_))/ Mass_);

    }

    /** Xbox Callback*/
    void DilController::joyxboxCallback(const sensor_msgs::Joy::ConstPtr &joyXbox) {

        Begin_ = ros::Time::now().toSec();

        SimpleSim_Messages_ROS::DriverControl movement;

        //xbox steer:
        movement.steerAngle = joyXbox->axes[3];
        std::cout << "xbox steer: " << movement.steerAngle << std::endl;

        /**calculalting the input joy values based on the EngineModel
         * scaled acceleration --> gives values in between 0 to 1 ---< axes 2 in xbox
         * scaled_brake --> gives values in between 0 to 1 ---> axes 5 in xbox  */

        scaled_acceleration = -(0.5*(joyXbox->axes[2]-1));
        scaled_brake = (0.5*(joyXbox->axes[5]-1));

        if(scaled_acceleration > 0 && scaled_brake < 0)
        {
            /** calculating the mean and passing the value to Engine model */
            actual_movement_to_vehicle = ((scaled_acceleration+scaled_brake) * 0.5);
        }
        else if(scaled_acceleration>0)
        {
            actual_movement_to_vehicle = scaled_acceleration;
        }
        else if(scaled_brake < 0)
        {
            actual_movement_to_vehicle = scaled_acceleration;
        }

        double joy_velocity = EngineModel(actual_movement_to_vehicle);
        movement.acceleration = joy_velocity;
        std::cout << "Joy_velocity:" << joy_velocity << std::endl;
        vel_pub_.publish(movement);
    }

    /** Logitech Callback*/

    void DilController::joyLogitechCallback(const sensor_msgs::Joy::ConstPtr &joyLog) {

        Begin_ = ros::Time::now().toSec();

        SimpleSim_Messages_ROS::DriverControl movement;

        //scaling the pedal in between -1 to 1 ==> 0 to 1
        movement.steerAngle = joyLog->axes[0];
        //std::cout << "Logitech_values: " << movement.steerAngle << std::endl;

        //calucalte the input joy values based on the EngineModel
        /**if we remap the joystick axes of both xbox and Logitech wheel in launch,
         * then we need not to give the axes values,
         * Other wise the Acceleration pedal axes is 2
         * Brake pedal axes is 3*/

        scaled_acceleration =  -(0.5*(joyLog->axes[2]-1));
        scaled_brake = (0.5*(joyLog->axes[3]-1));

        if(scaled_acceleration > 0 && scaled_brake < 0)
        {
            /** calculating the mean and passing the value to Engine model */
            actual_movement_to_vehicle = ((scaled_acceleration+scaled_brake) * 0.5);
        }
        else if(scaled_acceleration>0)
        {
            actual_movement_to_vehicle = scaled_acceleration;
        }
        else if(scaled_brake < 0)
        {
            actual_movement_to_vehicle = scaled_acceleration;
        }

        double joy_velocity = EngineModel(actual_movement_to_vehicle);
        movement.acceleration = joy_velocity;
        //std::cout << "Joy_velocity:" << joy_velocity << std::endl;
        vel_pub_.publish(movement);

    }


    int main(int argc, char **argv) {
        ros::init(argc, argv, "dil_controller");
        DilController dil_controller;

        ros::spin();
    }

