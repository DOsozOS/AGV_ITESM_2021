#ifndef _AGV_PLUGIN_HH_
#define _AGV_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

//#include <gazebo/math.hh>
//ROS HEADER FILES
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32MultiArray.h"
// Include the cmath library
#include <cmath>

namespace gazebo
{
  class AGVplugin : public ModelPlugin
  {
    private: physics::WorldPtr world = physics::get_world("default");
    /// \brief Pointer to the model.
    private: physics::ModelPtr model;
    /// \brief Pointer to the left front steering joint.
    private: physics::JointPtr LFsteeringJoint;
    /// \brief Pointer to the right front steering joint.
    private: physics::JointPtr RFsteeringJoint;
    //Rear left Hinge
    private: physics::JointPtr RLhinge;
    //Rear right Hinge
    private: physics::JointPtr RRhinge;
    /// \brief A PID controller for the joint.
    private: common::PID steerPID1;
    private: common::PID speedPID1;
    private: common::PID steerPID2;
    private: common::PID speedPID2;
    //Add a Node and subscriber to the plugin.
    /// \brief A node used for transport
    private: transport::NodePtr SteeringNode;
    private: transport::NodePtr RearTorqueNode;    
    private: transport::NodePtr RearWheelSpeedNode;
    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr Steering_sub;
    private: transport::SubscriberPtr Torque_sub;
    private: transport::SubscriberPtr Speed_sub;

    //MEMBER VARIABLES FOR ROS COM
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNodeSteering;
    private: std::unique_ptr<ros::NodeHandle> rosRearTorqueNode;
    private: std::unique_ptr<ros::NodeHandle> rosRearSpeedNode;
    private: std::unique_ptr<ros::NodeHandle> rospetitionNode;
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub_Steering;
    private: ros::Subscriber rosSub_Torque;
    private: ros::Subscriber rosSub_Speed;
    private: ros::Subscriber rosSub_Cmd;
    private: ros::Publisher rosPub_Status;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue_Steering;
    private: ros::CallbackQueue rosQueue_Torque;
    private: ros::CallbackQueue rosQueue_Speed;
    private: ros::CallbackQueue rosQueue_Cmd;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread_Steering;
    private: std::thread rosQueueThread_Torque;
    private: std::thread rosQueueThread_Speed;
    private: std::thread rosQueueThread_Cmd;
    /// sun light gazebo

    private: double alphaIzqu;
    private: double alphaDerc;
  private: 

    public: void SetSteering(const double &_steering)
    {
      double L = 2;
      double T = 1;
      double R = L/sin(_steering);
      double alphaIzqu = atan(L/(R-T/2));
      double alphaDerc = atan(L/(R+T/2));

      this->LFsteeringJoint->SetPosition(0,alphaIzqu,true);
      this->RFsteeringJoint->SetPosition(0,alphaDerc,true);
      //this->LFsteeringJoint->SetPosition(0,-0.008726646259972,true);
      //this->RFsteeringJoint->SetPosition(0,-0.095993108859688,true);
    }
    public: void RearWheelDriveTorque(const double &_torque)
    {
      this->model->GetLinks()[5]->AddRelativeTorque(ignition::math::Vector3<double>(0.0f, 0.0f, _torque));
      this->model->GetLinks()[6]->AddRelativeTorque(ignition::math::Vector3<double>(0.0f, 0.0f, _torque));
    }
    public: void RearWheelSpeed(const double &_speed)
    {
      this->model->GetJointController()->SetVelocityTarget(this->RLhinge->GetScopedName(), _speed);
      this->model->GetJointController()->SetVelocityTarget(this->RRhinge->GetScopedName(), _speed);
    }
    public: void PublisherStatus(const int &_cmd)
    {
      double Xpose = this->model->GetLinks()[0]->WorldCoGPose().Pos().X();
      double Ypose = this->model->GetLinks()[0]->WorldCoGPose().Pos().Y();
      double Zpose = this->model->GetLinks()[0]->WorldCoGPose().Pos().Z();
      double Yaw = this->model->GetLinks()[0]->WorldCoGPose().Rot().Yaw();
      double rotarySpeed = (this->RLhinge->GetVelocity(0)+this->RRhinge->GetVelocity(0))/2;
      double longitudinalSpeed = this->model->RelativeLinearVel().X();
      double lateralSpeed = this->model->RelativeLinearVel().Y();
      double worldTime = this->world->SimTime().Double();
      std_msgs::Float32MultiArray array;
      array.data.clear();
      array.data.push_back(Xpose);
      array.data.push_back(Ypose);
      array.data.push_back(Zpose);      
      array.data.push_back(rotarySpeed);
      array.data.push_back(longitudinalSpeed);  
      array.data.push_back(lateralSpeed);      
      array.data.push_back(worldTime);     
      array.data.push_back(Yaw);
      rosPub_Status.publish(array);
    }
    private: void OnMsgSteering(ConstVector3dPtr &_msg)
    {
      this->SetSteering(_msg->x());
    }
    private: void OnMsgForce(ConstVector3dPtr &_msg)
    {
      this->RearWheelDriveTorque(_msg->x());        
    }
    private: void OnMsgSpeed(ConstVector3dPtr &_msg)
    {
      this->RearWheelSpeed(_msg->x());        
    }

    public: AGVplugin()
            {
              printf("Starting AGV Plugin\n");
            }
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {      
      printf("VERSION 1.0\n");
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, Something must be wrong with the AGV model.sdf file\n";
        return;
      }
      int joints = _model->GetJointCount();
      std::cerr << "\nNumber of the AGV joints ";
      std::cerr << joints;
      this->model = _model;
      
      std::cerr << "\nAGV Joint 0 "+model->GetJoints()[0]->GetName() + " \n";
      std::cerr << "AGV Joint 1 "+model->GetJoints()[1]->GetName() + " \n";
      std::cerr << "AGV Joint 2 "+model->GetJoints()[2]->GetName() + " \n";
      std::cerr << "AGV Joint 3 "+model->GetJoints()[3]->GetName() + " \n";
      std::cerr << "AGV Joint 4 "+model->GetJoints()[4]->GetName() + " \n";
      std::cerr << "AGV Joint 5 "+model->GetJoints()[5]->GetName() + " \n";

      this->speedPID1 = common::PID(4, 0.1, 0.01);
      this->speedPID2 = common::PID(4, 0.1, 0.01);
      this->LFsteeringJoint = model->GetJoints()[0];
      this->RFsteeringJoint = model->GetJoints()[2];
      this->RRhinge = model->GetJoints()[4];
      this->RLhinge = model->GetJoints()[5];

      this->model->GetJointController()->SetVelocityPID(
        this->RRhinge->GetScopedName(), this->speedPID1);
      this->model->GetJointController()->SetVelocityPID(
        this->RLhinge->GetScopedName(), this->speedPID2);

      this->model->GetJointController()->SetVelocityTarget(this->RLhinge->GetScopedName(), 0.0);
      this->model->GetJointController()->SetVelocityTarget(this->RRhinge->GetScopedName(), 0.0);
      this->model->GetJointController()->Update();
      
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }

      
      // Create our ROS node. This acts in a similar manner to the Gazebo node
      this->rosNodeSteering.reset(new ros::NodeHandle("gazebo_client"));
      this->rosRearTorqueNode.reset(new ros::NodeHandle("gazebo_client"));
      this->rosRearSpeedNode.reset(new ros::NodeHandle("gazebo_client"));
      this->rospetitionNode.reset(new ros::NodeHandle("gazebo_client"));
      ros::NodeHandle StatusNode;

      ros::SubscribeOptions so_steering =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->model->GetName() + "/steering_cmd",
            1,
            boost::bind(&AGVplugin::OnRosMsgSteering, this, _1),
            ros::VoidPtr(), &this->rosQueue_Steering);

      ros::SubscribeOptions so_torque =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->model->GetName() + "/rear_force_cmd",
            1,
            boost::bind(&AGVplugin::OnRosMsgTorque, this, _1),
            ros::VoidPtr(), &this->rosQueue_Torque);

      ros::SubscribeOptions so_speed =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->model->GetName() + "/rear_speed_cmd",
            1,
            boost::bind(&AGVplugin::OnRosMsgSpeed, this, _1),
            ros::VoidPtr(), &this->rosQueue_Speed);

      ros::SubscribeOptions so_cmd =
        ros::SubscribeOptions::create<std_msgs::Int32>(
            "/" + this->model->GetName() + "/status_cmd",
            1,
            boost::bind(&AGVplugin::OnRosMsgStatus, this, _1),
            ros::VoidPtr(), &this->rosQueue_Cmd);


      this->rosSub_Steering = this->rosNodeSteering->subscribe(so_steering);
      this->rosSub_Torque = this->rosRearTorqueNode->subscribe(so_torque);
      this->rosSub_Speed = this->rosRearSpeedNode->subscribe(so_speed);
      this->rosSub_Cmd = this->rospetitionNode->subscribe(so_cmd);
      this->rosPub_Status= StatusNode.advertise<std_msgs::Float32MultiArray>("/my_agv/status_pub", 1);

      // Spin up the queue helper thread.
      this->rosQueueThread_Steering =
        std::thread(std::bind(&AGVplugin::QueueThread_Steering, this));
      this->rosQueueThread_Torque =
        std::thread(std::bind(&AGVplugin::QueueThread_Torque, this));
      this->rosQueueThread_Speed =
        std::thread(std::bind(&AGVplugin::QueueThread_Speed, this));
      this->rosQueueThread_Cmd =
        std::thread(std::bind(&AGVplugin::QueueThread_Status, this));
    }

      public: void OnRosMsgSteering(const std_msgs::Float32ConstPtr &_msg)
      {
        this->SetSteering(_msg->data);
      }
      public: void OnRosMsgTorque(const std_msgs::Float32ConstPtr &_msg)
      {
        this->RearWheelDriveTorque(_msg->data);        
      }
      public: void OnRosMsgSpeed(const std_msgs::Float32ConstPtr &_msg)
      {
        this->RearWheelSpeed(_msg->data);        
      }
      public: void OnRosMsgStatus(const std_msgs::Int32ConstPtr &_msg)
      {
        this->PublisherStatus(_msg->data);        
      }

      private: void QueueThread_Steering()
      {
        static const double timeout = 0.01;
        while (this->rosNodeSteering->ok())
        {
          this->rosQueue_Steering.callAvailable(ros::WallDuration(timeout));
        }
      }
      private: void QueueThread_Torque()
      {
        static const double timeout = 0.01;
        while (this->rosRearTorqueNode->ok())
        {
          this->rosQueue_Torque.callAvailable(ros::WallDuration(timeout));
        }
      }
      private: void QueueThread_Speed()
      {
        static const double timeout = 0.01;
        while (this->rosRearSpeedNode->ok())
        {
          this->rosQueue_Speed.callAvailable(ros::WallDuration(timeout));
        }
      }
      private: void QueueThread_Status()
      {
        static const double timeout = 0.01;
        while (this->rospetitionNode->ok())
        {
          this->rosQueue_Cmd.callAvailable(ros::WallDuration(timeout));
        }
      }
  };
  GZ_REGISTER_MODEL_PLUGIN(AGVplugin)
}
#endif

