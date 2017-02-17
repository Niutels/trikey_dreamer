#include <ros/ros.h>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <sdf/parser.hh>
#include <stdio.h>
#include <unistd.h>

namespace gazebo
{
  class TrikeyDreamerPlugin: public ModelPlugin
  {
  public:
    TrikeyDreamerPlugin()
    {
      printf("Hello World!\n");
      std::cout << "Constructing TrikeyDreamerPlugin" << std::endl;
      first_iter=true;
      q0 = std::vector<double>();
      prior_q = std::vector<double>();
      q_dot = std::vector<double>();
    }

    ~TrikeyDreamerPlugin()
    {
    }

    template<typename T>
    T sdf_get_value(sdf::ElementPtr sdf, const std::string &key, const T &def)
    {
      if(sdf->HasElement(key))
      {
        std::string value = sdf->GetElement(key)->Get<std::string>();
        return boost::lexical_cast<T>(value);
      }
      else
        return def;
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr sdfElement)
    {
      printf("Hello Loading Package!\n");
      // std::cerr << "Loading TrikeyDreamerPlugin" << std::endl;
      this->model = _parent;
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&TrikeyDreamerPlugin::OnUpdate, this, _1));

      gazebo::physics::Joint_V joints = this->model->GetJoints();
      gazebo::physics::Link_V links = this->model->GetLinks();

      printf("1\n");

      for(unsigned int i = 0; i < joints.size(); i++)
      {
        joints[i]->GetName();
        joints[i]->GetLowStop(0).Radian();
        joints[i]->GetHighStop(0).Radian();
      }
      printf("2\n");

      // next_loop_time = boost::get_system_time();
      // std::cout << "Done loading ValkyireAnklePlugin" << std::endl;
    }

    bool first_iter;
    long last_iters;
    long iters;
    std::vector<double> q0;
    std::vector<double> q_dot; // dq/dt
    std::vector<double> prior_q; // prior value of q

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      double seconds_per_step = this->model->GetWorld()->GetPhysicsEngine()->GetUpdatePeriod();
      double alternate_time = this->model->GetWorld()->GetSimTime().Double();
      // std::cerr << "Update spam! from ValkyireAnklePlugin" << std::endl;
      last_iters = iters;
      iters = this->model->GetWorld()->GetIterations();

      gazebo::physics::Joint_V joints = this->model->GetJoints();
      if (first_iter)
      {
        for(unsigned int i = 0; i < joints.size(); i++)
        {
          q0.push_back(joints[i]->GetAngle(0).Radian());
          prior_q.push_back(q0[i]);
          q_dot.push_back(0.0);
          std::cout<<joints[i]->GetName()<<std::endl;
        }
        first_iter=false;
      }
      for(unsigned int i = 0; i < joints.size(); i++)
      {
        q_dot[i]=(joints[i]->GetAngle(0).Radian()-prior_q[i])*100.0;
        prior_q[i] = joints[i]->GetAngle(0).Radian();
      }

      // read directly from the links
      for(unsigned int i = 0; i < joints.size(); i++)
      {
        joints[i]->GetName();
        gazebo::physics::JointWrench gazebo_wrench = joints[i]->GetForceTorque(0);
        math::Vector3 axis = joints[i]->GetLocalAxis(0);
        double raw_position = joints[i]->GetAngle(0).Radian();
        // std::cout<< raw_position<<", ";
        double raw_velocity = joints[i]->GetVelocity(0);
        double raw_torque = -(axis.x * gazebo_wrench.body2Torque.x + 
          axis.y * gazebo_wrench.body2Torque.y + 
          axis.z * gazebo_wrench.body2Torque.z);
        joints[i]->GetForce(0);

      }
      // read more from the root link
      gazebo::physics::Link_V links = this->model->GetLinks();
      gazebo::math::Pose root_link_pose = links.at(0)->GetWorldPose();
      gazebo::math::Vector3 root_link_linear_vel = links.at(0)->GetWorldLinearVel();
      gazebo::math::Vector3 root_link_angular_vel = links.at(0)->GetWorldAngularVel();
      gazebo::math::Vector3 root_link_linear_accel = links.at(0)->GetWorldLinearAccel();
      gazebo::math::Vector3 root_link_angular_accel = links.at(0)->GetWorldAngularAccel();

      //apply command
      for(unsigned int i = 0; i < joints.size(); i++)
      {
          double error = joints[i]->GetAngle(0).Radian()-q0[i];
          //joints[i]->SetForce(0, -1000.0*error-100.0*q_dot[i]);
          if (joints[i]->GetName() == "base_to_wheel_j0"){
            joints[i]->SetForce(0, 5.0 + 1.0);
          }
          else if (joints[i]->GetName() == "base_to_wheel_j1"){
            joints[i]->SetForce(0, -5.0 + 1.0);
          }
          else if (joints[i]->GetName() == "base_to_wheel_j2"){
            joints[i]->SetForce(0, 1.0);
          } 
          else if ((joints[i]->GetName() == "torso_lower_pitch") ||
                  (joints[i]->GetName() == "torso_yaw") ||
                  (joints[i]->GetName() == "right_shoulder_extensor") ||
                  (joints[i]->GetName() == "left_shoulder_extensor") ||                  
                  (joints[i]->GetName() == "torso_upper_pitch")) {
            joints[i]->SetForce(0, -500.0*error);
          }

          else{
            //joints[i]->SetForce(0, -50.0*error);
          }

      }
    }

  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
  };

// Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN (TrikeyDreamerPlugin)

} // namespace gazebo