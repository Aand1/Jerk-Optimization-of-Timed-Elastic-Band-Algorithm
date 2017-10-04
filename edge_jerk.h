#ifndef EDGE_JERK_H
#define EDGE_JERK_H

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>

#include <iostream>

namespace teb_local_planner
{
class EdgeJerk : public BaseTebMultiEdge<2,double>
{
public:
    EdgeJerk()
    {
        this->resize(7);
    }
    void computeError()
    {
        ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on Edgejerk()");
        const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
        const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
        const VertexPose* conf3 = static_cast<const VertexPose*>(_vertices[2]);
        const VertexPose* conf4 = static_cast<const VertexPose*>(_vertices[3]);
        const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[4]);
        const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[5]);
        const VertexTimeDiff* dt3 = static_cast<const VertexTimeDiff*>(_vertices[6]);

        //traslational and angular differences
        const Eigen::Vector2D diff1 = conf2->position()-conf1->position();
        const Eigen::Vector2D diff2 = conf3->position()-conf2->position();
        const Eigen::Vector2D diff3 = conf4->position()-conf3->position();

        double dist1 = diff1.norm();
        double dist2 = diff2.norm();
        double dist3 = diff3.norm();

        const double angle_diff1 = g2o::normalize_theta(conf2->theta()-conf1->theta());
        const double angle_diff2 = g2o::normalize_theta(conf3->theta()-conf2->theta());
        const double angle_diff3 = g2o::normalize_theta(conf4->theta()-conf3->theta());


        if (cfg_->trajectory.exact_arc_length)
        {
            if (angle_diff1!=0)
            {
                const double radius = dist1/(2*sin(angle_diff1/2));
                dist1 = fabs(radius*angle_diff1);
            }
            if (angle_diff2!=0)
            {
                const double radius = dist2/(2*sin(angle_diff2/2));
                dist2 = fabs(radius*angle_diff2);
            }
            if (angle_diff3!=0)
            {
                const double radius = dist3/(2*sin(angle_diff3/2));
                dist3 = fabs(radius*angle_diff3);
            }

        }
        // translational velocity

        double vel1 = dist1/dt1->dt();
        double vel2 = dist2/dt2->dt();
        double vel3 = dist3/dt3->dt();

        // translational velocity directions

        vel1 *= fast_sigmoid(100*(diff1.x()*cos(conf1->theta())+diff1.y()*sin(conf1->theta())));
        vel2 *= fast_sigmoid(100*(diff2.x()*cos(conf2->theta())+diff2.y()*sin(conf2->theta())));
        vel3 *= fast_sigmoid(100*(diff3.x()*cos(conf3->theta())+diff3.y()*sin(conf3->theta())));

        // translational acceleration

        const double acc_lin1 = (vel2-vel1)*2/(dt1->dt()+dt2->dt());
        const double acc_lin2 = (vel3-vel2)*2/(dt2->dt()+dt3->dt());

        // angular velocity

        double omega1 = angle_diff1/dt1->dt();
        double omega2 = angle_diff2/dt2->dt();
        double omege3 = angle_diff3/dt3->dt();

        // angular acceleration

        const double acc_rot1 = (omega2-omega1)*2/(dt1->dt()+dt2->dt());
        const double acc_rot2 = (omega3-omega2)*2/(dt2->dt()+dt3->dt());

        // tranlational and angular jerk

        const double jerk_lin = (acc_lin2-acc_lin1)/(0.25*dt1->dt()+0.5*dt2->dt()+0.25*dt3->dt());
        const double jerk_rot = (acc_rot2-acc_rot1)/(0.25*dt1->dt()+0.5*dt2->dt()+0.25*dt3->dt());

        // edge error
        _error[0] = penaltyBoundToInterval(jerk_lin,cfg_->robot.jerk_lim_x,cfg_->optim.penalty_epsilon);
        _error[1] = penaltyBoundToInterval(jerk_rot,cfg_->robot.jerk_lim_theta,cfg_->optim.penalty_epsilon);

        ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeJerk::computeError() translational: _error[0]=%f\n",_error[0]);
        ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeJerk::computeError() rotational: _error[1]=%f\n",_error[1]);
    }

};

class EdgeJerkStart : public BaseTebMultiEdge<2, const geometry_msgs::Twist*>
{
public:
  EdgeJerkStart()
  {
      _measurement = Null;
      this->resize(5);
  }
  void computeError()
  {
      ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeJerkStart()");
      const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
      const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
      const VertexPose* conf3 = static_cast<const VertexPose*>(_vertices[2]);
      const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
      const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

      //traslational and angular differences
      const Eigen::Vector2D diff1 = conf2->position()-conf1->position();
      const Eigen::Vector2D diff2 = conf3->position()-conf2->position();

      double dist1 = diff1.norm();
      double dist2 = diff2.norm();

      const double angle_diff1 = g2o::normalize_theta(conf2->theta()-conf1->theta());
      const double angle_diff2 = g2o::normalize_theta(conf3->theta()-conf2->theta());

       if (cfg_->trajectory.exact_arc_length)
        {
            if (angle_diff1!=0)
            {
                const double radius = dist1/(2*sin(angle_diff1/2));
                dist1 = fabs(radius*angle_diff1);
            }
            if (angle_diff2!=0)
            {
                const double radius = dist2/(2*sin(angle_diff2/2));
                dist2 = fabs(radius*angle_diff2);
            }

        }
       // translational velocity

      double vel1 = _measurement->linear.x;
      double vel2 = dist1/dt1->dt();
      double vel3 = dist2/dt2->dt();

      // translational velocity directions

      vel2 *= fast_sigmoid(100*(diff1.x()*cos(conf2->theta())+diff1.y()*sin(conf1->theta())));
      vel3 *= fast_sigmoid(100*(diff2.x()*cos(conf3->theta())+diff2.y()*sin(conf2->theta())));

      // translational acceleration

      const double acc_lin1 = (vel2-vel1)/dt1->dt();
      const double acc_lin2 = (vel3-vel2)/dt2->dt();

      // angular velocity

      double omega1 = _measurement->angular.z;
      double omega2 = angle_diff1/dt1->dt();
      double omege3 = angle_diff2/dt2->dt();

      // angular acceleration

      const double acc_rot1 = (omega2-omega1)/dt1->dt();
      const double acc_rot2 = (omega3-omega2)/dt2->dt();

     // tranlational and angular jerk

      const double jerk_lin = (acc_lin2-acc_lin1)/(0.5*dt1->dt()+0.5*dt2->dt());
      const double jerk_rot = (acc_rot2-acc_rot1)/(0.5*dt1->dt()+0.5*dt2->dt());

    // edge error
      _error[0] = penaltyBoundToInterval(jerk_lin,cfg_->robot.jerk_lim_x,cfg_->optim.penalty_epsilon);
      _error[1] = penaltyBoundToInterval(jerk_rot,cfg_->robot.jerk_lim_theta,cfg_->optim.penalty_epsilon);

      ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeJerk::computeError() translational: _error[0]=%f\n",_error[0]);
      ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeJerk::computeError() rotational: _error[1]=%f\n",_error[1]);      
  }
   void setInitialVelocity(const geometry_msgs::Twist& vel_start)
  {
    _measurement = &vel_start;
  }

};

class EdgeJerkGoal : public BaseTebMultiEdge<2, const geometry_msgs::Twist*>
{
public:
  EdgeJerkGoal()
  {
      _measurement=Null;
      this->resize(5);
  }
  computeError()
  {
       ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeJerkStart()");
      const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
      const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
      const VertexPose* conf3 = static_cast<const VertexPose*>(_vertices[2]);
      const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
      const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

      //traslational and angular differences
      const Eigen::Vector2D diff1 = conf2->position()-conf1->position();
      const Eigen::Vector2D diff2 = conf3->position()-conf2->position();

      double dist1 = diff1.norm();
      double dist2 = diff2.norm();

      const double angle_diff1 = g2o::normalize_theta(conf2->theta()-conf1->theta());
      const double angle_diff2 = g2o::normalize_theta(conf3->theta()-conf2->theta());

       if (cfg_->trajectory.exact_arc_length)
        {
            if (angle_diff1!=0)
            {
                const double radius = dist1/(2*sin(angle_diff1/2));
                dist1 = fabs(radius*angle_diff1);
            }
            if (angle_diff2!=0)
            {
                const double radius = dist2/(2*sin(angle_diff2/2));
                dist2 = fabs(radius*angle_diff2);
            }

        }
       // translational velocity

      double vel1 = dist1/dt1->dt();
      double vel2 = dist2/dt2->dt();
      double vel3 = _measurement->linear.x;

      // translational velocity directions

      vel2 *= fast_sigmoid(100*(diff1.x()*cos(conf1->theta())+diff1.y()*sin(conf2->theta())));
      vel3 *= fast_sigmoid(100*(diff2.x()*cos(conf2->theta())+diff2.y()*sin(conf2->theta())));

      // translational acceleration

      const double acc_lin1 = (vel2-vel1)/dt1->dt();
      const double acc_lin2 = (vel3-vel2)/dt2->dt();

      // angular velocity

      double omega1 = angle_diff1/dt1->dt();
      double omega2 = angle_diff2/dt2->dt();
      double omege3 = _measurement->angular.z;

      // angular acceleration

      const double acc_rot1 = (omega2-omega1)/dt1->dt();
      const double acc_rot2 = (omega3-omega2)/dt2->dt();

     // tranlational and angular jerk

      const double jerk_lin = (acc_lin2-acc_lin1)/(0.5*dt1->dt()+0.5*dt2->dt());
      const double jerk_rot = (acc_rot2-acc_rot1)/(0.5*dt1->dt()+0.5*dt2->dt());

     // edge error
      _error[0] = penaltyBoundToInterval(jerk_lin,cfg_->robot.jerk_lim_x,cfg_->optim.penalty_epsilon);
      _error[1] = penaltyBoundToInterval(jerk_rot,cfg_->robot.jerk_lim_theta,cfg_->optim.penalty_epsilon);

      ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeJerk::computeError() translational: _error[0]=%f\n",_error[0]);
      ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeJerk::computeError() rotational: _error[1]=%f\n",_error[1]);      

  }
  void setGoalVelocity(const geometry_msgs::Twist& vel_goal)
  {
    _measurement = &vel_goal;
  }

};
}
#endif
