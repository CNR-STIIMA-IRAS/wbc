#include "JointLimitsViabilityConstraint.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace wbc{

JointLimitsViabilityConstraint::JointLimitsViabilityConstraint(double dt, bool reduced, uint dim_contact)
    : Constraint(Constraint::bounds),
      dt(dt),
      reduced(reduced),
      dim_contact(dim_contact){
}

void JointLimitsViabilityConstraint::update(RobotModelPtr robot_model){

    const uint nj = robot_model->nj();
    const uint na = robot_model->na();
    const uint nc = robot_model->nc();
    const uint nfb = robot_model->nfb();
    const uint nv = reduced ? nj + dim_contact*nc : nj + na + dim_contact*nc;

    const bool check_positions = options.check_positions;
    const bool use_viability = check_positions && options.use_viability;
    if(use_viability && (uint)viability_acc.size() != na)
        throw std::runtime_error("JointLimitsViabilityConstraint: viability accelerations have size "
                                 + std::to_string(viability_acc.size()) + " but should have size " + std::to_string(na));

    // Default bounds (no limit), as in JointLimitsAccelerationConstraint
    lb_vec.setConstant(nv, -10000);
    ub_vec.setConstant(nv, +10000);

    const auto position = robot_model->getQ().tail(na);
    const auto velocity = robot_model->getQd().tail(na);
    joint_limits = robot_model->jointLimits();

    for(uint i = 0; i < na; i++){

        const double q = position[i];
        const double qd = velocity[i];
        double lb = -10000, ub = +10000;

        // Acceleration limits (might not be defined in the URDF)
        if(!std::isnan(joint_limits.min.acceleration[i])) lb = joint_limits.min.acceleration[i];
        if(!std::isnan(joint_limits.max.acceleration[i])) ub = joint_limits.max.acceleration[i];

        // Velocity limits, one-step bound
        if(options.check_velocities && std::isfinite(joint_limits.min.velocity[i]) && std::isfinite(joint_limits.max.velocity[i])){
            lb = std::max(lb, (joint_limits.min.velocity[i] - qd) / dt);
            ub = std::min(ub, (joint_limits.max.velocity[i] - qd) / dt);
        }

        // Position limits
        const double q_min = joint_limits.min.position[i] + options.position_margin;
        const double q_max = joint_limits.max.position[i] - options.position_margin;
        if(check_positions && std::isfinite(q_min) && std::isfinite(q_max)){

            // One-step bound: q_{k+1} = q + dt*qd + 0.5*dt^2*qdd within [q_min, q_max]
            lb = std::max(lb, 2*(q_min - q - dt*qd) / (dt*dt));
            ub = std::min(ub, 2*(q_max - q - dt*qd) / (dt*dt));

            if(use_viability){
                // Upper limit: require qd + dt*qdd <= sqrt(2*a*(D_max - 0.5*dt^2*qdd)), D_max = q_max - q - dt*qd,
                // i.e., from the next state the joint can still stop before q_max with deceleration a. Squaring gives
                // a quadratic inequality in qdd, whose larger root is the bound below. A negative discriminant means
                // that the state is already not viable: brake with -a.
                const double a = viability_acc[i];
                const double d_max = q_max - q - dt*qd;
                const double disc_max = a*a*dt*dt + 4*a*dt*qd + 8*a*d_max;
                ub = std::min(ub, disc_max >= 0 ? (-2*qd - a*dt + std::sqrt(disc_max)) / (2*dt) : -a);

                // Lower limit, symmetric (q -> -q)
                const double d_min = q - q_min + dt*qd;
                const double disc_min = a*a*dt*dt - 4*a*dt*qd + 8*a*d_min;
                lb = std::max(lb, disc_min >= 0 ? (-2*qd + a*dt - std::sqrt(disc_min)) / (2*dt) : a);

                // If the state is not viable (moving too fast towards a limit to stop before it with deceleration a),
                // the bounds above ask for a deceleration larger than a, up to thousands of rad/s^2, which the robot
                // cannot produce and which makes the QP infeasible. Never ask for more than a: brake as hard as
                // allowed and accept reaching the limit.
                ub = std::max(ub, -a);
                lb = std::min(lb, a);
            }
        }

        // Keep the box consistent: lb > ub can happen if the state is outside the limits or if the position bounds
        // conflict with the acceleration limits. Then prefer moving away from the closer limit.
        if(lb > ub){
            if(check_positions && joint_limits.max.position[i] - q < q - joint_limits.min.position[i])
                lb = ub;
            else
                ub = lb;
        }
        lb_vec(nfb + i) = lb;
        ub_vec(nfb + i) = ub;
    }

    // Joint effort limits, only if the torques are QP variables (full TSID). In the reduced TSID they are enforced
    // by EffortLimitsAccelerationConstraint
    if(reduced)
        return;
    for(uint i = 0; i < na; i++){
        lb_vec(nj + i) = joint_limits.min.effort[i];
        ub_vec(nj + i) = joint_limits.max.effort[i];
    }
}

} // namespace wbc
