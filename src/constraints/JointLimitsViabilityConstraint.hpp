#ifndef WBC_CORE_JOINT_LIMITS_VIABILITY_CONSTRAINT_HPP
#define WBC_CORE_JOINT_LIMITS_VIABILITY_CONSTRAINT_HPP

#include "../core/Constraint.hpp"
#include <memory>

namespace wbc{

/**
 * @brief Joint limits as bounds on the joint accelerations of an acceleration-based WBC, including joint position
 * limits enforced with the viability bound of Del Prete, "Joint Position and Velocity Bounds in Discrete-Time
 * Acceleration/Torque Control of Robot Manipulators", IEEE RA-L 2018.
 *
 * Drop-in replacement for JointLimitsAccelerationConstraint (same variable layout, same bounds type), which only
 * enforces acceleration limits. Per actuated joint, the bounds on qdd are the intersection of:
 *  - the joint acceleration limits (if defined in the robot model),
 *  - optionally the joint velocity limits (one-step bound: qd_{k+1} within limits),
 *  - optionally the joint position limits:
 *     - one-step bound: q_{k+1} = q + dt*qd + 0.5*dt^2*qdd within [q_min, q_max],
 *     - viability bound (if enabled): the next state must be able to stop before the limit with a deceleration of
 *       at most a_max, i.e., qd_{k+1} <= sqrt(2*a_max*(q_max - q_{k+1})), and symmetrically for q_min. Assuming
 *       constant acceleration over one control step, this gives a closed-form bound on qdd that only depends on the
 *       current state, so the QP stays linear. The position bounds never ask for a deceleration larger than a_max:
 *       if the current state is not viable, the joint brakes with a_max and may reach the limit.
 * In the full TSID formulation (reduced = false), the joint torques are bounded by the effort limits as well.
 */
class JointLimitsViabilityConstraint : public Constraint {
public:

    struct Options{
        /** Enforce joint velocity limits (one-step bound) */
        bool check_velocities = false;
        /** Enforce joint position limits */
        bool check_positions = true;
        /** If check_positions is true: add the viability bound on top of the one-step bound. Requires the
         *  viability accelerations to be set, see setViabilityAccelerations() */
        bool use_viability = true;
        /** Shrinks the position range on both sides [rad or m] */
        double position_margin = 0.0;
    };

    /**
     * @param dt Control time step [s]
     * @param reduced True if the joint torques are not QP variables (reduced TSID)
     * @param dim_contact Number of variables per contact (3: point contact, 6: surface contact)
     */
    explicit JointLimitsViabilityConstraint(double dt, bool reduced = false, uint dim_contact = 3);

    virtual ~JointLimitsViabilityConstraint() = default;

    virtual void update(RobotModelPtr robot_model) override;

    void setOptions(const Options& opts){options = opts;}
    const Options& getOptions() const {return options;}

    /**
     * @brief Maximum deceleration a_max (> 0) used by the viability bound, one entry per actuated joint, in the order
     * of RobotModel::jointNames(). Can be changed at every control cycle, e.g., depending on the contact state.
     */
    void setViabilityAccelerations(const Eigen::VectorXd& acc){viability_acc = acc;}
    const Eigen::VectorXd& getViabilityAccelerations() const {return viability_acc;}

protected:

    /** Control timestep: used to integrate and differentiate velocities */
    double dt;
    bool reduced;
    uint dim_contact;
    types::JointLimits joint_limits;
    Options options;
    Eigen::VectorXd viability_acc;
};
typedef std::shared_ptr<JointLimitsViabilityConstraint> JointLimitsViabilityConstraintPtr;

} // namespace wbc
#endif // WBC_CORE_JOINT_LIMITS_VIABILITY_CONSTRAINT_HPP
