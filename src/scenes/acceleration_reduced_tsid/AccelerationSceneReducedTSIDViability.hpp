#ifndef WBC_ACCELERATION_SCENE_REDUCED_TSID_VIABILITY_HPP
#define WBC_ACCELERATION_SCENE_REDUCED_TSID_VIABILITY_HPP

#include "AccelerationSceneReducedTSID.hpp"
#include "../../constraints/JointLimitsViabilityConstraint.hpp"

namespace wbc{

/**
 * @brief AccelerationSceneReducedTSID with joint position limits: same QP, but the joint limits constraint
 * (JointLimitsAccelerationConstraint, acceleration limits only) is replaced by JointLimitsViabilityConstraint, which
 * also enforces joint position (and optionally velocity) limits with the viability bound of Del Prete (RA-L 2018).
 * Configure it via getJointLimitsConstraint(): JointLimitsViabilityConstraint::setOptions() and
 * JointLimitsViabilityConstraint::setViabilityAccelerations() (required, as the viability bound is enabled by default).
 */
class AccelerationSceneReducedTSIDViability : public AccelerationSceneReducedTSID{
protected:
    static SceneRegistry<AccelerationSceneReducedTSIDViability> reg;

    JointLimitsViabilityConstraintPtr joint_limits_constraint;

public:
    AccelerationSceneReducedTSIDViability(RobotModelPtr robot_model, QPSolverPtr solver, const double dt, uint dim_contact = 3, bool use_spatial_acc_bias = true);
    virtual ~AccelerationSceneReducedTSIDViability(){}

    /** @brief The joint limits constraint of this scene, e.g., to set its options and viability accelerations */
    JointLimitsViabilityConstraintPtr getJointLimitsConstraint(){return joint_limits_constraint;}
};

} // namespace wbc

#endif // WBC_ACCELERATION_SCENE_REDUCED_TSID_VIABILITY_HPP
