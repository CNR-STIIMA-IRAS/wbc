#include "AccelerationSceneReducedTSIDViability.hpp"
#include "../../constraints/JointLimitsAccelerationConstraint.hpp"
#include <stdexcept>
#include <string>

namespace wbc {

SceneRegistry<AccelerationSceneReducedTSIDViability> AccelerationSceneReducedTSIDViability::reg("acceleration_reduced_tsid_viability");

AccelerationSceneReducedTSIDViability::AccelerationSceneReducedTSIDViability(RobotModelPtr robot_model, QPSolverPtr solver, const double dt, uint dim_contact, bool use_spatial_acc_bias) :
    AccelerationSceneReducedTSID(robot_model, solver, dt, dim_contact, use_spatial_acc_bias){

    // The base class creates its constraints in its constructor. Replace its joint limits constraint (a single bounds
    // constraint is supported by the scene) with the viability one, keeping the order of the constraints.
    const bool reduced = true;
    joint_limits_constraint = std::make_shared<JointLimitsViabilityConstraint>(dt, reduced, dim_contact);
    uint n_replaced = 0;
    for(ConstraintPtr& constraint : constraints){
        if(std::dynamic_pointer_cast<JointLimitsAccelerationConstraint>(constraint)){
            constraint = joint_limits_constraint;
            n_replaced++;
        }
    }
    if(n_replaced != 1)
        throw std::runtime_error("AccelerationSceneReducedTSIDViability: expected exactly one JointLimitsAccelerationConstraint "
                                 "in AccelerationSceneReducedTSID, found " + std::to_string(n_replaced)
                                 + ". Has the base scene changed?");
}

} // namespace wbc
