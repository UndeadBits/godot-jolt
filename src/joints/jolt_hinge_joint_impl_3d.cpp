#include "jolt_hinge_joint_impl_3d.hpp"

#include "objects/jolt_body_impl_3d.hpp"
#include "spaces/jolt_space_3d.hpp"

namespace {

constexpr double DEFAULT_BIAS = 0.3;
constexpr double DEFAULT_LIMIT_BIAS = 0.3;
constexpr double DEFAULT_SOFTNESS = 0.9;
constexpr double DEFAULT_RELAXATION = 1.0;

} // namespace

JoltHingeJointImpl3D::JoltHingeJointImpl3D(
	JoltBodyImpl3D* p_body_a,
	JoltBodyImpl3D* p_body_b,
	const Transform3D& p_local_ref_a,
	const Transform3D& p_local_ref_b,
	bool p_lock
)
	: JoltJointImpl3D(p_body_a, p_body_b, p_local_ref_a, p_local_ref_b, p_lock) {
	rebuild(p_lock);
}

JoltHingeJointImpl3D::JoltHingeJointImpl3D(
	JoltBodyImpl3D* p_body_a,
	[[maybe_unused]] const Transform3D& p_local_ref_a,
	const Transform3D& p_local_ref_b,
	bool p_lock
)
	: JoltJointImpl3D(p_body_a, p_local_ref_b) {
	rebuild(p_lock);
}

double JoltHingeJointImpl3D::get_param(PhysicsServer3D::HingeJointParam p_param) const {
	switch (p_param) {
		case PhysicsServer3D::HINGE_JOINT_BIAS: {
			return DEFAULT_BIAS;
		}
		case PhysicsServer3D::HINGE_JOINT_LIMIT_UPPER: {
			return limit_upper;
		}
		case PhysicsServer3D::HINGE_JOINT_LIMIT_LOWER: {
			return limit_lower;
		}
		case PhysicsServer3D::HINGE_JOINT_LIMIT_BIAS: {
			return DEFAULT_LIMIT_BIAS;
		}
		case PhysicsServer3D::HINGE_JOINT_LIMIT_SOFTNESS: {
			return DEFAULT_SOFTNESS;
		}
		case PhysicsServer3D::HINGE_JOINT_LIMIT_RELAXATION: {
			return DEFAULT_RELAXATION;
		}
		case PhysicsServer3D::HINGE_JOINT_MOTOR_TARGET_VELOCITY: {
			return motor_target_velocity;
		}
		case PhysicsServer3D::HINGE_JOINT_MOTOR_MAX_IMPULSE: {
			return motor_max_impulse;
		}
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled hinge joint parameter: '%d'", p_param));
		}
	}
}

void JoltHingeJointImpl3D::set_param(
	PhysicsServer3D::HingeJointParam p_param,
	double p_value,
	bool p_lock
) {
	switch (p_param) {
		case PhysicsServer3D::HINGE_JOINT_BIAS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_BIAS)) {
				WARN_PRINT(vformat(
					"Hinge joint bias is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
				));
			}
		} break;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_UPPER: {
			limit_upper = p_value;
			rebuild(p_lock);
		} break;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_LOWER: {
			limit_lower = p_value;
			rebuild(p_lock);
		} break;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_BIAS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_LIMIT_BIAS)) {
				WARN_PRINT(vformat(
					"Hinge joint bias limit is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
				));
			}
		} break;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_SOFTNESS: {
			if (!Math::is_equal_approx(p_value, DEFAULT_SOFTNESS)) {
				WARN_PRINT(vformat(
					"Hinge joint softness is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
				));
			}
		} break;
		case PhysicsServer3D::HINGE_JOINT_LIMIT_RELAXATION: {
			if (!Math::is_equal_approx(p_value, DEFAULT_RELAXATION)) {
				WARN_PRINT(vformat(
					"Hinge joint relaxation is not supported by Godot Jolt. "
					"Any such value will be ignored. "
					"This joint connects %s.",
					owners_to_string()
				));
			}
		} break;
		case PhysicsServer3D::HINGE_JOINT_MOTOR_TARGET_VELOCITY: {
			motor_target_velocity = p_value;

			if (auto* constraint = static_cast<JPH::HingeConstraint*>(jolt_ref.GetPtr())) {
				constraint->SetTargetAngularVelocity((float)p_value);
			}
		} break;
		case PhysicsServer3D::HINGE_JOINT_MOTOR_MAX_IMPULSE: {
			motor_max_impulse = p_value;

			const double max_torque = estimate_max_motor_torque();

			if (auto* constraint = static_cast<JPH::HingeConstraint*>(jolt_ref.GetPtr())) {
				JPH::MotorSettings& motor_settings = constraint->GetMotorSettings();
				motor_settings.mMinTorqueLimit = (float)-max_torque;
				motor_settings.mMaxTorqueLimit = (float)max_torque;
			}
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled hinge joint parameter: '%d'", p_param));
		} break;
	}
}

bool JoltHingeJointImpl3D::get_flag(PhysicsServer3D::HingeJointFlag p_flag) const {
	switch (p_flag) {
		case PhysicsServer3D::HINGE_JOINT_FLAG_USE_LIMIT: {
			return use_limits;
		} break;
		case PhysicsServer3D::HINGE_JOINT_FLAG_ENABLE_MOTOR: {
			return motor_enabled;
		} break;
		default: {
			ERR_FAIL_D_MSG(vformat("Unhandled hinge joint flag: '%d'", p_flag));
		} break;
	}
}

void JoltHingeJointImpl3D::set_flag(
	PhysicsServer3D::HingeJointFlag p_flag,
	bool p_enabled,
	bool p_lock
) {
	switch (p_flag) {
		case PhysicsServer3D::HINGE_JOINT_FLAG_USE_LIMIT: {
			use_limits = p_enabled;
			rebuild(p_lock);
		} break;
		case PhysicsServer3D::HINGE_JOINT_FLAG_ENABLE_MOTOR: {
			motor_enabled = p_enabled;

			if (auto* constraint = static_cast<JPH::HingeConstraint*>(jolt_ref.GetPtr())) {
				constraint->SetMotorState(
					motor_enabled ? JPH::EMotorState::Velocity : JPH::EMotorState::Off
				);
			}
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled hinge joint flag: '%d'", p_flag));
		} break;
	}
}

void JoltHingeJointImpl3D::rebuild(bool p_lock) {
	destroy();

	JoltSpace3D* space = get_space();

	if (space == nullptr) {
		return;
	}

	JPH::BodyID body_ids[2] = {body_a->get_jolt_id()};
	int32_t body_count = 1;

	if (body_b != nullptr) {
		body_ids[1] = body_b->get_jolt_id();
		body_count = 2;
	}

	const JoltWritableBodies3D bodies = space->write_bodies(body_ids, body_count, p_lock);

	JPH::HingeConstraintSettings constraint_settings;

	float axis_shift = 0.0f;

	if (!use_limits || limit_lower > limit_upper) {
		constraint_settings.mLimitsMin = -JPH::JPH_PI;
		constraint_settings.mLimitsMax = JPH::JPH_PI;
	} else {
		const double limit_middle = Math::lerp(limit_lower, limit_upper, 0.5);

		axis_shift = (float)limit_middle;

		const auto extent = (float)Math::abs(limit_middle - limit_lower);
		constraint_settings.mLimitsMin = -extent;
		constraint_settings.mLimitsMax = extent;
	}

	const Basis shifted_basis = world_ref.basis.rotated(Vector3(axis_shift, 0.0f, 0.0f));

	constraint_settings.mSpace = JPH::EConstraintSpace::WorldSpace;
	constraint_settings.mPoint1 = to_jolt(world_ref.origin);
	constraint_settings.mHingeAxis1 = to_jolt(-world_ref.basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mNormalAxis1 = to_jolt(world_ref.basis.get_column(Vector3::AXIS_X));
	constraint_settings.mPoint2 = to_jolt(world_ref.origin);
	constraint_settings.mHingeAxis2 = to_jolt(-shifted_basis.get_column(Vector3::AXIS_Z));
	constraint_settings.mNormalAxis2 = to_jolt(shifted_basis.get_column(Vector3::AXIS_X));

	const double max_torque = estimate_max_motor_torque();

	constraint_settings.mMotorSettings.mMinTorqueLimit = (float)-max_torque;
	constraint_settings.mMotorSettings.mMaxTorqueLimit = (float)max_torque;

	if (body_b != nullptr) {
		const JoltWritableBody3D jolt_body_a = bodies[0];
		ERR_FAIL_COND(jolt_body_a.is_invalid());

		const JoltWritableBody3D jolt_body_b = bodies[1];
		ERR_FAIL_COND(jolt_body_b.is_invalid());

		jolt_ref = constraint_settings.Create(*jolt_body_a, *jolt_body_b);
	} else {
		const JoltWritableBody3D jolt_body_a = bodies[0];
		ERR_FAIL_COND(jolt_body_a.is_invalid());

		jolt_ref = constraint_settings.Create(*jolt_body_a, JPH::Body::sFixedToWorld);
	}

	space->add_joint(this);

	auto* constraint = static_cast<JPH::HingeConstraint*>(jolt_ref.GetPtr());

	constraint->SetTargetAngularVelocity((float)motor_target_velocity);

	if (motor_enabled) {
		constraint->SetMotorState(JPH::EMotorState::Velocity);
	}
}

double JoltHingeJointImpl3D::estimate_max_motor_torque() const {
	// HACK(mihe): This will break if the physics time step changes in any way during the
	// lifetime of this joint, but it can't really be fixed since Godot only provides a max
	// impulse and not a max force. As far as I can tell this is similarly broken in Godot
	// Physics as well, so at least we're being consistent.
	return motor_max_impulse / estimate_physics_step();
}
